"""ROS 2 node: reads /scan and /odom, runs the Bump&Go controller, publishes /cmd_vel.

All decisions are in controller.py and all LiDAR processing in lidar_utils.py;
this file only connects them to ROS (parameters, topics, timers, safe stop).
"""

import math
import time
from dataclasses import fields

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations

from lab02_pkg.controller import BumpAndGoController, ControllerConfig
from lab02_pkg.lidar_utils import count_zero_rays, sector_median, sector_min

FRONT = (0.0, 15.0)     # sector (centre, half width) in degrees
LEFT = (90.0, 30.0)
RIGHT = (-90.0, 30.0)


class BumpAndGoNode(Node):
    # ---------- 1. Setup: parameters, topics, timers ----------
    def __init__(self):
        super().__init__('bump_and_go')
        # every ControllerConfig field is a ROS parameter with the same name and default
        values = {}
        for f in fields(ControllerConfig):
            self.declare_parameter(f.name, f.default)
            values[f.name] = self.get_parameter(f.name).value
        self.controller = BumpAndGoController(ControllerConfig(**values))

        self.declare_parameter('control_frequency', 10.0)   # [Hz]
        self.declare_parameter('sensor_timeout_sec', 1.0)   # [s] stop if sensors become stale
        self.sensor_timeout = self.get_parameter('sensor_timeout_sec').value
        if self.sensor_timeout <= 0:
            raise ValueError('sensor_timeout_sec must be positive')
        period = 1.0 / self.get_parameter('control_frequency').value

        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(period, self.control_loop)
        self.create_timer(2.0, self.status_log)             # diagnostics every 2 s

        self.scan = None
        self.got_odom = False
        self.last_scan_time = None
        self.last_odom_time = None
        self.last_cmd = Twist()
        self.x = self.y = self.yaw = 0.0

    # ---------- 2. Sensors: remember the latest data ----------
    def scan_callback(self, msg):
        self.scan = msg
        self.last_scan_time = time.monotonic()

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return
        self.got_odom = True
        self.x, self.y, self.yaw = x, y, yaw
        self.last_odom_time = time.monotonic()

    def sensors_ready(self):
        """Require recent LiDAR and odometry before commanding movement."""
        now = time.monotonic()
        return (self.scan is not None
                and self.got_odom
                and self.last_scan_time is not None
                and self.last_odom_time is not None
                and now - self.last_scan_time <= self.sensor_timeout
                and now - self.last_odom_time <= self.sensor_timeout)

    # ---------- 3. Control loop: sensors -> controller -> /cmd_vel ----------
    def control_loop(self):
        cmd = Twist()
        if self.sensors_ready():
            v, w = self.controller.step(
                self.x, self.y, self.yaw,
                front=sector_min(self.scan, *FRONT),
                left=sector_median(self.scan, *LEFT),
                right=sector_median(self.scan, *RIGHT))
            cmd.linear.x, cmd.angular.z = v, w
            if self.controller.event:
                self.get_logger().info(self.controller.event)
        self.cmd_pub.publish(cmd)   # zero when sensors are missing or the goal is reached
        self.last_cmd = cmd

    # ---------- 4. Diagnostics and safe stop ----------
    def status_log(self):
        """Print what the controller sees and sends, to debug the real robot."""
        if not self.sensors_ready():
            now = time.monotonic()
            ages = []
            for name, t in (('/scan', self.last_scan_time), ('/odom', self.last_odom_time)):
                ages.append(f'{name}=never' if t is None else f'{name}={now - t:.1f}s ago')
            self.get_logger().warn(
                f'sensor data missing or stale ({", ".join(ages)}); commanding zero velocity')
            return
        front = sector_min(self.scan, *FRONT)
        if front is None:
            self.get_logger().warn(
                f'no usable front LiDAR data ({count_zero_rays(self.scan)}/'
                f'{len(self.scan.ranges)} rays are 0.0); commanding zero velocity')
            return
        self.get_logger().info(
            f'state={self.controller.state} front={front:.2f} m  '
            f'v={self.last_cmd.linear.x:.2f} m/s  w={self.last_cmd.angular.z:.2f} rad/s  '
            f'odom={"OK" if self.got_odom else "MISSING"}')

    def stop_robot(self):
        """Send zero velocity, so the robot does not keep the last command after Ctrl+C."""
        for _ in range(3):
            self.cmd_pub.publish(Twist())


def main(args=None):
    # handle Ctrl+C ourselves, so ROS is still running when we send the stop command
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BumpAndGoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
            node.get_logger().info('stopped: zero velocity sent')
        node.destroy_node()
        rclpy.try_shutdown()
