import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations


class BumpAndGo(Node):
    # ---------- 1. Setup: parameters, topics, timer ----------
    def __init__(self):
        super().__init__('bump_and_go')
        self.declare_parameter('max_linear_vel', 0.22)      # [m/s]  TurtleBot3 limit
        self.declare_parameter('max_angular_vel', 1.5)      # [rad/s] suggested limit
        self.declare_parameter('control_frequency', 10.0)   # [Hz]
        self.declare_parameter('obstacle_threshold', 0.5)   # [m]
        self.declare_parameter('stop_at_goal', True)        # False on the real robot
        self.declare_parameter('goal_x', 6.5)               # [m] green cube in the maze
        self.declare_parameter('goal_y', 3.0)
        self.declare_parameter('goal_tolerance', 0.8)       # [m]
        self.declare_parameter('rotation_tolerance_deg', 5.0)  # [deg] turn tolerance
        self.declare_parameter('min_angular_vel', 0.3)      # [rad/s] motor dead zone
        self.declare_parameter('sensor_timeout_sec', 1.0)  # [s] stop if sensors become stale
        self.declare_parameter('turn_angle_deg', 90.0)      # [deg] largest turn after a bump
        self.declare_parameter('snap_to_axes', True)        # round the turn to the maze axes
        self.declare_parameter('turn_until_clear', False)   # stop turning once the front is free
        self.declare_parameter('clear_margin', 0.3)         # [m] free = threshold + margin
        self.v_max = self.get_parameter('max_linear_vel').value
        self.w_max = self.get_parameter('max_angular_vel').value
        self.threshold = self.get_parameter('obstacle_threshold').value
        self.stop_at_goal = self.get_parameter('stop_at_goal').value
        self.goal = (self.get_parameter('goal_x').value, self.get_parameter('goal_y').value)
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.rot_tol = math.radians(self.get_parameter('rotation_tolerance_deg').value)
        self.w_min = self.get_parameter('min_angular_vel').value
        self.sensor_timeout = self.get_parameter('sensor_timeout_sec').value
        self.turn_angle = math.radians(self.get_parameter('turn_angle_deg').value)
        self.snap = self.get_parameter('snap_to_axes').value
        self.until_clear = self.get_parameter('turn_until_clear').value
        self.clear_margin = self.get_parameter('clear_margin').value
        period = 1.0 / self.get_parameter('control_frequency').value
        if self.sensor_timeout <= 0:
            raise ValueError('sensor_timeout_sec must be positive')

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
        self.state = 'GO'          # 'GO', 'ROTATE' or 'DONE'
        self.target_yaw = 0.0
        self.turn_start_yaw = 0.0

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

    # ---------- 3. LiDAR: closest obstacle in a direction ----------
    def sector_distance(self, center_deg, half_width_deg, use_median=False):
        """Return the nearest (or median) usable range, or None when this sector has no data."""
        s = self.scan
        values = []
        if not math.isfinite(s.range_max) or s.range_max <= s.range_min:
            return None
        for i, r in enumerate(s.ranges):
            angle = math.degrees(s.angle_min + i * s.angle_increment)
            diff = (angle - center_deg + 180.0) % 360.0 - 180.0   # angle difference in [-180, 180)
            if abs(diff) <= half_width_deg:
                if math.isinf(r) and r > 0:
                    values.append(s.range_max)  # no return within the sensor's range
                elif math.isfinite(r) and s.range_min <= r <= s.range_max:
                    values.append(r)
        if not values:
            return None
        if use_median:                       # how open the side is, not its closest corner
            return sorted(values)[len(values) // 2]
        return min(values)

    # ---------- 4. Control loop: decide and publish ----------
    def control_loop(self):
        cmd = Twist()
        if not self.sensors_ready():
            self.cmd_pub.publish(cmd)
            self.last_cmd = cmd
            return

        front = self.sector_distance(0.0, 15.0)
        if front is None:
            self.cmd_pub.publish(cmd)
            self.last_cmd = cmd
            return

        if (self.stop_at_goal and self.state != 'DONE'
                and math.hypot(self.x - self.goal[0], self.y - self.goal[1]) < self.goal_tol):
            self.state = 'DONE'
            self.get_logger().info(f'goal reached at x={self.x:.2f} y={self.y:.2f}')

        if self.state == 'GO':
            if front > self.threshold:
                cmd.linear.x = self.v_max                       # Go
            else:                                               # Bump
                left = self.sector_distance(90.0, 30.0, use_median=True)
                right = self.sector_distance(-90.0, 30.0, use_median=True)
                if left is None or right is None:
                    self.cmd_pub.publish(cmd)
                    self.last_cmd = cmd
                    return
                turn = self.turn_angle if left > right else -self.turn_angle
                self.target_yaw = self.yaw + turn
                if self.snap:                   # snap to the maze axes (0, 90, 180, 270 deg)
                    self.target_yaw = round(self.target_yaw / (math.pi / 2)) * (math.pi / 2)
                self.turn_start_yaw = self.yaw
                self.state = 'ROTATE'
                self.get_logger().info(
                    f'wall at {front:.2f} m (left {left:.2f}, right {right:.2f}) -> turn '
                    f'{"left" if turn > 0 else "right"}')

        elif self.state == 'ROTATE':
            error = math.atan2(math.sin(self.target_yaw - self.yaw),
                               math.cos(self.target_yaw - self.yaw))
            turned = abs(math.atan2(math.sin(self.yaw - self.turn_start_yaw),
                                    math.cos(self.yaw - self.turn_start_yaw)))
            front_clear = (self.until_clear and turned > math.radians(20.0)
                           and front > self.threshold + self.clear_margin)
            if abs(error) < self.rot_tol or front_clear:
                self.state = 'GO'
            else:
                w = max(-self.w_max, min(self.w_max, 2.0 * error))   # P controller
                if abs(w) < self.w_min:                              # dead zone of the real motors
                    w = self.w_min if w > 0 else -self.w_min
                cmd.angular.z = w

        self.cmd_pub.publish(cmd)   # in 'DONE' cmd stays zero -> robot stops
        self.last_cmd = cmd

    # ---------- 5. Diagnostics and safe stop ----------
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
        front = self.sector_distance(0.0, 15.0)
        if front is None:
            zeros = sum(1 for r in self.scan.ranges if r == 0.0)
            self.get_logger().warn(
                f'no usable front LiDAR data ({zeros}/{len(self.scan.ranges)} rays are 0.0); '
                'commanding zero velocity')
            return
        self.get_logger().info(
            f'state={self.state} front={front:.2f} m  '
            f'v={self.last_cmd.linear.x:.2f} m/s  w={self.last_cmd.angular.z:.2f} rad/s  '
            f'odom={"OK" if self.got_odom else "MISSING"}')

    def stop_robot(self):
        """Send zero velocity, so the robot does not keep the last command after Ctrl+C."""
        for _ in range(3):
            self.cmd_pub.publish(Twist())


def main(args=None):
    # handle Ctrl+C ourselves, so ROS is still running when we send the stop command
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = BumpAndGo()
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
