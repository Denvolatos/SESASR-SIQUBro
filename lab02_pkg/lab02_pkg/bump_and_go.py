import math
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
        self.declare_parameter('rotation_tolerance_deg', 5.0)  # [deg] turn is finished below this error
        self.declare_parameter('min_angular_vel', 0.3)      # [rad/s] real motors do not move below this
        self.v_max = self.get_parameter('max_linear_vel').value
        self.w_max = self.get_parameter('max_angular_vel').value
        self.threshold = self.get_parameter('obstacle_threshold').value
        self.stop_at_goal = self.get_parameter('stop_at_goal').value
        self.goal = (self.get_parameter('goal_x').value, self.get_parameter('goal_y').value)
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.rot_tol = math.radians(self.get_parameter('rotation_tolerance_deg').value)
        self.w_min = self.get_parameter('min_angular_vel').value
        period = 1.0 / self.get_parameter('control_frequency').value

        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(period, self.control_loop)
        self.create_timer(2.0, self.status_log)             # diagnostics every 2 s

        self.scan = None
        self.got_odom = False
        self.last_cmd = Twist()
        self.x = self.y = self.yaw = 0.0
        self.state = 'GO'          # 'GO', 'ROTATE' or 'DONE'
        self.target_yaw = 0.0

    # ---------- 2. Sensors: remember the latest data ----------
    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        self.got_odom = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    # ---------- 3. LiDAR: closest obstacle in a direction ----------
    def sector_distance(self, center_deg, half_width_deg):
        """Smallest valid range within +-half_width_deg of center_deg (0 = front, +90 = left)."""
        s = self.scan
        values = []
        for i, r in enumerate(s.ranges):
            angle = math.degrees(s.angle_min + i * s.angle_increment)
            diff = (angle - center_deg + 180.0) % 360.0 - 180.0   # angle difference in [-180, 180)
            if abs(diff) <= half_width_deg:
                if math.isinf(r) or r > s.range_max:
                    values.append(s.range_max)                      # nothing seen -> free
                elif r >= s.range_min:
                    values.append(r)                                # valid measurement
        return min(values) if values else s.range_max

    # ---------- 4. Control loop: decide and publish ----------
    def control_loop(self):
        if self.scan is None:
            return
        cmd = Twist()

        if (self.stop_at_goal and self.state != 'DONE'
                and math.hypot(self.x - self.goal[0], self.y - self.goal[1]) < self.goal_tol):
            self.state = 'DONE'
            self.get_logger().info(f'goal reached at x={self.x:.2f} y={self.y:.2f}')

        if self.state == 'GO':
            front = self.sector_distance(0.0, 15.0)
            if front > self.threshold:
                cmd.linear.x = self.v_max                       # Go
            else:                                               # Bump
                left = self.sector_distance(90.0, 30.0)
                right = self.sector_distance(-90.0, 30.0)
                turn = math.pi / 2 if left > right else -math.pi / 2
                # snap to the maze axes (0, 90, 180, 270 deg)
                self.target_yaw = round((self.yaw + turn) / (math.pi / 2)) * (math.pi / 2)
                self.state = 'ROTATE'
                self.get_logger().info(
                    f'wall at {front:.2f} m (left {left:.2f}, right {right:.2f}) -> turn '
                    f'{"left" if turn > 0 else "right"}')

        elif self.state == 'ROTATE':
            error = math.atan2(math.sin(self.target_yaw - self.yaw),
                               math.cos(self.target_yaw - self.yaw))
            if abs(error) < self.rot_tol:
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
        if self.scan is None:
            self.get_logger().warn('waiting for /scan ... (no LiDAR data received yet)')
            return
        front = self.sector_distance(0.0, 15.0)
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
