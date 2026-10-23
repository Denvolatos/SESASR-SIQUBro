"""Simulate the motor dead zone of the real TurtleBot3 in Gazebo.

Controller --/cmd_vel_raw--> [this node] --/cmd_vel--> simulated robot
Angular velocities smaller than `angular_dead_zone` are set to 0, like a real
motor that cannot overcome friction at very low speed.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DeadZoneFilter(Node):
    def __init__(self):
        super().__init__('dead_zone_filter')
        self.declare_parameter('angular_dead_zone', 0.25)   # [rad/s]
        self.dz = self.get_parameter('angular_dead_zone').value
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.callback, 10)

    def callback(self, msg):
        if abs(msg.angular.z) < self.dz:
            msg.angular.z = 0.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(DeadZoneFilter())


if __name__ == '__main__':
    main()
