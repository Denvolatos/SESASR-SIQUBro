"""
ROS 2 node: EKF localization with landmarks (Task 1) and optional sensor fusion (Task 2).

Prediction runs in a 20 Hz timer, each landmark in a message is one update.
With fusion:=true the state also has v and w, corrected by /odom and /imu.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from landmark_msgs.msg import LandmarkArray
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
import tf_transformations
import yaml

from lab04_pkg import probabilistic_models as pm
from lab04_pkg.ekf import RobotEKF


def load_landmarks(path):
    """Landmark coordinates as {id: (x, y)}; z is not used, range and bearing are planar."""
    with open(path, 'r') as f:
        data = yaml.safe_load(f)['landmarks']
    return {i: (x, y) for i, x, y in zip(data['id'], data['x'], data['y'])}


class EkfNode(Node):
    # ---------- 1. Setup: parameters, filter, topics ----------
    def __init__(self):
        super().__init__('ekf_node')
        default_file = os.path.join(get_package_share_directory('turtlebot3_perception'),
                                    'config', 'landmarks.yaml')
        p = {
            'fusion': False,
            'prediction_rate': 20.0,      # [Hz]
            'landmarks_topic': '/landmarks',
            'landmarks_file': default_file,
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'init_pose': [0.0, 0.0, 0.0],  # [m, m, rad]
            'init_std': [0.1, 0.1, 0.1],   # [m, m, rad]
            'std_v': 0.05,                # [m/s] process noise on the velocities
            'std_w': 0.1,                 # [rad/s]
            'std_range': 0.05,            # [m] measurement noise
            'std_bearing_deg': 2.0,       # [deg]
            'std_odom_v': 0.02,           # [m/s]
            'std_odom_w': 0.05,           # [rad/s]
            'std_imu_w': 0.02,            # [rad/s]
        }
        for name, value in p.items():
            self.declare_parameter(name, value)
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        self.fusion = get('fusion')
        self.landmarks = load_landmarks(get('landmarks_file'))
        self.map_frame, self.base_frame = get('map_frame'), get('base_frame')

        self.M = np.diag([get('std_v')**2, get('std_w')**2])
        self.Q_landmark = np.diag([get('std_range')**2, math.radians(get('std_bearing_deg'))**2])
        self.Q_odom = np.diag([get('std_odom_v')**2, get('std_odom_w')**2])
        self.Q_imu = np.array([[get('std_imu_w')**2]])

        pose, std = list(get('init_pose')), list(get('init_std'))
        if self.fusion:   # the velocities start at zero and are known well: the robot is standing
            mu, sigma = pose + [0.0, 0.0], [s**2 for s in std] + [0.01, 0.01]
        else:
            mu, sigma = pose, [s**2 for s in std]
        self.ekf = RobotEKF(mu, np.diag(sigma))

        self.odom = None       # latest (v, w) from /odom
        self.imu = None        # latest w from /imu
        self.new_odom = self.new_imu = False
        self.last_time = None

        self.pub = self.create_publisher(Odometry, '/ekf', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(LandmarkArray, get('landmarks_topic'), self.landmark_callback, 10)
        if self.fusion:
            self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile_sensor_data)
        self.create_timer(1.0 / get('prediction_rate'), self.prediction_step)
        self.get_logger().info(f'EKF started: state size {len(mu)}, '
                               f'{len(self.landmarks)} landmarks')

    # ---------- 2. Sensors: remember the latest velocities ----------
    def odom_callback(self, msg):
        self.odom = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.new_odom = True

    def imu_callback(self, msg):
        self.imu = msg.angular_velocity.z
        self.new_imu = True

    # ---------- 3. Prediction at a fixed rate ----------
    def prediction_step(self):
        now = self.get_clock().now()
        if self.last_time is None or self.odom is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:          # the simulation clock has not moved
            return

        if self.fusion:
            mu = self.ekf.mu
            self.ekf.predict(pm.extended_g(mu, dt), pm.extended_G(mu, dt),
                             pm.extended_V(mu, dt), self.M)
            self.fuse_velocities()
        else:
            mu, u = self.ekf.mu, self.odom
            self.ekf.predict(pm.motion_g(mu, u, dt), pm.motion_G(mu, u, dt),
                             pm.motion_V(mu, u, dt), self.M)
        self.publish_state()

    def fuse_velocities(self):
        """Task 2: correct v and w once per prediction with the latest /odom and /imu."""
        if self.new_odom:
            self.ekf.update(self.odom, pm.odom_h(self.ekf.mu), pm.odom_H(), self.Q_odom)
            self.new_odom = False
        if self.new_imu:
            self.ekf.update([self.imu], pm.imu_h(self.ekf.mu), pm.imu_H(), self.Q_imu)
            self.new_imu = False

    # ---------- 4. Update: one per landmark in the message ----------
    def landmark_callback(self, msg):
        updated = False
        for lm in msg.landmarks:
            if lm.id not in self.landmarks:
                self.get_logger().warn(f'unknown landmark id {lm.id}', once=True)
                continue
            pos = self.landmarks[lm.id]
            mu = self.ekf.mu
            self.ekf.update([lm.range, lm.bearing], pm.landmark_h(mu, pos),
                            pm.landmark_H(mu, pos, dim=len(mu)), self.Q_landmark, angle_idx=1)
            updated = True
        if updated:
            self.publish_state()

    # ---------- 5. Output ----------
    def publish_state(self):
        mu, S = self.ekf.mu, self.ekf.Sigma
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(mu[0])
        msg.pose.pose.position.y = float(mu[1])
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, float(mu[2]))
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = q[0], q[1]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[2], q[3]
        cov = [0.0] * 36   # order x, y, z, roll, pitch, yaw
        for a, i in ((0, 0), (1, 1), (2, 5)):
            for b, j in ((0, 0), (1, 1), (2, 5)):
                cov[6 * i + j] = float(S[a, b])
        msg.pose.covariance = cov
        if self.fusion:
            msg.twist.twist.linear.x, msg.twist.twist.angular.z = float(mu[3]), float(mu[4])
        self.pub.publish(msg)


def main(args=None):
    # handle Ctrl+C ourselves, so that the node can shut down cleanly
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = EkfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
