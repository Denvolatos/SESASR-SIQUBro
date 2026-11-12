"""ROS 2 adapter for Jin's independent Task 1 EKF implementation."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from landmark_msgs.msg import LandmarkArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
import tf_transformations
import yaml

from lab04_pkg.jin_ekf import JinEKF


def read_landmarks(path):
    """Read the course landmark map and check the planar coordinate arrays."""
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)["landmarks"]
    ids, xs, ys = data["id"], data["x"], data["y"]
    if len(ids) != len(xs) or len(ids) != len(ys) or len(set(ids)) != len(ids):
        raise ValueError("landmark map has mismatched or duplicate IDs")
    return {int(i): (float(x), float(y)) for i, x, y in zip(ids, xs, ys)}


class JinEkfNode(Node):
    """Predict at 20 Hz and correct once for every observed known landmark."""

    def __init__(self):
        super().__init__("jin_ekf_node")
        map_path = os.path.join(
            get_package_share_directory("turtlebot3_perception"),
            "config",
            "landmarks.yaml",
        )
        defaults = {
            "landmarks_file": map_path,
            "landmarks_topic": "/landmarks",
            "output_topic": "/ekf_jin",
            "map_frame": "map",
            "base_frame": "base_footprint",
            "init_pose": [0.0, 0.0, 0.0],
            "init_std": [0.1, 0.1, 0.1],
            "prediction_rate": 20.0,
            "max_odom_age": 0.5,
            "std_v": 0.05,
            "std_w": 0.1,
            "std_range": 0.05,
            "std_bearing_deg": 2.0,
            "innovation_gate": 0.0,  # 0 disables rejection
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        params = {key: self.get_parameter(key).value for key in defaults}
        if params["prediction_rate"] <= 0 or params["max_odom_age"] <= 0:
            raise ValueError(
                "prediction_rate and max_odom_age must be positive"
            )
        self.landmarks = read_landmarks(params["landmarks_file"])
        self.filter = JinEKF(
            params["init_pose"],
            params["init_std"],
            [params["std_v"], params["std_w"]],
            [params["std_range"], math.radians(params["std_bearing_deg"])],
            params["innovation_gate"],
        )
        self.map_frame = params["map_frame"]
        self.base_frame = params["base_frame"]
        self.last_velocity = None
        self.last_odom_time = None
        self.last_prediction_time = None
        self.publisher = self.create_publisher(
            Odometry, params["output_topic"], 10
        )
        self.create_subscription(
            Odometry, "/odom", self.on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            LandmarkArray, params["landmarks_topic"], self.on_landmarks, 10
        )
        self.create_timer(1.0 / params["prediction_rate"], self.on_timer)
        self.max_odom_age = params["max_odom_age"]
        self.get_logger().info(
            f"Jin EKF: {len(self.landmarks)} landmarks, "
            f"output {params['output_topic']}"
        )

    def on_odom(self, message):
        """Keep the latest measured velocity for the next prediction step."""
        self.last_velocity = (
            message.twist.twist.linear.x,
            message.twist.twist.angular.z,
        )
        self.last_odom_time = self.get_clock().now()

    def on_timer(self):
        """Advance using elapsed ROS time, including simulation time."""
        now = self.get_clock().now()
        previous = self.last_prediction_time
        self.last_prediction_time = now
        if previous is None or self.last_velocity is None:
            return
        dt = (now - previous).nanoseconds * 1e-9
        age = (now - self.last_odom_time).nanoseconds * 1e-9
        if dt <= 0 or age < 0 or age > self.max_odom_age:
            return
        if self.filter.predict(self.last_velocity, dt):
            self.publish_estimate()

    def on_landmarks(self, message):
        """Use each known landmark as a separate EKF correction."""
        accepted = False
        for landmark in message.landmarks:
            position = self.landmarks.get(landmark.id)
            if position is None:
                continue
            if self.filter.update_landmark(
                (landmark.range, landmark.bearing), position
            ):
                accepted = True
        if accepted:
            self.publish_estimate()

    def publish_estimate(self):
        """Publish pose and its planar covariance in nav_msgs/Odometry."""
        state, covariance = self.filter.state, self.filter.covariance
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = float(state[0])
        message.pose.pose.position.y = float(state[1])
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, float(state[2]))
        message.pose.pose.orientation.x = float(q[0])
        message.pose.pose.orientation.y = float(q[1])
        message.pose.pose.orientation.z = float(q[2])
        message.pose.pose.orientation.w = float(q[3])
        for row, ros_row in enumerate((0, 1, 5)):
            for col, ros_col in enumerate((0, 1, 5)):
                message.pose.covariance[6 * ros_row + ros_col] = float(
                    covariance[row, col]
                )
        self.publisher.publish(message)


def main(args=None):
    """Run the node until shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = JinEkfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
