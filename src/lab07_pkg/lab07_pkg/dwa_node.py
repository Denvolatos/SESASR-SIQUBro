# Copyright 2026 SESASR Group 203 (SIQUBro)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


"""
ROS 2 Humble node for Task 1 (static goal) and Task 2/3 (dynamic following) DWA.

Runs at 15 Hz via a ROS timer (no blocking while loops). Core math lives in
scan_utils.py, dwa.py, and navigation.py without rclpy dependencies.
"""

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

try:
    from landmark_msgs.msg import LandmarkArray
except ImportError:
    LandmarkArray = None

from lab07_pkg.dwa import DWAConfig
from lab07_pkg.navigation import (
    LandmarkObservation,
    NavConfig,
    NavigationManager,
    ScanData,
    yaw_from_quaternion,
)
from lab07_pkg.scan_utils import ScanConfig


class DwaNode(Node):
    """ROS 2 wrapper around NavigationManager and DWAController."""

    def __init__(self) -> None:
        """Declare ROS 2 parameters, interfaces, and the 15 Hz control timer."""
        super().__init__('dwa_node')

        defaults = {
            'mode': 'goal',
            'goal_source': 'topic',           # 'topic' (/goal_pose) or 'landmark' (one-shot tag)
            'landmark_goal_standoff': 0.5,    # [m] backoff from tag in goal/landmark mode
            'follow_source': 'dynamic_goal',  # 'dynamic_goal' (sim) or 'landmarks' (real)
            'control_rate': 15.0,
            'sim_time': 1.5,
            'time_granularity': 0.1,
            'v_samples': 10,
            'w_samples': 21,
            'max_lin_vel': 0.22,
            'min_lin_vel': 0.0,
            'max_ang_vel': 1.5,
            'min_ang_vel': -1.5,
            'max_lin_acc': 0.5,
            'max_ang_acc': 3.2,
            'robot_radius': 0.105,
            'weight_heading': 0.40,
            'weight_vel': 0.30,
            'weight_obs': 0.30,
            'weight_target': 0.35,
            'obstacle_max_dist': 1.0,
            'obs_norm_min_span': 0.05,
            'slowdown_dist_threshold': 1.2,
            'desired_target_dist': 0.8,
            'target_dist_sigma': 0.5,
            'camera_fov_half_rad': 0.576,
            'max_scan_range': 3.5,
            'min_valid_range': 0.05,
            'min_valid_ratio': 0.20,
            'max_nan_ratio': 0.95,
            'scan_median_filter': True,
            'num_ranges': 24,
            'nan_policy': 'min',
            'laser_offset_x': -0.032,
            'laser_offset_y': 0.0,
            'laser_offset_yaw': 0.0,
            'camera_offset_x': 0.0,
            'camera_offset_y': 0.0,
            'camera_offset_yaw': 0.0,
            'collision_tol': 0.18,
            'planning_margin': 0.10,
            'band_floor_buffer': 0.03,
            'enable_recovery_spin': False,
            'recovery_ang_vel': 0.4,
            'window_source': 'odom',
            'max_stamp_skew_sec': 0.5,
            'goal_dist_tol': 0.25,
            'max_num_steps': 1500,
            'feedback_rate': 50,
            'sensor_timeout_sec': 1.0,
            'target_loss_timeout_sec': 2.0,
            'target_jump_threshold': 2.5,
            'follow_use_trail': True,
            'trail_spacing': 0.10,
            'trail_reach_dist': 0.40,
            'stuck_time_sec': 2.0,
            'stuck_vel_thresh': 0.03,
            'stuck_dist_margin': 0.5,
            'recovery_exit_dist': 0.3,
            'recovery_exit_margin': 0.2,
            'recovery_max_sec': 5.0,
            'lost_dist': 3.0,
            'lost_dist_time_sec': 10.0,
            'target_id': -1,
            'require_target_id': False,
            'odom_frame': 'odom',
            'world_frame': 'world',
            'odom_origin_in_world': [0.0, 0.0, 0.0],
            'odom_topic': '/odom',
            'scan_topic': '/scan',
            'goal_topic': '/goal_pose',
            'dynamic_goal_topic': '/dynamic_goal_pose',
            'landmarks_topic': '/camera/landmarks',
            'cmd_vel_topic': '/cmd_vel',
            'result_topic': '/nav_result',
            'distance_topic': '/distance_to_goal',
        }
        for name, val in defaults.items():
            self.declare_parameter(name, val)

        def get(name):
            return self.get_parameter(name).value
        self.mode = str(get('mode'))
        self.goal_source = str(get('goal_source'))
        self.follow_source = str(get('follow_source'))

        control_rate = float(get('control_rate'))
        dt = 1.0 / control_rate

        dwa_cfg = DWAConfig(
            dt=dt,
            sim_time=float(get('sim_time')),
            time_granularity=float(get('time_granularity')),
            v_samples=int(get('v_samples')),
            w_samples=int(get('w_samples')),
            max_lin_vel=float(get('max_lin_vel')),
            min_lin_vel=float(get('min_lin_vel')),
            max_ang_vel=float(get('max_ang_vel')),
            min_ang_vel=float(get('min_ang_vel')),
            max_lin_acc=float(get('max_lin_acc')),
            max_ang_acc=float(get('max_ang_acc')),
            robot_radius=float(get('robot_radius')),
            collision_tol=float(get('collision_tol')),
            planning_margin=float(get('planning_margin')),
            band_floor_buffer=float(get('band_floor_buffer')),
            enable_recovery_spin=bool(get('enable_recovery_spin')),
            recovery_ang_vel=float(get('recovery_ang_vel')),
            weight_heading=float(get('weight_heading')),
            weight_vel=float(get('weight_vel')),
            weight_obs=float(get('weight_obs')),
            weight_target=float(get('weight_target')),
            obstacle_max_dist=float(get('obstacle_max_dist')),
            obs_norm_min_span=float(get('obs_norm_min_span')),
            follow_mode=(self.mode == 'follow'),
            slowdown_dist_threshold=float(get('slowdown_dist_threshold')),
            desired_target_dist=float(get('desired_target_dist')),
            target_dist_sigma=float(get('target_dist_sigma')),
            camera_fov_half_rad=float(get('camera_fov_half_rad')),
        )

        scan_cfg = ScanConfig(
            max_scan_range=float(get('max_scan_range')),
            min_valid_range=float(get('min_valid_range')),
            min_valid_ratio=float(get('min_valid_ratio')),
            max_nan_ratio=float(get('max_nan_ratio')),
            scan_median_filter=bool(get('scan_median_filter')),
            num_ranges=int(get('num_ranges')),
            nan_policy=str(get('nan_policy')),
            laser_offset_x=float(get('laser_offset_x')),
            laser_offset_y=float(get('laser_offset_y')),
            laser_offset_yaw=float(get('laser_offset_yaw')),
        )

        origin = list(get('odom_origin_in_world'))
        if len(origin) != 3:
            raise ValueError(f'odom_origin_in_world must have 3 elements, got {origin}')

        # In follow mode, default max_num_steps=0 disables arbitrary step timeout
        max_steps = int(get('max_num_steps'))
        if self.mode == 'follow':
            max_steps = 0

        nav_cfg = NavConfig(
            mode=self.mode,
            goal_source=self.goal_source,
            landmark_goal_standoff=float(get('landmark_goal_standoff')),
            goal_dist_tol=float(get('goal_dist_tol')),
            max_num_steps=max_steps,
            feedback_rate=int(get('feedback_rate')),
            sensor_timeout_sec=float(get('sensor_timeout_sec')),
            target_loss_timeout_sec=float(get('target_loss_timeout_sec')),
            target_jump_threshold=float(get('target_jump_threshold')),
            follow_use_trail=bool(get('follow_use_trail')),
            trail_spacing=float(get('trail_spacing')),
            trail_reach_dist=float(get('trail_reach_dist')),
            stuck_time_sec=float(get('stuck_time_sec')),
            stuck_vel_thresh=float(get('stuck_vel_thresh')),
            stuck_dist_margin=float(get('stuck_dist_margin')),
            recovery_exit_dist=float(get('recovery_exit_dist')),
            recovery_exit_margin=float(get('recovery_exit_margin')),
            recovery_max_sec=float(get('recovery_max_sec')),
            lost_dist=float(get('lost_dist')),
            lost_dist_time_sec=float(get('lost_dist_time_sec')),
            target_id=int(get('target_id')),
            require_target_id=bool(get('require_target_id')),
            window_source=str(get('window_source')),
            max_stamp_skew_sec=float(get('max_stamp_skew_sec')),
            camera_offset_x=float(get('camera_offset_x')),
            camera_offset_y=float(get('camera_offset_y')),
            camera_offset_yaw=float(get('camera_offset_yaw')),
            odom_frame=str(get('odom_frame')),
            world_frame=str(get('world_frame')),
            odom_origin_in_world=(float(origin[0]), float(origin[1]), float(origin[2])),
        )

        self.manager = NavigationManager(dwa_cfg, scan_cfg, nav_cfg)
        if self.manager.standoff_warning:
            self.get_logger().warn(self.manager.standoff_warning)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, str(get('cmd_vel_topic')), 10)
        self.result_pub = self.create_publisher(String, str(get('result_topic')), 10)
        self.dist_pub = self.create_publisher(Float32, str(get('distance_topic')), 10)

        # Core sensor subscriptions
        self.create_subscription(
            Odometry, str(get('odom_topic')), self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, str(get('scan_topic')), self.scan_callback, qos_profile_sensor_data
        )

        # Target subscriptions depending on mode and goal_source
        if self.mode == 'goal':
            if self.goal_source == 'topic':
                self.create_subscription(
                    PoseStamped, str(get('goal_topic')), self.goal_callback, 10
                )
            elif self.goal_source == 'landmark':
                if LandmarkArray is None:
                    raise RuntimeError(
                        'landmark_msgs is not installed; cannot subscribe to landmarks'
                    )
                self.create_subscription(
                    LandmarkArray,
                    str(get('landmarks_topic')),
                    self.landmarks_callback,
                    qos_profile_sensor_data,
                )
            else:
                raise ValueError(
                    f"goal_source must be 'topic' or 'landmark', got '{self.goal_source}'"
                )
        elif self.follow_source == 'dynamic_goal':
            self.create_subscription(
                Odometry,
                str(get('dynamic_goal_topic')),
                self.dynamic_goal_callback,
                qos_profile_sensor_data,
            )
        elif self.follow_source == 'landmarks':
            if LandmarkArray is None:
                raise RuntimeError('landmark_msgs is not installed; cannot subscribe to landmarks')
            self.create_subscription(
                LandmarkArray,
                str(get('landmarks_topic')),
                self.landmarks_callback,
                qos_profile_sensor_data,
            )
        else:
            raise ValueError(
                f"follow_source must be 'dynamic_goal' or 'landmarks', got '{self.follow_source}'"
            )

        # Reset service to unlock Collision state explicitly
        self.create_service(Trigger, '/reset_nav', self.reset_nav_callback)

        # Control loop timer (15 Hz)
        self.create_timer(dt, self.control_step)
        self.get_logger().info(
            f'DWA node started at {control_rate:.1f} Hz '
            f'(mode={self.mode}, follow_source={self.follow_source}, '
            f'nan_policy={scan_cfg.nan_policy}, v_max={dwa_cfg.max_lin_vel:.2f} m/s)'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _msg_stamp_or_now(self, stamp) -> float:
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return t if t > 0.0 else self._now_sec()

    def odom_callback(self, msg: Odometry) -> None:
        """Record latest odometry pose and velocity using message header timestamp."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(ori.x, ori.y, ori.z, ori.w)
        twist = msg.twist.twist
        self.manager.update_odom(
            pose=(pos.x, pos.y, yaw),
            vel=(twist.linear.x, twist.angular.z),
            stamp_sec=self._msg_stamp_or_now(msg.header.stamp),
        )

    def scan_callback(self, msg: LaserScan) -> None:
        """Store the latest LiDAR scan with its capture header timestamp."""
        scan = ScanData(
            ranges=msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
        )
        self.manager.update_scan(
            scan,
            stamp_sec=self._msg_stamp_or_now(msg.header.stamp),
            now_sec=self._now_sec(),
        )

    def goal_callback(self, msg: PoseStamped) -> None:
        """Accept or reject a new static goal pose."""
        accepted, info = self.manager.set_goal(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.header.frame_id,
        )
        if accepted:
            self.get_logger().info(info)
        else:
            self.get_logger().warn(info)

    def dynamic_goal_callback(self, msg: Odometry) -> None:
        """Update the moving target from /dynamic_goal_pose."""
        pos = msg.pose.pose.position
        frame_id = msg.header.frame_id if msg.header.frame_id else self.manager.nav_cfg.world_frame
        accepted, info = self.manager.update_dynamic_goal(
            pos.x, pos.y, frame_id, stamp_sec=self._now_sec()
        )
        if not accepted:
            self.get_logger().warn(info)
        elif 'jump' in info:
            self.get_logger().warn(info)

    def landmarks_callback(self, msg) -> None:
        """Project observed AprilTags into odom using the message header timestamp."""
        obs = [
            LandmarkObservation(
                landmark_id=int(lm.id),
                range_m=float(lm.range),
                bearing_rad=float(lm.bearing),
            )
            for lm in msg.landmarks
        ]
        obs_stamp = self._msg_stamp_or_now(msg.header.stamp)
        ok, info = self.manager.update_landmarks(
            obs, stamp_sec=obs_stamp, now_sec=self._now_sec()
        )
        if ok:
            if 'skew' in info.lower():
                self.get_logger().warn(info)
            elif 'locked' in info.lower():
                self.get_logger().info(info)

    def reset_nav_callback(self, _request, response):
        """Handle /reset_nav service to clear collision lock and stop robot."""
        ok, msg = self.manager.reset_navigation()
        self.stop_robot()
        response.success = ok
        response.message = msg
        self.get_logger().info(msg)
        return response

    def publish_cmd(self, v: float, w: float) -> None:
        """Publish a Twist velocity command."""
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

    def stop_robot(self) -> None:
        """Publish zero linear and angular velocity."""
        self.publish_cmd(0.0, 0.0)

    def control_step(self) -> None:
        """Run one 15 Hz DWA control tick."""
        out = self.manager.step(self._now_sec())

        if out.info:
            self.get_logger().info(out.info)

        if out.warning:
            self.get_logger().warn(out.warning)

        if out.publish_cmd:
            self.publish_cmd(out.cmd_vel[0], out.cmd_vel[1])

        if out.feedback_distance is not None:
            dist_msg = Float32()
            dist_msg.data = float(out.feedback_distance)
            self.dist_pub.publish(dist_msg)
            self.get_logger().info(
                f'[Step {out.step_count}] Distance to goal: {out.feedback_distance:.3f} m'
            )

        if out.event is not None:
            res_msg = String()
            res_msg.data = out.event
            self.result_pub.publish(res_msg)
            if out.event == 'Lost':
                self.get_logger().warn(
                    f'Follow target Lost reported (steps={out.step_count}); continuing pursuit'
                )
            else:
                self.get_logger().info(
                    f'Navigation finished with result: {out.event} (steps={out.step_count})'
                )


def main(args=None) -> None:
    """Initialize rclpy, spin DwaNode, and send zero velocity on shutdown."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DwaNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop_robot()
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
