"""
Bump&Go state machine. Pure Python: no ROS, so it can be unit-tested directly.

GO      front > threshold        -> drive straight
        front <= threshold       -> choose the more open side, go to ROTATE
ROTATE  heading reached (or front clear with turn_until_clear) -> back to GO
DONE    goal reached (only with stop_at_goal)                  -> stand still
"""

import math
from dataclasses import dataclass


def wrap_angle(a):
    """Angle in [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class ControllerConfig:
    max_linear_vel: float = 0.22       # [m/s]
    max_angular_vel: float = 1.5       # [rad/s]
    obstacle_threshold: float = 0.5    # [m]
    stop_at_goal: bool = True
    goal_x: float = 6.5                # [m] green cube in the simulated maze
    goal_y: float = 3.0
    goal_tolerance: float = 0.8        # [m]
    rotation_tolerance_deg: float = 5.0
    min_angular_vel: float = 0.3       # [rad/s] motor dead zone of the real robot
    turn_angle_deg: float = 90.0       # largest turn after a bump
    snap_to_axes: bool = True          # round the turn to the maze axes (simulation)
    turn_until_clear: bool = False     # stop turning once the front is free (real robot)
    clear_margin: float = 0.3          # [m] front is free above threshold + margin


class BumpAndGoController:
    def __init__(self, config):
        self.cfg = config
        self.state = 'GO'
        self.target_yaw = 0.0
        self.turn_start_yaw = 0.0
        self.event = None      # one-line description of the last decision, for the log

    def step(self, x, y, yaw, front, left, right):
        """
        One control step. front/left/right in metres (None = no data).

        Returns (linear velocity, angular velocity).
        """
        c = self.cfg
        self.event = None
        if front is None:
            return 0.0, 0.0

        if (c.stop_at_goal and self.state != 'DONE'
                and math.hypot(x - c.goal_x, y - c.goal_y) < c.goal_tolerance):
            self.state = 'DONE'
            self.event = f'goal reached at x={x:.2f} y={y:.2f}'

        if self.state == 'GO':
            if front > c.obstacle_threshold:
                return c.max_linear_vel, 0.0                       # Go
            if left is None or right is None:                     # cannot choose a side
                return 0.0, 0.0
            turn = math.radians(c.turn_angle_deg)                 # Bump
            turn = turn if left > right else -turn
            self.target_yaw = yaw + turn
            if c.snap_to_axes:
                self.target_yaw = round(self.target_yaw / (math.pi / 2)) * (math.pi / 2)
            self.turn_start_yaw = yaw
            self.state = 'ROTATE'
            self.event = (f'wall at {front:.2f} m (left {left:.2f}, right {right:.2f}) -> '
                          f'turn {"left" if turn > 0 else "right"}')
            return 0.0, 0.0

        if self.state == 'ROTATE':
            error = wrap_angle(self.target_yaw - yaw)
            turned = abs(wrap_angle(yaw - self.turn_start_yaw))
            front_clear = (c.turn_until_clear and turned > math.radians(20.0)
                           and front > c.obstacle_threshold + c.clear_margin)
            if abs(error) < math.radians(c.rotation_tolerance_deg) or front_clear:
                self.state = 'GO'
                return 0.0, 0.0
            w = max(-c.max_angular_vel, min(c.max_angular_vel, 2.0 * error))   # P controller
            if abs(w) < c.min_angular_vel:                                     # dead zone
                w = math.copysign(c.min_angular_vel, w)
            return 0.0, w

        return 0.0, 0.0                                           # DONE
