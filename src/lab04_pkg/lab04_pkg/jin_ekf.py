"""Independent three-state EKF for Guanchen Jin's localization experiment.

The filter has no ROS dependency. State is [x, y, heading], control is
[linear velocity, angular velocity], and landmark observations are planar
[range, bearing]. Velocities are measured from /odom by the ROS wrapper.
"""

import math

import numpy as np


def wrap_angle(angle):
    """Return an angle in [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _sinc_and_derivative(value):
    """Compute sinc and its derivative without a zero singularity."""
    if abs(value) < 1e-4:
        square = value * value
        return (
            1.0 - square / 6.0 + square * square / 120.0,
            -value / 3.0 + value * square / 30.0,
        )
    return math.sin(value) / value, (
        value * math.cos(value) - math.sin(value)
    ) / value**2


def velocity_step(state, velocity, dt):
    """Return the next pose and Jacobians with respect to state and velocity.

    The midpoint/sinc form is the exact constant-velocity unicycle motion
    model and remains well behaved when angular velocity approaches zero.
    """
    x, y, heading = state
    v, w = velocity
    half_turn = w * dt / 2.0
    sinc, sinc_prime = _sinc_and_derivative(half_turn)
    mid_heading = heading + half_turn
    c, s = math.cos(mid_heading), math.sin(mid_heading)
    distance = v * dt * sinc

    next_state = np.array(
        [x + distance * c, y + distance * s, wrap_angle(heading + w * dt)],
        dtype=float,
    )

    G = np.eye(3)
    G[0, 2], G[1, 2] = -distance * s, distance * c

    V = np.zeros((3, 2))
    V[0, 0], V[1, 0] = dt * sinc * c, dt * sinc * s
    V[0, 1] = v * dt**2 / 2.0 * (sinc_prime * c - sinc * s)
    V[1, 1] = v * dt**2 / 2.0 * (sinc_prime * s + sinc * c)
    V[2, 1] = dt
    return next_state, G, V


def landmark_prediction(state, landmark):
    """Return expected [range, bearing] and its state Jacobian."""
    dx, dy = landmark[0] - state[0], landmark[1] - state[1]
    square_range = dx * dx + dy * dy
    if square_range < 1e-12:
        raise ValueError("landmark is too close for a bearing Jacobian")
    distance = math.sqrt(square_range)
    expected = np.array([distance, wrap_angle(math.atan2(dy, dx) - state[2])])
    H = np.array(
        [
            [-dx / distance, -dy / distance, 0.0],
            [dy / square_range, -dx / square_range, -1.0],
        ]
    )
    return expected, H


class JinEKF:
    """EKF with an optional innovation gate."""

    def __init__(
        self,
        initial_pose,
        initial_std,
        velocity_std,
        landmark_std,
        innovation_gate=0.0,
    ):
        self.state = np.asarray(initial_pose, dtype=float).copy()
        if self.state.shape != (3,):
            raise ValueError("initial_pose must contain x, y, heading")
        self.state[2] = wrap_angle(self.state[2])
        self.covariance = np.diag(
            np.square(np.asarray(initial_std, dtype=float))
        )
        self.velocity_covariance = np.diag(
            np.square(np.asarray(velocity_std, dtype=float))
        )
        self.landmark_covariance = np.diag(
            np.square(np.asarray(landmark_std, dtype=float))
        )
        if (
            self.covariance.shape != (3, 3)
            or self.velocity_covariance.shape != (2, 2)
            or self.landmark_covariance.shape != (2, 2)
        ):
            raise ValueError("standard deviation arrays have the wrong size")
        if (
            not np.isfinite(self.state).all()
            or not np.isfinite(self.covariance).all()
            or not np.isfinite(self.velocity_covariance).all()
            or not np.isfinite(self.landmark_covariance).all()
        ):
            raise ValueError("initial state and covariance must be finite")
        if (
            min(np.diag(self.velocity_covariance)) <= 0
            or min(np.diag(self.landmark_covariance)) <= 0
        ):
            raise ValueError("velocity and landmark noise must be positive")
        self.innovation_gate = float(innovation_gate)
        if not math.isfinite(self.innovation_gate) or self.innovation_gate < 0:
            raise ValueError("innovation_gate must be nonnegative")

    def predict(self, velocity, dt):
        """Propagate using the latest measured wheel velocities."""
        if dt <= 0 or not np.isfinite(dt) or not np.isfinite(velocity).all():
            return False
        next_state, G, V = velocity_step(self.state, velocity, dt)
        self.state = next_state
        self.covariance = (
            G @ self.covariance @ G.T + V @ self.velocity_covariance @ V.T
        )
        self.covariance = (self.covariance + self.covariance.T) / 2.0
        return True

    def update_landmark(self, observation, landmark):
        """Correct with one landmark; reject invalid or gated data."""
        z = np.asarray(observation, dtype=float)
        if z.shape != (2,) or not np.isfinite(z).all() or z[0] <= 0:
            return False
        try:
            expected, H = landmark_prediction(self.state, landmark)
        except ValueError:
            return False
        residual = z - expected
        residual[1] = wrap_angle(residual[1])
        S = H @ self.covariance @ H.T + self.landmark_covariance
        if self.innovation_gate:
            squared_distance = residual @ np.linalg.solve(S, residual)
            if squared_distance > self.innovation_gate:
                return False
        K = np.linalg.solve(S, H @ self.covariance).T
        self.state += K @ residual
        self.state[2] = wrap_angle(self.state[2])
        I_KH = np.eye(3) - K @ H
        self.covariance = (
            I_KH @ self.covariance @ I_KH.T
            + K @ self.landmark_covariance @ K.T
        )
        self.covariance = (self.covariance + self.covariance.T) / 2.0
        return True
