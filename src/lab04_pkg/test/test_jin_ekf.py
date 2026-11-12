"""Behavioral checks for Jin's independent Task 1 filter."""

import math

from lab04_pkg.jin_ekf import (
    JinEKF,
    landmark_prediction,
    velocity_step,
    wrap_angle,
)
import numpy as np
import pytest


def numerical_jacobian(function, point, step=1e-6):
    """Use centered differences to check the derived linearization."""
    point = np.asarray(point, dtype=float)
    columns = []
    for index in range(point.size):
        offset = np.zeros_like(point)
        offset[index] = step
        columns.append(
            (function(point + offset) - function(point - offset)) / (2 * step)
        )
    return np.column_stack(columns)


@pytest.mark.parametrize("angular_velocity", [0.0, 1e-7, 0.6, -0.8])
def test_motion_jacobians_across_straight_and_curved_motion(angular_velocity):
    pose, control, dt = (
        np.array([0.3, -0.2, 0.8]),
        np.array([0.2, angular_velocity]),
        0.05,
    )
    next_pose, G, V = velocity_step(pose, control, dt)
    numeric_G = numerical_jacobian(
        lambda state: velocity_step(state, control, dt)[0], pose
    )
    numeric_V = numerical_jacobian(
        lambda u: velocity_step(pose, u, dt)[0], control
    )
    assert next_pose.shape == (3,)
    assert G == pytest.approx(numeric_G, abs=1e-8)
    assert V == pytest.approx(numeric_V, abs=1e-8)


def test_landmark_jacobian_and_bearing_wrap():
    pose, landmark = np.array([-0.5, 0.2, 0.4]), (1.1, -1.1)
    expected, H = landmark_prediction(pose, landmark)
    numeric_H = numerical_jacobian(
        lambda state: landmark_prediction(state, landmark)[0], pose
    )
    assert H == pytest.approx(numeric_H, abs=1e-8)
    assert -math.pi <= expected[1] <= math.pi
    assert wrap_angle(math.pi + 0.01) == pytest.approx(-math.pi + 0.01)


def test_filter_converges_and_keeps_covariance_valid():
    rng = np.random.default_rng(370672)
    filter_ = JinEKF(
        [0.0, 0.0, 0.0], [0.5, 0.5, 0.5], [0.05, 0.1], [0.05, math.radians(2)]
    )
    truth = np.array([-0.4, 0.3, 0.2])
    landmarks = [(-1.1, -1.1), (-1.1, 1.1), (1.1, -1.1), (1.1, 1.1)]
    for _ in range(150):
        control = (0.12, 0.2)
        truth = velocity_step(truth, control, 0.05)[0]
        assert filter_.predict(control, 0.05)
        for landmark in landmarks:
            z = landmark_prediction(truth, landmark)[0]
            z += rng.normal(0, [0.05, math.radians(2)])
            assert filter_.update_landmark(z, landmark)
    assert np.linalg.norm(filter_.state[:2] - truth[:2]) < 0.05
    assert abs(wrap_angle(filter_.state[2] - truth[2])) < 0.05
    assert filter_.covariance == pytest.approx(filter_.covariance.T, abs=1e-12)
    assert np.linalg.eigvalsh(filter_.covariance).min() >= 0


def test_invalid_and_gated_observations_leave_estimate_unchanged():
    filter_ = JinEKF(
        [0.0, 0.0, 0.0],
        [0.1] * 3,
        [0.05, 0.1],
        [0.05, 0.03],
        innovation_gate=9.21,
    )
    pose_before = filter_.state.copy()
    covariance_before = filter_.covariance.copy()
    assert not filter_.update_landmark([float("nan"), 0.0], (1.0, 0.0))
    assert not filter_.update_landmark([0.0, 0.0], (1.0, 0.0))
    assert not filter_.update_landmark([5.0, 2.0], (1.0, 0.0))
    assert filter_.state == pytest.approx(pose_before)
    assert filter_.covariance == pytest.approx(covariance_before)
    assert filter_.update_landmark([1.02, 0.01], (1.0, 0.0))
