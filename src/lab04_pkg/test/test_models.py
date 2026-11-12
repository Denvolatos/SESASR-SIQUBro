import math

from lab04_pkg import probabilistic_models as pm
from lab04_pkg.ekf import RobotEKF
import numpy as np
import pytest

CASES = [((1.0, -2.0, 0.7), (0.2, 0.5)), ((0.0, 0.0, -2.5), (0.15, -1.2)),
         ((3.0, 1.0, 3.1), (0.22, 0.0))]
DT = 0.05


def numeric_jacobian(f, x, eps=1e-6):
    x = np.array(x, dtype=float)
    columns = []
    for i in range(len(x)):
        d = np.zeros_like(x)
        d[i] = eps
        columns.append((np.array(f(x + d)) - np.array(f(x - d))) / (2 * eps))
    return np.array(columns).T


@pytest.mark.parametrize('mu, u', CASES)
def test_motion_jacobians(mu, u):
    G = numeric_jacobian(lambda s: pm.motion_g(s, u, DT), mu)
    V = numeric_jacobian(lambda c: pm.motion_g(mu, c, DT), u, eps=1e-3 if u[1] == 0 else 1e-6)
    assert pm.motion_G(mu, u, DT) == pytest.approx(G, abs=1e-6)
    assert pm.motion_V(mu, u, DT) == pytest.approx(V, abs=1e-5)


def test_straight_line_matches_small_turn():
    mu, v = (1.0, 2.0, 0.4), 0.2
    assert pm.motion_g(mu, (v, 0.0), DT) == pytest.approx(pm.motion_g(mu, (v, 1e-5), DT), abs=1e-6)
    assert pm.motion_V(mu, (v, 0.0), DT) == pytest.approx(pm.motion_V(mu, (v, 1e-5), DT), abs=1e-6)


def test_quarter_circle():
    pose = pm.motion_g((0.0, 0.0, 0.0), (1.0, 1.0), math.pi / 2)
    assert pose == pytest.approx([1.0, 1.0, math.pi / 2])


def test_straight_line_north():
    assert pm.motion_g((0.0, 0.0, math.pi / 2), (0.2, 0.0), 2.0) == pytest.approx(
        [0.0, 0.4, math.pi / 2])


def test_landmark_model_and_jacobian():
    mu, lm = (1.0, 2.0, 0.3), (2.0, 4.0)
    z = pm.landmark_h(mu, lm)
    assert z[0] == pytest.approx(math.hypot(1.0, 2.0))
    assert z[1] == pytest.approx(math.atan2(2.0, 1.0) - 0.3)
    H = numeric_jacobian(lambda s: pm.landmark_h(s, lm), mu)
    assert pm.landmark_H(mu, lm) == pytest.approx(H, abs=1e-6)
    H5 = pm.landmark_H((*mu, 0.1, 0.2), lm, dim=5)
    assert H5.shape == (2, 5)
    assert H5[:, 3:] == pytest.approx(np.zeros((2, 2)))


@pytest.mark.parametrize('w', [0.5, -1.0, 0.0])
def test_extended_jacobians(w):
    mu = (1.0, -1.0, 0.8, 0.2, w)
    eps = 1e-3 if w == 0.0 else 1e-6
    G = numeric_jacobian(lambda s: pm.extended_g(s, DT), mu, eps=eps)
    assert pm.extended_G(mu, DT) == pytest.approx(G, abs=1e-5)
    assert pm.odom_H() @ np.array(mu) == pytest.approx(pm.odom_h(mu))
    assert pm.imu_H() @ np.array(mu) == pytest.approx(pm.imu_h(mu))


def test_pose_samples_reproduce_the_measurement():
    rng = np.random.default_rng(1)
    lm, z = (2.0, 1.0), np.array([1.5, 0.4])
    for _ in range(20):
        pose = pm.sample_pose_from_landmark(z, lm, (0.0, 0.0), rng)
        assert pm.landmark_h(pose, lm) == pytest.approx(z, abs=1e-9)


def test_bearing_residual_wraps():
    ekf = RobotEKF([0.0, 0.0, 0.0], np.eye(3))
    z_hat = np.array([1.0, math.pi - 0.01])
    H = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    ekf.update([1.0, -math.pi + 0.01], z_hat, H, np.eye(2), angle_idx=1)
    assert abs(ekf.mu[2]) < 0.1      # a small correction, not a turn of almost 2 pi


def test_filter_converges_to_the_true_pose():
    rng = np.random.default_rng(3)
    landmarks = [(-1.1, -1.1), (1.1, 0.0), (0.0, 1.1), (1.1, 1.1)]
    truth = np.array([-0.5, -0.5, 0.2])
    ekf = RobotEKF([0.0, 0.0, 0.0], np.diag([0.25, 0.25, 0.25]))
    Q = np.diag([0.05**2, math.radians(2.0)**2])
    M = np.diag([0.05**2, 0.1**2])
    u = (0.1, 0.2)
    start = np.linalg.norm(ekf.mu[:2] - truth[:2])
    for _ in range(400):
        truth = pm.motion_g(truth, u, DT)
        ekf.predict(pm.motion_g(ekf.mu, u, DT), pm.motion_G(ekf.mu, u, DT),
                    pm.motion_V(ekf.mu, u, DT), M)
        for lm in landmarks:
            z = pm.sample_landmark_model(truth, lm, (0.05, math.radians(2.0)), rng)
            ekf.update(z, pm.landmark_h(ekf.mu, lm), pm.landmark_H(ekf.mu, lm), Q, angle_idx=1)
    assert np.linalg.norm(ekf.mu[:2] - truth[:2]) < 0.05 < start
    assert abs(pm.wrap_angle(ekf.mu[2] - truth[2])) < 0.05
    assert ekf.Sigma == pytest.approx(ekf.Sigma.T)
