"""Velocity motion model and landmark measurement model. Pure Python: no ROS."""

import math

import numpy as np

SMALL_W = 1e-6     # [rad/s] below this, the motion is treated as a straight line


def wrap_angle(a):
    """Angle in [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


# ---------- Velocity motion model, state [x, y, theta], command [v, w] ----------
def motion_g(mu, u, dt):
    """Next pose without noise."""
    x, y, theta = mu[:3]
    v, w = u
    if abs(w) < SMALL_W:
        return np.array([x + v * dt * math.cos(theta),
                         y + v * dt * math.sin(theta),
                         theta + w * dt])
    r = v / w
    beta = theta + w * dt
    return np.array([x - r * math.sin(theta) + r * math.sin(beta),
                     y + r * math.cos(theta) - r * math.cos(beta),
                     beta])


def motion_G(mu, u, dt):
    """Jacobian of motion_g with respect to the state."""
    theta = mu[2]
    v, w = u
    if abs(w) < SMALL_W:
        return np.array([[1.0, 0.0, -v * dt * math.sin(theta)],
                         [0.0, 1.0, v * dt * math.cos(theta)],
                         [0.0, 0.0, 1.0]])
    r = v / w
    beta = theta + w * dt
    return np.array([[1.0, 0.0, -r * math.cos(theta) + r * math.cos(beta)],
                     [0.0, 1.0, -r * math.sin(theta) + r * math.sin(beta)],
                     [0.0, 0.0, 1.0]])


def motion_V(mu, u, dt):
    """Jacobian of motion_g with respect to the command."""
    theta = mu[2]
    v, w = u
    if abs(w) < SMALL_W:
        return np.array([[dt * math.cos(theta), -0.5 * v * dt**2 * math.sin(theta)],
                         [dt * math.sin(theta), 0.5 * v * dt**2 * math.cos(theta)],
                         [0.0, dt]])
    beta = theta + w * dt
    s, c, sb, cb = math.sin(theta), math.cos(theta), math.sin(beta), math.cos(beta)
    return np.array([[(sb - s) / w, v * (s - sb) / w**2 + v * dt * cb / w],
                     [(c - cb) / w, -v * (c - cb) / w**2 + v * dt * sb / w],
                     [0.0, dt]])


def sample_velocity_motion_model(x, u, a, dt, rng=None):
    """
    Sample the next pose.

    x: pose [x, y, theta], u: command [v, w], dt: time step.
    a: noise coefficients [a1..a6]; a1*v^2 + a2*w^2 is the variance of the noise on v,
    a3, a4 the same for w, a5, a6 for the final extra rotation.
    """
    rng = np.random.default_rng() if rng is None else rng
    v, w = u
    v_hat = v + rng.normal(0.0, math.sqrt(a[0] * v**2 + a[1] * w**2))
    w_hat = w + rng.normal(0.0, math.sqrt(a[2] * v**2 + a[3] * w**2))
    gamma = rng.normal(0.0, math.sqrt(a[4] * v**2 + a[5] * w**2))
    pose = motion_g(x, (v_hat, w_hat), dt)
    pose[2] += gamma * dt
    return pose


# ---------- Landmark model, measurement [range, bearing] ----------
def landmark_h(mu, landmark):
    """Return the expected range and bearing of a landmark [m_x, m_y]."""
    dx, dy = landmark[0] - mu[0], landmark[1] - mu[1]
    return np.array([math.hypot(dx, dy), wrap_angle(math.atan2(dy, dx) - mu[2])])


def landmark_H(mu, landmark, dim=3):
    """Jacobian of landmark_h with respect to a state of size dim (3 or 5)."""
    dx, dy = landmark[0] - mu[0], landmark[1] - mu[1]
    q = dx**2 + dy**2
    r = math.sqrt(q)
    H = np.zeros((2, dim))
    H[0, :3] = [-dx / r, -dy / r, 0.0]
    H[1, :3] = [dy / q, -dx / q, -1.0]
    return H


def sample_landmark_model(x, landmark, sigma, rng=None):
    """Sample a measurement [range, bearing] of the landmark from pose x."""
    rng = np.random.default_rng() if rng is None else rng
    r, phi = landmark_h(x, landmark)
    return np.array([r + rng.normal(0.0, sigma[0]), wrap_angle(phi + rng.normal(0.0, sigma[1]))])


def sample_pose_from_landmark(z, landmark, sigma, rng=None):
    """Sample a pose [x, y, theta] compatible with the measurement z = [range, bearing]."""
    rng = np.random.default_rng() if rng is None else rng
    gamma = rng.uniform(0.0, 2.0 * math.pi)
    r = z[0] + rng.normal(0.0, sigma[0])
    phi = z[1] + rng.normal(0.0, sigma[1])
    return np.array([landmark[0] + r * math.cos(gamma),
                     landmark[1] + r * math.sin(gamma),
                     wrap_angle(gamma - math.pi - phi)])


# ---------- Task 2: state [x, y, theta, v, w] ----------
def extended_g(mu, dt):
    """Next state: the pose moves with the velocities in the state, which stay constant."""
    return np.concatenate((motion_g(mu[:3], mu[3:5], dt), mu[3:5]))


def extended_G(mu, dt):
    """Jacobian of extended_g with respect to the state."""
    G = np.eye(5)
    G[:3, :3] = motion_G(mu[:3], mu[3:5], dt)
    G[:3, 3:5] = motion_V(mu[:3], mu[3:5], dt)
    return G


def extended_V(mu, dt):
    """Jacobian of extended_g with respect to noise on the velocities [v, w]."""
    V = np.zeros((5, 2))
    V[:3, :] = motion_V(mu[:3], mu[3:5], dt)
    V[3:5, :] = np.eye(2)
    return V


def odom_h(mu):
    return np.array([mu[3], mu[4]])


def odom_H():
    return np.array([[0.0, 0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 0.0, 1.0]])


def imu_h(mu):
    return np.array([mu[4]])


def imu_H():
    return np.array([[0.0, 0.0, 0.0, 0.0, 1.0]])
