"""Extended Kalman filter with the prediction and update equations. Pure Python: no ROS."""

import numpy as np


class RobotEKF:
    THETA = 2     # index of the heading in the state

    def __init__(self, mu, Sigma):
        self.mu = np.array(mu, dtype=float)
        self.Sigma = np.array(Sigma, dtype=float)

    def predict(self, mu_new, G, V, M):
        """Prediction: mu_new = g(mu, u), Sigma = G Sigma G^T + V M V^T."""
        self.mu = np.array(mu_new, dtype=float)
        self.mu[self.THETA] = np.arctan2(np.sin(self.mu[self.THETA]), np.cos(self.mu[self.THETA]))
        self.Sigma = G @ self.Sigma @ G.T + V @ M @ V.T

    def update(self, z, z_hat, H, Q, angle_idx=None):
        """Update with measurement z; angle_idx is the angle in z, if there is one."""
        y = np.asarray(z, dtype=float) - z_hat
        if angle_idx is not None:
            y[angle_idx] = np.arctan2(np.sin(y[angle_idx]), np.cos(y[angle_idx]))
        S = H @ self.Sigma @ H.T + Q
        K = self.Sigma @ H.T @ np.linalg.inv(S)
        self.mu = self.mu + K @ y
        self.mu[self.THETA] = np.arctan2(np.sin(self.mu[self.THETA]), np.cos(self.mu[self.THETA]))
        I_KH = np.eye(len(self.mu)) - K @ H
        self.Sigma = I_KH @ self.Sigma @ I_KH.T + K @ Q @ K.T   # Joseph form, stays symmetric
