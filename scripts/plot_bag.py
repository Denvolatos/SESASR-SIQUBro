"""Plot a Lab02/Lab03 rosbag: trajectory, velocity commands, odometry error.

Usage:  python3 plot_bag.py <bag_folder> [output_folder]
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')                       # save files, no window needed
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ---------- 1. Read the bag ----------
def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    data = {name: [] for name in types}
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        msg = deserialize_message(raw, get_message(types[topic]))
        data[topic].append((stamp_ns * 1e-9, msg))   # time in seconds
    return data


def pose_series(samples):
    """Odometry messages -> arrays t, x, y, yaw."""
    t = np.array([s[0] for s in samples])
    x = np.array([s[1].pose.pose.position.x for s in samples])
    y = np.array([s[1].pose.pose.position.y for s in samples])
    yaw = np.array([2 * math.atan2(s[1].pose.pose.orientation.z, s[1].pose.pose.orientation.w)
                    for s in samples])
    return t, x, y, yaw


def main():
    bag = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(os.path.abspath(bag)), 'plots')
    os.makedirs(out, exist_ok=True)
    name = os.path.basename(os.path.normpath(bag))
    data = read_bag(bag)

    t_odom, x_odom, y_odom, _ = pose_series(data['/odom'])
    t0 = t_odom[0]                                   # time zero = first odometry message
    has_gt = len(data.get('/ground_truth', [])) > 0
    if has_gt:
        t_gt, x_gt, y_gt, _ = pose_series(data['/ground_truth'])

    # ---------- 2. Trajectory on the XY plane ----------
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(x_odom, y_odom, label='odometry (/odom)', color='tab:blue')
    if has_gt:
        ax.plot(x_gt, y_gt, '--', label='ground truth (/ground_truth)', color='tab:orange')
    ax.plot(x_odom[0], y_odom[0], 'go', label='start')
    ax.plot(x_odom[-1], y_odom[-1], 'rs', label='end')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(f'Robot trajectory ({name})')
    ax.axis('equal')
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out, f'{name}_trajectory.pdf'))
    fig.savefig(os.path.join(out, f'{name}_trajectory.png'), dpi=150)

    # ---------- 3. Velocity commands over time ----------
    cmd = data['/cmd_vel']
    t_cmd = np.array([c[0] for c in cmd]) - t0
    v = np.array([c[1].linear.x for c in cmd])
    w = np.array([c[1].angular.z for c in cmd])
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    a1.plot(t_cmd, v, color='tab:blue', label='linear velocity v')
    a1.set_ylabel('v [m/s]')
    a1.grid(True)
    a1.legend(loc='upper right')
    a2.plot(t_cmd, w, color='tab:red', label='angular velocity ω')
    a2.set_ylabel('ω [rad/s]')
    a2.set_xlabel('time [s]')
    a2.grid(True)
    a2.legend(loc='upper right')
    a1.set_title(f'Velocity commands on /cmd_vel ({name})')
    fig.tight_layout()
    fig.savefig(os.path.join(out, f'{name}_cmd_vel.pdf'))
    fig.savefig(os.path.join(out, f'{name}_cmd_vel.png'), dpi=150)

    # ---------- 4. Odometry error (simulation only) ----------
    if has_gt:
        gx = np.interp(t_odom, t_gt, x_gt)           # ground truth at the odometry time stamps
        gy = np.interp(t_odom, t_gt, y_gt)
        err = np.hypot(x_odom - gx, y_odom - gy)
        fig, ax = plt.subplots(figsize=(8, 3.5))
        ax.plot(t_odom - t0, err, color='tab:purple', label='|odom − ground truth|')
        ax.set_xlabel('time [s]')
        ax.set_ylabel('position error [m]')
        ax.set_title(f'Odometry position error ({name})')
        ax.grid(True)
        ax.legend(loc='upper left')
        fig.tight_layout()
        fig.savefig(os.path.join(out, f'{name}_odom_error.pdf'))
        fig.savefig(os.path.join(out, f'{name}_odom_error.png'), dpi=150)
        print(f'final odometry error: {err[-1]:.3f} m, max: {err.max():.3f} m')

    print(f'duration: {t_odom[-1] - t0:.1f} s, plots saved in {out}')


if __name__ == '__main__':
    main()
