"""
Plots and metrics for an EKF rosbag: /ekf and /odom against /ground_truth (when recorded).

Usage:  python3 evaluate_ekf.py <bag_folder> <output_folder> [label]
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

LANDMARKS = [(-1.1, -1.1), (-1.1, 0.0), (-1.1, 1.1), (0.0, -1.1), (0.0, 0.0), (0.0, 1.1),
             (1.1, -1.1), (1.1, 0.0), (1.1, 1.1)]
COLORS = {'/ground_truth': 'tab:green', '/odom': 'tab:blue', '/ekf': 'tab:red'}


def read_poses(path):
    """Topic -> array [t, x, y, theta], with t taken from header.stamp."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    data = {name: [] for name in COLORS if name in types}
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic in data:
            m = deserialize_message(raw, get_message(types[topic]))
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            data[topic].append((t, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
    return {k: np.array(sorted(v)) for k, v in data.items() if v}


def align_start(poses, ref):
    """Rigidly move an odometry track so that it starts at the first pose of ref."""
    out = poses.copy()
    d = ref[0, 3] - poses[0, 3]
    c, s = math.cos(d), math.sin(d)
    dx, dy = poses[:, 1] - poses[0, 1], poses[:, 2] - poses[0, 2]
    out[:, 1] = ref[0, 1] + c * dx - s * dy
    out[:, 2] = ref[0, 2] + s * dx + c * dy
    out[:, 3] = np.arctan2(np.sin(poses[:, 3] + d), np.cos(poses[:, 3] + d))
    return out


def errors(est, gt):
    """Errors x, y, theta at the est times, against the nearest ground-truth sample."""
    idx = np.searchsorted(gt[:, 0], est[:, 0]).clip(1, len(gt) - 1)
    idx -= (est[:, 0] - gt[idx - 1, 0]) < (gt[idx, 0] - est[:, 0])
    e = est[:, 1:4] - gt[idx, 1:4]
    e[:, 2] = np.arctan2(np.sin(e[:, 2]), np.cos(e[:, 2]))
    return e


def metrics(e):
    pos = np.hypot(e[:, 0], e[:, 1])
    return {'RMSE x': np.sqrt(np.mean(e[:, 0]**2)), 'RMSE y': np.sqrt(np.mean(e[:, 1]**2)),
            'RMSE pos': np.sqrt(np.mean(pos**2)), 'RMSE theta': np.sqrt(np.mean(e[:, 2]**2)),
            'MAE x': np.mean(np.abs(e[:, 0])), 'MAE y': np.mean(np.abs(e[:, 1])),
            'MAE pos': np.mean(pos), 'MAE theta': np.mean(np.abs(e[:, 2])),
            'final pos': pos[-1], 'max pos': pos.max()}


def main():
    bag, out = sys.argv[1], sys.argv[2]
    label = sys.argv[3] if len(sys.argv) > 3 else os.path.basename(bag.rstrip('/'))
    os.makedirs(out, exist_ok=True)
    data = read_poses(bag)
    print({k: (len(v), np.round(v[0, 1:], 3).tolist()) for k, v in data.items()})
    ref = data.get('/ground_truth', data['/ekf'])
    data['/odom'] = align_start(data['/odom'], ref)      # /odom has its own origin
    t0 = min(v[0, 0] for v in data.values())

    fig, axes = plt.subplots(3, 1, figsize=(7, 6), sharex=True)
    for ax, i, name in zip(axes, (1, 2, 3), ('x [m]', 'y [m]', 'θ [rad]')):
        for topic, v in data.items():
            ax.plot(v[:, 0] - t0, v[:, i], color=COLORS[topic], lw=1, label=topic)
        ax.set_ylabel(name)
        ax.grid(True)
    axes[0].legend(ncol=3, fontsize=8, loc='upper right')
    axes[-1].set_xlabel('time [s]')
    fig.suptitle(f'Pose over time ({label})', fontsize=10)
    fig.tight_layout()
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(out, f'{label}_states.{ext}'), dpi=200)

    fig, ax = plt.subplots(figsize=(5.2, 5))
    for topic, v in data.items():
        ax.plot(v[:, 1], v[:, 2], color=COLORS[topic], lw=1, label=topic)
    ax.plot(ref[0, 1], ref[0, 2], 'ko', ms=5, label='start')
    if '/ground_truth' in data:
        ax.plot(*zip(*LANDMARKS), 'ks', ms=5, mfc='none', label='landmarks')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.axis('equal')
    ax.grid(True)
    ax.legend(fontsize=8)
    ax.set_title(f'Trajectory ({label})', fontsize=10)
    fig.tight_layout()
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(out, f'{label}_trajectory.{ext}'), dpi=200)

    duration = data['/ekf'][-1, 0] - data['/ekf'][0, 0]
    print(f'{label}: duration {duration:.1f} s')
    if '/ground_truth' in data:
        lines = []
        for topic in ('/odom', '/ekf'):
            m = metrics(errors(data[topic], data['/ground_truth']))
            lines.append(f'{topic}: ' + ', '.join(f'{k} {v:.4f}' for k, v in m.items()))
            print(lines[-1])
        with open(os.path.join(out, f'{label}_metrics.txt'), 'w') as f:
            f.write(f'duration {duration:.1f} s\n' + '\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
