"""
Run the Task 1 and Task 2 filters on the same recorded data, using the message stamps.

This compares the two filters on one trajectory, which two live runs cannot do.
Usage:  python3 replay_filters.py <bag_folder> <sim.yaml> [init_x init_y init_theta]
"""
import math
import os
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from lab04_pkg import probabilistic_models as pm  # noqa: E402
from lab04_pkg.ekf import RobotEKF  # noqa: E402

LANDMARKS = {11: (-1.1, -1.1), 12: (-1.1, 0.0), 13: (-1.1, 1.1), 21: (0.0, -1.1), 22: (0.0, 0.0),
             23: (0.0, 1.1), 31: (1.1, -1.1), 32: (1.1, 0.0), 33: (1.1, 1.1)}


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    events, gt = [], []
    while reader.has_next():
        topic, raw, t_rec = reader.read_next()
        if topic not in ('/odom', '/imu', '/landmarks', '/ground_truth'):
            continue
        m = deserialize_message(raw, get_message(types[topic]))
        t = stamp(m)
        if topic == '/ground_truth':
            q = m.pose.pose.orientation
            gt.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                       math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))))
        elif topic == '/odom':
            events.append((t, 0, (m.twist.twist.linear.x, m.twist.twist.angular.z)))
        elif topic == '/imu':
            events.append((t, 1, m.angular_velocity.z))
        else:
            events.append((t, 2, [(x.id, x.range, x.bearing) for x in m.landmarks]))
    events.sort(key=lambda e: (e[0], e[1]))
    return events, np.array(sorted(gt))


def run(events, gt, cfg, fusion, init, rate=20.0):
    M = np.diag([cfg['std_v']**2, cfg['std_w']**2])
    Q_l = np.diag([cfg['std_range']**2, math.radians(cfg['std_bearing_deg'])**2])
    Q_o = np.diag([cfg['std_odom_v']**2, cfg['std_odom_w']**2])
    Q_i = np.array([[cfg['std_imu_w']**2]])
    var = [s**2 for s in cfg['init_std']]
    ekf = RobotEKF(list(init) + [0.0, 0.0], np.diag(var + [0.01, 0.01])) if fusion else \
        RobotEKF(list(init), np.diag(var))
    odom, imu, new_o, new_i = None, None, False, False
    next_tick, last = None, None
    out = []
    for t, kind, data in events:
        while next_tick is not None and next_tick <= t:
            dt = next_tick - last if last is not None else None
            last = next_tick
            next_tick += 1.0 / rate
            if dt is None or odom is None:
                continue
            mu = ekf.mu
            if fusion:
                ekf.predict(pm.extended_g(mu, dt), pm.extended_G(mu, dt), pm.extended_V(mu, dt), M)
                if new_o:
                    ekf.update(odom, pm.odom_h(ekf.mu), pm.odom_H(), Q_o)
                    new_o = False
                if new_i:
                    ekf.update([imu], pm.imu_h(ekf.mu), pm.imu_H(), Q_i)
                    new_i = False
            else:
                ekf.predict(pm.motion_g(mu, odom, dt), pm.motion_G(mu, odom, dt),
                            pm.motion_V(mu, odom, dt), M)
            out.append((last, *ekf.mu[:3]))
        if next_tick is None:
            next_tick = t
        if kind == 0:
            odom, new_o = data, True
        elif kind == 1:
            imu, new_i = data, True
        else:
            for i, r, b in data:
                pos = LANDMARKS[i]
                ekf.update([r, b], pm.landmark_h(ekf.mu, pos),
                           pm.landmark_H(ekf.mu, pos, dim=len(ekf.mu)), Q_l, angle_idx=1)
    est = np.array(out)
    idx = np.searchsorted(gt[:, 0], est[:, 0]).clip(0, len(gt) - 1)
    e = est[:, 1:4] - gt[idx, 1:4]
    e[:, 2] = np.arctan2(np.sin(e[:, 2]), np.cos(e[:, 2]))
    pos = np.hypot(e[:, 0], e[:, 1])
    settled = np.argmax(pos[10:] < 0.05) / rate if (pos[10:] < 0.05).any() else float('nan')
    return {'RMSE pos': np.sqrt(np.mean(pos**2)), 'MAE pos': pos.mean(),
            'RMSE th': np.sqrt(np.mean(e[:, 2]**2)), 'MAE th': np.abs(e[:, 2]).mean(),
            'max pos': pos.max(), 't<5cm': settled, 'n': len(est)}


def main():
    bag, cfg_path = sys.argv[1:3]
    cfg = yaml.safe_load(open(cfg_path))['ekf_node']['ros__parameters']
    init = [float(v) for v in sys.argv[3:6]] or cfg['init_pose']
    events, gt = read(bag)
    print('init', init)
    for name, fusion in (('Task 1', False), ('Task 2', True)):
        r = run(events, gt, cfg, fusion, init)
        print(name, ' '.join(f'{k} {v:.4f}' for k, v in r.items()))


if __name__ == '__main__':
    main()
