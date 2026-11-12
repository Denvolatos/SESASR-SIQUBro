"""
Task 0: sample the velocity motion model and the landmark model, and derive the Jacobians.

Usage (from the package folder, no ROS needed):  python3 scripts/task0_models.py <output_folder>
"""
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import sympy as sp  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from lab04_pkg import probabilistic_models as pm  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else '.'
rng = np.random.default_rng(42)


def save(fig, name):
    fig.tight_layout()
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(OUT, f'{name}.{ext}'), dpi=200)
    plt.close(fig)


def draw_pose(ax, pose, label, color='tab:red'):
    ax.quiver(pose[0], pose[1], math.cos(pose[2]), math.sin(pose[2]), color=color, scale=12,
              width=0.008, zorder=5, label=label)


def motion_samples():
    x0, u, dt, n = np.array([0.0, 0.0, 0.0]), (0.5, 0.8), 1.0, 500
    cases = {'Larger noise on the angular motion': [0.001, 0.001, 0.3, 0.3, 0.02, 0.02],
             'Larger noise on the linear motion': [0.3, 0.3, 0.001, 0.001, 0.001, 0.001]}
    fig, axes = plt.subplots(1, 2, figsize=(9, 4))
    for ax, (title, a) in zip(axes, cases.items()):
        s = np.array([pm.sample_velocity_motion_model(x0, u, a, dt, rng) for _ in range(n)])
        ax.scatter(s[:, 0], s[:, 1], s=6, alpha=0.5, label=f'{n} sampled poses')
        draw_pose(ax, x0, 'current pose')
        ax.plot(*pm.motion_g(x0, u, dt)[:2], 'k+', ms=11, mew=2, label='next pose, no noise')
        ax.set_title(f'{title}\na = {a}', fontsize=9)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.axis('equal')
        ax.grid(True)
        ax.legend(fontsize=8, loc='upper left')
        print(f'{title}: std x {s[:, 0].std():.3f} m, y {s[:, 1].std():.3f} m, '
              f'theta {s[:, 2].std():.3f} rad')
    save(fig, 'task0_motion_model')


def landmark_samples():
    """Forward model p(z|x,m) and inverse model p(x|z,m) of a landmark, with two noise levels."""
    x0, lm, n = np.array([0.0, 0.0, 0.0]), (1.0, 1.0), 1000
    z_true = pm.landmark_h(x0, lm)          # real range and bearing from the (unknown) pose
    cases = {'small noise': (0.05, math.radians(5.0)), 'large noise': (0.2, math.radians(20.0))}
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.3))
    for k, (name, sigma) in enumerate(cases.items()):
        # forward model: noisy measurements z of the landmark seen from x0
        zs = np.array([pm.sample_landmark_model(x0, lm, sigma, rng) for _ in range(n)])
        axes[0].scatter(zs[:, 0], np.degrees(zs[:, 1]), s=4, alpha=0.4,
                        label=f'{name}: σ = [{sigma[0]} m, {math.degrees(sigma[1]):.0f}°]')
        # inverse model: one candidate pose for each sampled measurement
        poses = np.array([pm.sample_pose_from_landmark(z, lm, (0.0, 0.0), rng) for z in zs])
        ax = axes[k + 1]
        ax.scatter(poses[:, 0], poses[:, 1], s=4, alpha=0.4, label=f'{n} candidate poses')
        sel = poses[rng.choice(n, 40, replace=False)]
        ax.quiver(sel[:, 0], sel[:, 1], np.cos(sel[:, 2]), np.sin(sel[:, 2]), color='tab:blue',
                  scale=25, width=0.004, alpha=0.6)
        ax.plot(*lm, 'ks', ms=8, label='landmark m')
        draw_pose(ax, x0, 'true pose (unknown)')
        ax.set_title(f'Inverse model, {name}: σ = [{sigma[0]} m, {math.degrees(sigma[1]):.0f}°]',
                     fontsize=9)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.axis('equal')
        ax.grid(True)
        ax.legend(fontsize=7, loc='lower right')
        r = np.hypot(poses[:, 0] - lm[0], poses[:, 1] - lm[1])
        print(f'landmark, {name}: range of candidate poses {r.mean():.3f} ± {r.std():.3f} m')
    axes[0].plot(z_true[0], math.degrees(z_true[1]), 'k+', ms=12, mew=2, label='true z')
    axes[0].set_title('Forward model: sampled z = [r, φ]', fontsize=9)
    axes[0].set_xlabel('range r [m]')
    axes[0].set_ylabel('bearing φ [deg]')
    axes[0].grid(True)
    axes[0].legend(fontsize=7)
    save(fig, 'task0_landmark_model')


def jacobians():
    x, y, th, v, w, dt, mx, my = sp.symbols(r'x y theta v omega \Delta{t} m_x m_y')
    g = sp.Matrix([x - v / w * sp.sin(th) + v / w * sp.sin(th + w * dt),
                   y + v / w * sp.cos(th) - v / w * sp.cos(th + w * dt),
                   th + w * dt])
    h = sp.Matrix([sp.sqrt((mx - x)**2 + (my - y)**2), sp.atan2(my - y, mx - x) - th])
    state = sp.Matrix([x, y, th])
    mats = {'G': g.jacobian(state), 'V': g.jacobian(sp.Matrix([v, w])), 'H': h.jacobian(state)}
    with open(os.path.join(OUT, 'task0_jacobians.tex'), 'w') as f:
        for name, m in mats.items():
            f.write(f'{name} = {sp.latex(sp.simplify(m))}\n\n')
    # the symbolic result must agree with the functions used in the filter
    point = {x: 1.0, y: -2.0, th: 0.7, v: 0.2, w: 0.5, dt: 0.05, mx: 2.0, my: 1.0}
    mu, u, lm = (1.0, -2.0, 0.7), (0.2, 0.5), (2.0, 1.0)
    for name, m, code in (('G', mats['G'], pm.motion_G(mu, u, 0.05)),
                          ('V', mats['V'], pm.motion_V(mu, u, 0.05)),
                          ('H', mats['H'], pm.landmark_H(mu, lm))):
        err = np.abs(np.array(m.subs(point).evalf(), dtype=float) - code).max()
        print(f'{name}: largest difference sympy vs code = {err:.2e}')


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    motion_samples()
    landmark_samples()
    jacobians()
