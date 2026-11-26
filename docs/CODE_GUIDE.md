# 📖 Code Guide / 代码介绍指南

**Language / 语言：** [🇬🇧 English](#-english) · [🇨🇳 中文](#-中文) · [⬅️ README](../README.md)

---

## 🇬🇧 English

**Jump to:** [R01 Bump&Go (`lab02_pkg`)](#r01--bumpgo-lab02_pkg) · [R02 EKF (`lab04_pkg`)](#r02--ekf-localization-lab04_pkg) · [R03 DWA (`lab07_pkg`)](#r03--dwa-obstacle-avoidance-and-following-lab07_pkg)

All three packages follow the same ROS 2 split: **pure-Python algorithms without ROS**, a **thin ROS 2 node**, **parameters in YAML**, a **launch file**, and **unit tests**.

---

## R01 — Bump&Go (`lab02_pkg`)

### 1. Package layout

The package follows the usual ROS 2 split: **algorithm without ROS**, a **thin ROS node**, **parameters in YAML**, a **launch file**, and **unit tests**.

```
src/lab02_pkg/
├── lab02_pkg/
│   ├── controller.py         # Bump&Go state machine (pure Python, no ROS)
│   ├── lidar_utils.py        # LiDAR sectors: closest point, median, invalid rays (pure Python)
│   └── bump_and_go_node.py   # ROS 2 node: parameters, topics, timers, safe stop
├── config/
│   ├── sim.yaml              # parameters for the Gazebo maze
│   └── real_robot.yaml       # parameters for the real TurtleBot3
├── launch/
│   └── bump_and_go.launch.py # starts the node with sim.yaml or real_robot.yaml
├── test/
│   ├── test_controller.py    # state machine, no ROS needed
│   ├── test_lidar_utils.py   # LiDAR helpers, no ROS needed
│   └── test_node_safety.py   # missing / stale sensor data -> zero velocity
├── package.xml               # metadata and dependencies
├── setup.py                  # installs config/ and launch/, registers "bump_and_go"
├── setup.cfg, resource/      # needed by ROS 2 (do not modify)
```

How the pieces talk to each other at every control step (10 Hz):

```
/scan ─► lidar_utils: front = sector_min(0°±15°)
                      left / right = side_metric(±90°±30°)  (min in sim, median on the real robot)
/odom ─► x, y, yaw                                     │
                                                       ▼
          controller.step(x, y, yaw, front, left, right) ─► (v, ω) ─► /cmd_vel
```

Run it:

```bash
ros2 launch lab02_pkg bump_and_go.launch.py              # simulation  (config/sim.yaml)
ros2 launch lab02_pkg bump_and_go.launch.py robot:=real  # real robot  (config/real_robot.yaml)
```

`ros2 run lab02_pkg bump_and_go` still works and uses the default values (= simulation).

### 2. `lidar_utils.py`

```python
angle = math.degrees(scan.angle_min + i * scan.angle_increment)   # angle of ray i
diff = (angle - center_deg + 180.0) % 360.0 - 180.0              # difference in [-180, 180)
```

- The angle of each ray is computed from the message, so the code works with any number of rays (230 in simulation, about 213 on the real LDS-02).
- `diff` handles the wrap-around: a ray at 355° is 5° away from 0°, not 355°.
- Angles: 0° = front, +90° = left, −90° = right (counter-clockwise positive).
- Positive `inf` = no return within range → counted as `range_max` (free).
- `NaN`, `−inf`, readings below 5 cm and above `range_max` are ignored. The real LDS-02 reports **NaN** where the beam hits nothing and declares `range_min = 0.0`, so without the 5 cm limit a 0.0 reading would look like a wall at 0 m.
- `sector_min` (front): the **closest** point, for safety.
- Sides: `side_metric` chooses `sector_min` or `sector_median`. On the real robot the **median** works better: with the minimum, one close corner hides a long free corridor and the robot turns the wrong way. In simulation the **minimum** is kept: the simulated LiDAR returns `inf` (= 3.5 m) for no return, so in open areas the median is 3.5 m on both sides and cannot tell them apart.
- An empty sector returns `None` and the controller stops.

### 3. `controller.py` — state machine

```
GO      front > threshold        → drive at max_linear_vel
        front ≤ threshold        → choose the side with the larger median, go to ROTATE
ROTATE  heading reached          → back to GO
        (turn_until_clear) turned > 20° and front > threshold + clear_margin → back to GO
        otherwise                → ω = clip(2·error, ±max_angular_vel), at least min_angular_vel
DONE    (stop_at_goal) near the goal → velocity 0
```

Key lines:

```python
self.target_yaw = yaw + turn                     # turn = ±turn_angle_deg
if c.snap_to_axes:
    self.target_yaw = round(self.target_yaw / (math.pi / 2)) * (math.pi / 2)
```

Snapping to 0/90/180/270° keeps the robot aligned with the simulated maze, whose walls are parallel to the odometry axes. On the real robot the axes depend on how the robot was placed (50° off → a 130° turn), so it is switched off.

```python
error = wrap_angle(self.target_yaw - yaw)        # atan2(sin, cos) -> [-pi, pi]
w = max(-c.max_angular_vel, min(c.max_angular_vel, 2.0 * error))   # P controller
if abs(w) < c.min_angular_vel:                                     # motor dead zone
    w = math.copysign(c.min_angular_vel, w)
```

`wrap_angle` makes the robot turn 2° (not 358°) from −179° to +179°. The minimum speed is a precaution: near the target the P controller asks for very little, and if the motors had a dead zone the robot would stay stuck in ROTATE. The simulated test in `scripts/deadzone_filter.py` shows this with an assumed dead zone of 0.25 rad/s; it was not measured on the real robot.

`step()` returns `(v, ω)` and sets `self.event` (e.g. `wall at 0.49 m (left 0.79, right 0.23) -> turn left`), which the node prints.

### 4. `bump_and_go_node.py` — ROS 2 node

| Code | Purpose |
|---|---|
| `for f in fields(ControllerConfig): self.declare_parameter(f.name, f.default)` | Every controller setting is a ROS parameter with the same name, so the YAML files can set it |
| `create_subscription(LaserScan, '/scan', ..., qos_profile_sensor_data)` | LiDAR; sensor QoS is required on the real robot |
| `create_subscription(Odometry, '/odom', ..., 10)` | Position and orientation (quaternion → yaw with `euler_from_quaternion`) |
| `create_publisher(Twist, '/cmd_vel', 10)` | Velocity commands |
| `create_timer(period, self.control_loop)` | Control step every `1 / control_frequency` s |
| `sensors_ready()` | LiDAR and odometry must be newer than `sensor_timeout_sec`, otherwise zero velocity |
| `status_log()` (every 2 s) | `state=GO front=1.85 m v=0.15 m/s w=0.00 rad/s odom=OK`, or which sensor is missing and since when |
| `stop_robot()` + `SignalHandlerOptions.NO` | On `Ctrl+C` our code still sends zero velocity three times before ROS shuts down |

Callbacks are passed **without parentheses** (`self.scan_callback`): we hand the function to ROS, which calls it when a message arrives.

### 5. ⚙️ Parameters

Set in `config/sim.yaml` and `config/real_robot.yaml` (after editing: `colcon build`). For a quick test: `ros2 run lab02_pkg bump_and_go --ros-args -p name:=value`.

| Parameter | Sim | Real | Unit | Meaning |
|---|---|---|---|---|
| `max_linear_vel` | 0.22 | **0.15** | m/s | Forward speed. **At 0.22 the real robot only turned and never drove forward** (10/29) |
| `max_angular_vel` | 1.5 | 1.5 | rad/s | Maximum turning speed (robot limit 2.84) |
| `obstacle_threshold` | 0.5 | 0.5 | m | Front distance that triggers a turn |
| `stop_at_goal` | true | false | – | Stop near the goal (simulation only) |
| `goal_x`, `goal_y`, `goal_tolerance` | 6.5, 3.0, 0.8 | – | m | Green cube in the Lab02 maze |
| `rotation_tolerance_deg` | 5 | 5 | deg | A turn is finished below this yaw error |
| `min_angular_vel` | 0.3 | 0.3 | rad/s | Precaution against a motor dead zone |
| `turn_angle_deg` | 90 | **45** | deg | Largest turn per bump; in a corner the robot bumps again and turns more |
| `snap_to_axes` | true | **false** | – | Round the turn to the maze axes |
| `turn_until_clear` | false | **true** | – | Stop turning as soon as the front is free |
| `clear_margin` | 0.3 | 0.3 | m | Front is free above `obstacle_threshold + clear_margin` |
| `side_metric` | min | **median** | – | How to measure how open each side is |
| `control_frequency` | 10 | 10 | Hz | Control loop |
| `sensor_timeout_sec` | 1.0 | 1.0 | s | Stop if LiDAR or odometry is older than this |

### 6. Tests

```bash
colcon build --symlink-install && colcon test --packages-select lab02_pkg && colcon test-result --verbose
```

`test_controller.py` and `test_lidar_utils.py` need no ROS: they feed numbers to the pure modules. This is the main reason for keeping the algorithm out of the node.

### 7. `scripts/plot_bag.py`

| Section | What it does |
|---|---|
| 1. Read the bag | `rosbag2_py.SequentialReader` reads every message; `deserialize_message` turns bytes into ROS messages; time is the recording time stamp in seconds |
| `pose_series` | Extracts `t, x, y, yaw` from Odometry messages (yaw from the quaternion: `2·atan2(z, w)` for a planar rotation) |
| 2. Trajectory | XY plot of `/odom` and `/ground_truth`, with start and end markers, equal axis scale |
| 3. Velocity commands | `/cmd_vel` linear and angular velocity vs time |
| 4. Odometry error | Ground truth is interpolated (`np.interp`) at the odometry time stamps; the error is the Euclidean distance. Skipped if `/ground_truth` is not in the bag (real robot) |

All figures are saved as **PDF** (vector, for the report) and **PNG** (preview), with axis labels in SI units and legends.

### 8. Questions to prepare for the oral exam (R01)

1. Why are the callbacks passed without parentheses?
2. Why does the robot stop closer to the wall than `obstacle_threshold`?
3. Why is the target yaw rounded to multiples of 90° in simulation but not on the real robot?
4. What does `atan2(sin(e), cos(e))` do, and why is it needed?
5. Why do we use a minimum angular velocity, and what does the dead-zone test prove (and not prove)?
6. Why the minimum for the front sector, but the median for the sides on the real robot? Why not in simulation?
7. Why are `inf` treated as free space and `NaN` ignored?
8. Why do odometry and ground truth diverge over time? Why is the error larger on the real robot?
9. Why is the algorithm in a separate module without ROS?
10. Lifted on a stand, the robot never finishes a turn. Why? (Hint: `use_imu: true` in the TurtleBot3 driver.)

---

## R02 — EKF localization (`lab04_pkg`)

### 1. Package layout

```
src/lab04_pkg/
├── lab04_pkg/
│   ├── probabilistic_models.py   # Motion & landmark models + Jacobians (Task 0/1/2, pure Python)
│   ├── ekf.py                    # RobotEKF predict & Joseph-form update (pure Python)
│   └── ekf_node.py               # ROS 2 node: 20 Hz prediction, landmark/odom/IMU updates, /ekf pub
├── config/
│   ├── sim.yaml                  # Gazebo Lab04 parameters
│   ├── real_robot.yaml           # Real TurtleBot3 parameters
│   └── landmarks_thu1211.yaml    # Example real-arena landmark coordinates
├── launch/
│   └── ekf.launch.py             # Args: robot (sim|real), fusion (true|false), use_sim_time
├── scripts/
│   ├── task0_models.py           # Task 0 sampling plots + sympy Jacobian checks (no ROS)
│   ├── evaluate_ekf.py           # Bag plots + RMSE/MAE against /ground_truth
│   ├── replay_filters.py         # Run Task 1 and Task 2 offline on the same bag
│   ├── run_sim_experiment.sh     # Automated 150 s Gazebo run + Bump&Go + bag
│   └── robot_task3.sh            # Real-robot preflight check + build + EKF + bag recorder
├── test/
│   ├── test_models.py            # Numeric Jacobian checks, wrapping, filter convergence
│   └── test_{copyright,flake8,pep257}.py
├── LICENSE, package.xml, setup.py, setup.cfg, resource/
```

Data flow:

```
/odom (v, ω) ──► 20 Hz timer: predict(g, G, V, M) [+ fuse /odom & /imu if fusion:=true] ─► /ekf
/landmarks (or /camera/landmarks) ──► per-tag update(h, H, Q_landmark, angle_idx=1) ─────┘
```

### 2. Core algorithm (`probabilistic_models.py`, `ekf.py`, `ekf_node.py`)

- **Velocity motion model (`motion_g`, `motion_G`, `motion_V`)**:
  - For $|\omega| \ge 10^{-6}\text{ rad/s}$, moves along a circular arc of radius $r = v/\omega$:
    $x' = x - r\sin\theta + r\sin(\theta + \omega\Delta t)$, $y' = y + r\cos\theta - r\cos(\theta + \omega\Delta t)$, $\theta' = \theta + \omega\Delta t$.
  - For $|\omega| < 10^{-6}\text{ rad/s}$ (`SMALL_W`), switches to the exact straight-line limit to avoid dividing by zero, with matching analytical Jacobians $G$ (state) and $V$ (control).
- **Landmark measurement model (`landmark_h`, `landmark_H`)**:
  - Expected observation $z = [\sqrt{\Delta x^2 + \Delta y^2},\; \text{wrap}(\text{atan2}(\Delta y, \Delta x) - \theta)]^T$ for known tag position $(m_x, m_y)$.
  - `landmark_H(mu, landmark, dim=3|5)` returns a $2 \times 3$ or $2 \times 5$ Jacobian (velocity columns are zero because tags observe pose, not velocity).
- **Task 2 sensor fusion (`fusion:=true`, state $[x, y, \theta, v, \omega]^T$)**:
  - `extended_g` propagates pose using the state's own $(v, \omega)$ assuming constant velocity over $\Delta t$; `extended_G` puts `motion_G` in the top-left $3\times 3$ block and `motion_V` in the top-right $3\times 2$ block; `extended_V` injects process noise on $(v, \omega)$.
  - Once per 20 Hz prediction step, `fuse_velocities()` applies linear updates from `/odom` (`odom_H` observes $[v, \omega]$) and `/imu` (`imu_H` observes $[\omega]$) whenever fresh messages have arrived.
- **Filter update (`RobotEKF`)**:
  - Wraps the bearing innovation with `atan2(sin, cos)` (`angle_idx=1`) and wraps heading $\theta$ (`THETA = 2`) after every predict/update.
  - Uses the **Joseph form** $\Sigma = (I - KH)\Sigma(I - KH)^T + KQK^T$ so the covariance matrix stays symmetric and positive semi-definite despite floating-point round-off.
- **ROS 2 node (`ekf_node.py`)**:
  - Reads `dt` from the ROS clock (`self.get_clock().now()`) and skips if `dt <= 0` (paused simulation).
  - Updates landmarks sequentially inside `landmark_callback`, ignoring unknown tag IDs with a one-time warning.
  - Publishes `/ekf` with a planar quaternion (`z = sin(θ/2)`, `w = cos(θ/2)`, no `tf_transformations` dependency) and maps the $3\times 3$ pose covariance into ROS's $6\times 6$ array at indices `(x, y, yaw) = (0, 1, 5)`.

### 3. ⚙️ Parameters (`sim.yaml` vs `real_robot.yaml`)

| Parameter | `sim.yaml` | `real_robot.yaml` | Unit | Meaning |
|---|---|---|---|---|
| `use_sim_time` | `true` | `false` | – | Set `use_sim_time:=true` when replaying a real bag with `--clock` |
| `landmarks_topic` | `/landmarks` | `/camera/landmarks` | – | Simulated vs AprilTag detector topic |
| `landmarks_file` | (`turtlebot3_perception` default) | `landmarks_thu1211.yaml` | – | Bare filename resolves in `lab04_pkg/config/`; absolute path also works |
| `init_pose` | `[-2.0, -0.5, 0.0]` | `[0.0, 0.0, 0.0]` | m, m, rad | Initial pose in map frame (measure before each real run) |
| `init_std` | `[0.1, 0.1, 0.1]` | `[0.3, 0.3, 0.3]` | m, m, rad | Initial pose uncertainty (larger on real robot) |
| `prediction_rate` | `20.0` | `20.0` | Hz | Fixed prediction timer frequency |
| `std_v` / `std_w` | `0.05` / `0.1` | `0.05` / `0.1` | m/s, rad/s | Process noise on linear and angular velocity |
| `std_range` / `std_bearing_deg` | `0.05` / `2.0` | `0.1` / `3.0` | m, deg | Landmark range & bearing noise (larger than simulator 0.01 m / 1°) |
| `std_odom_v` / `std_odom_w` | `0.01` / `0.02` | `0.02` / `0.05` | m/s, rad/s | Task 2 `/odom` velocity measurement noise |
| `std_imu_w` | `0.02` | `0.02` | rad/s | Task 2 `/imu` yaw-rate measurement noise |

### 4. Tests & verification

```bash
colcon test --packages-select lab04_pkg && colcon test-result --verbose
```

- `test_models.py`: compares analytical `motion_G`, `motion_V`, `landmark_H`, and `extended_G` against central finite differences (including $\omega = 0$); checks straight-line continuity (`w=0` vs `w=1e-5`), quarter-circle kinematics, `sample_pose_from_landmark` inverse consistency, bearing residual wrap across $\pm\pi$, and 400-step EKF convergence with covariance symmetry.
- `test_copyright.py`, `test_flake8.py`, `test_pep257.py`: GPL-3.0 header and ROS 2 style checks.

### 5. Oral exam questions (R02 — with brief answers)

1. **Why does the velocity motion model have a separate branch for $|\omega| < 10^{-6}$?**
   The circular-arc formula divides by $\omega$ ($r = v/\omega$), which explodes as $\omega \to 0$. The straight-line branch is its exact limit and matches numerically at small $\omega$.
2. **Why use the Joseph form $\Sigma = (I-KH)\Sigma(I-KH)^T + KQK^T$ instead of $(I-KH)\Sigma$?**
   Floating-point subtraction in $(I-KH)\Sigma$ can break symmetry or produce tiny negative eigenvalues; the Joseph form is a sum of two symmetric positive semi-definite terms, so $\Sigma$ stays symmetric and well-conditioned.
3. **Why must the bearing innovation $y_\phi = z_\phi - \hat{z}_\phi$ be wrapped to $[-\pi, \pi]$?**
   If the predicted bearing is $+\pi - 0.01$ and the measurement is $-\pi + 0.01$, raw subtraction gives $-2\pi + 0.02$ instead of $+0.02$, causing the filter to jump by almost a full revolution.
4. **What is the difference between Task 1 (`fusion:=false`) and Task 2 (`fusion:=true`)?**
   Task 1 uses a 3D state $[x, y, \theta]$ driven directly by `/odom` velocities as control inputs. Task 2 augments the state to 5D $[x, y, \theta, v, \omega]$ with a constant-velocity prediction and fuses `/odom` ($v, \omega$) and `/imu` ($\omega$) as measurements.
5. **In simulation, Task 2 improved heading RMSE (~20–30 %) on the same bag, but not position RMSE (both 0.010 m). Why?**
   The IMU only measures angular velocity $\omega$, which sharpens heading between landmark updates; position accuracy is already dominated by the dense range-bearing landmark updates.
6. **Why are `std_range` (0.05 m) and `std_bearing_deg` (2.0°) in `sim.yaml` larger than the simulator's nominal noise (0.01 m, 1°)?**
   Setting $Q$ too tight makes the filter overconfident and sensitive to linearization error and timestamp lag between prediction and landmark messages; slightly inflating $Q$ keeps covariance consistent and avoids jitter.
7. **Why should a real-robot run start where the camera sees at least two AprilTags?**
   With only one landmark (range + bearing), any pose on the circle around the tag can match the measurement if the initial heading is uncertain; two or more tags break the ambiguity immediately.
8. **On the real robot, `/ekf` and `/odom` ended 0.28 m apart. Does that mean our EKF error is 0.28 m?**
   No. Without motion-capture ground truth, 0.28 m only measures the disagreement between `/ekf` and drifting wheel odometry; it confirms the filter stays consistent with the tags, not its absolute error.

---

## R03 — DWA obstacle avoidance and following (`lab07_pkg`)

### 1. Package layout

```
src/lab07_pkg/
├── lab07_pkg/
│   ├── scan_utils.py         # 3-pt median filter, cleaning, health check, 24 sectors, safety stop
│   ├── dwa.py                # Dynamic window, rollouts, Task 1/2 scoring, margin band, recovery
│   ├── navigation.py         # Goal/follow manager, trail, stuck recovery, Lost, collision lock
│   └── dwa_node.py           # ROS 2 node: 15 Hz timer, sensor QoS, /reset_nav, safe Ctrl+C stop
├── config/
│   ├── sim.yaml              # Gazebo hard_maze.world parameters
│   └── real_robot.yaml       # Real TurtleBot3 Burger parameters
├── launch/
│   └── dwa.launch.py         # Args: robot, mode, goal_source, target_id, use_sim_time
├── scripts/
│   ├── evaluate_nav.py       # Rosbag2 metrics & trajectory/velocity plots (sim & real)
│   ├── run_goal_trials.sh    # Repeated Gazebo Task 1 runner with process-group cleanup & CSV
│   ├── run_single_trial.py   # Subscribes first, publishes goal, logs simulation duration
│   └── offline_sim.py        # Pure-Python 2D helper simulator (not a replacement for Gazebo)
├── test/
│   ├── test_{scan_utils,dwa,navigation,follow_mode,evaluate_nav}.py
│   ├── test_{review_must_fix,review_should_fix,review2_fixes,review4_fixes}.py
│   ├── test_{task5_review3,task7_standoff,task8_follow_recovery,task9_margin_and_lost,task10_median_filter}.py
│   └── test_{copyright,flake8,pep257}.py
├── LICENSE, package.xml, setup.py, setup.cfg, resource/
```

Control data flow at 15 Hz:

```
/scan ─► scan_utils: median3_filter ─► health check ─► safety stop (center dist < r + tol)
                                                    └─► 24-sector obstacles (odom pose at scan stamp)
/odom ─► odom_history + (v, ω) ──► navigation.py (goal / trail / stuck recovery / Lost / lock)
/goal_pose | /dynamic_goal_pose | /camera/landmarks ──┘
                                                       ▼
                     dwa.py: dynamic window ─► 210 rollouts ─► filter & score ─► /cmd_vel
```

### 2. Core algorithms

- **`scan_utils.py`**:
  - `median3_filter(ranges)` runs a 360° circular 3-point median before cleaning (controlled by `scan_median_filter: true`). Non-finite rays (`NaN`, `±inf`) are excluded and preserved unchanged; a finite ray with two invalid neighbors keeps its value; a wall-edge ray with one finite neighbor takes the upper median (`[0.435, 0.293, inf] -> 0.435`). Trade-off: 1-ray-wide objects (~8 mm at 0.3 m, ~2.7 cm at 1 m) are filtered out, while boxes, walls and human legs are unaffected.
  - `clean_scan_ranges` maps `+inf` to `eff_max`, applies `nan_policy` (`'min'` in sim, `'max'` on real LDS-02), drops values `< min_valid_range` (`0.05 m`, filtering LDS-02 `0.0` glitches), and clamps above `max_scan_range`.
  - `check_scan_health` stops the robot if valid rays `< min_valid_ratio` (`20%`) or `NaN` rays `> max_nan_ratio` (`95%`, catching all-`NaN` failure under `nan_policy='max'`).
  - `downsample_scan` splits the scan into `num_ranges = 24` sectors, keeping each sector's minimum distance **and true ray angle**, then transforms with `laser_offset_x = -0.032 m`.
  - `check_safety_stop` checks all filtered rays from the **robot center** against `robot_radius + collision_tol`.
- **`dwa.py`** (adapted from `SESASR-Course/planning_control_methods`, GPL-3.0):
  - **Dynamic window**: intersects static velocity limits with acceleration bounds over `dt = 1/15 s`. Samples `v_samples = 10` × `w_samples = 21` (odd count, forcing exact $\omega = 0$ for straight motion). On the real robot, `window_source = 'last_cmd'` avoids static-friction deadlocks when measured odom speed is initially zero.
  - **Rollout**: integrates trajectories over `sim_time = 1.5 s` with `time_granularity = 0.1 s` (decoupled from `dt`). `max_ang_vel = 1.5 rad/s` keeps $\omega \cdot \text{sim\_time} = 2.25 < \pi$ so rollout headings do not wrap past 180°.
  - **Two-layer safety & margin band**:
    - Planning collision radius is `collision_radius + planning_margin` (`0.285 + 0.10 = 0.385 m` in sim; `0.305 + 0.10 = 0.405 m` on real). Colliding trajectories are removed **before** normalization.
    - Inside the margin band `[collision_radius + band_floor_buffer, planning_collision_radius)` (`band_floor_buffer = 0.03 m`), `band_entry_min` is locked on entry (cleared on exit/reset) and only trajectories with `future_min >= max(band_entry_min, band_floor_radius) - 1e-6` are allowed — preventing step-by-step creep toward the emergency stop line.
  - **Objective functions**:
    - Task 1 (`goal`): $J = \alpha\cdot\text{heading} + \beta\cdot\text{vel} + \gamma\cdot\text{dist}_{\text{obst}}$.
    - Task 2 (`follow`): $J = \alpha\cdot\text{heading} + \beta\cdot\text{vel}' + \gamma\cdot\text{dist}_{\text{obst}} + \delta\cdot\text{dist}_{\text{target}}$, where $\text{vel}'$ slows smoothly below `slowdown_dist_threshold = 1.2 m` and $\text{dist}_{\text{target}}$ combines a Gaussian around `desired_target_dist = 0.8 m` with a Gaussian bearing term matching the ±33° camera FoV.
    - When the obstacle-clearance span across safe trajectories is `< obs_norm_min_span` (`0.05 m`), linear scaling `clearance / obstacle_max_dist` is used instead of min-max so millimeter noise in narrow corridors is not amplified.
- **`navigation.py` & `dwa_node.py`**:
  - Matches each `/scan` and `/camera/landmarks` message to the closest timestamped pose in `odom_history` (guarded by `max_stamp_skew_sec = 0.5 s`).
  - `goal_source = 'landmark'` locks a one-shot static goal backed off by `landmark_goal_standoff = 0.5 m` from the tag toward the robot.
  - Follow-mode trail & stuck recovery: records target trail points every `trail_spacing = 0.10 m` (clears on target jump `> 2.5 m`, pops within `trail_reach_dist = 0.40 m`) so `heading` aims around corners while `vel'`/`dist_target` still use the current target. If `v < 0.03 m/s` for `stuck_time_sec = 2.0 s` while far from the target, switches temporarily (up to `5.0 s`, or until moving `0.3 m`) to progress-based scoring over safe trajectories only (with 0.003/0.08/0.25 m lookahead tie-breaking when spinning in place).
  - `Lost` event: if distance stays `> 3.0 m` for `10.0 s`, publishes `Lost` once on `/nav_result` without stopping (`active` stays `True`), and re-arms once back within `3.0 m`.
  - `Collision` locks navigation until `/reset_nav` (`std_srvs/Trigger`) is called; after reset, a protected stop-zone escape allows turning/moving away while keeping physical-contact and worsening-distance hard stops active.

### 3. ⚙️ Key parameters (`sim.yaml` vs `real_robot.yaml`)

| Parameter | `sim.yaml` | `real_robot.yaml` | Unit | Meaning |
|---|---|---|---|---|
| `control_rate` / `sim_time` / `time_granularity` | `15` / `1.5` / `0.1` | `15` / `1.5` / `0.1` | Hz, s, s | Control loop, rollout horizon and step |
| `v_samples` / `w_samples` | `10` / `21` | `10` / `21` | – | Velocity grid (21 guarantees $\omega = 0$) |
| `max_lin_vel` / `max_ang_vel` | `0.22` / `1.5` | **`0.15`** / `1.5` | m/s, rad/s | Kinematic caps (0.15 verified on real Burger) |
| `max_lin_acc` / `max_ang_acc` | `0.5` / `3.2` | `0.4` / `2.0` | m/s², rad/s² | Dynamic window acceleration limits |
| `window_source` | `'odom'` | **`'last_cmd'`** | – | Base speed for dynamic window |
| `robot_radius` / `collision_tol` | `0.105` / `0.18` | `0.105` / `0.20` | m | Emergency stop radius (`0.285` vs `0.305 m`) |
| `planning_margin` / `band_floor_buffer` | `0.10` / `0.03` | `0.10` / `0.03` | m | Extra DWA buffer and margin-band floor |
| `weight_heading` / `vel` / `obs` / `target` | `0.40` / `0.30` / `0.30` / `0.35` | same | – | Objective weights $\alpha, \beta, \gamma, \delta$ |
| `obs_norm_min_span` / `obstacle_max_dist` | `0.05` / `1.0` | `0.05` / `1.0` | m | Anti-noise norm threshold and clearance cap |
| `nan_policy` / `scan_median_filter` | `'min'` / `true` | **`'max'`** / `true` | – | NaN handling and 3-point median spike filter |
| `min_valid_ratio` / `max_nan_ratio` | `0.20` / `0.95` | `0.20` / `0.95` | – | Scan health check thresholds |
| `follow_source` / `require_target_id` | `'dynamic_goal'` / `false` | `'landmarks'` / **`true`** | – | Follow input; real robot requires `target_id` |
| `goal_source` / `landmark_goal_standoff` | `'topic'` / `0.5` | `'topic'` / `0.5` | –, m | Static goal source and tag backoff distance |
| `desired_target_dist` / `slowdown_dist_threshold` | `0.8` / `1.2` | `0.8` / `1.2` | m | Follow distance and `vel'` slowdown range |
| `follow_use_trail` / `stuck_time_sec` | `true` / `2.0` | `true` / `2.0` | –, s | Trail aiming and stuck recovery trigger |
| `lost_dist` / `lost_dist_time_sec` | `3.0` / `10.0` | `3.0` / `10.0` | m, s | Non-stopping `Lost` report threshold |

### 4. Tests & verification

```bash
colcon test --packages-select lab07_pkg && colcon test-result --verbose
```

78 tests pass in ROS 2 (`pytest`): 75 pure-Python unit & regression tests across 14 files (covering LiDAR filtering/median/health, DWA window/scoring/margin-band anti-creep, Task 1 & Task 2 navigation, trail & corridor stuck recovery, standoff, non-stopping `Lost`, collision lock & `/reset_nav`, and `evaluate_nav.py` metrics/plotting) plus `test_copyright.py`, `test_flake8.py`, and `test_pep257.py` (0 warnings).

### 5. Oral exam questions (R03 — with brief answers)

1. **Why are `dt = 1/15 s` and `time_granularity = 0.1 s` decoupled, and why cap `max_ang_vel` at `1.5 rad/s`?**
   `dt` bounds the dynamic window over one control cycle, whereas `time_granularity` integrates the 1.5 s rollout cheaply (15 steps per candidate). Capping $\omega$ at `1.5 rad/s` keeps $\omega \cdot 1.5 = 2.25 < \pi$ so simulated arcs never wrap past 180° and confuse the heading score.
2. **Why must colliding trajectories be removed *before* normalizing the objective terms?**
   If unsafe trajectories stay in min-max normalization, they stretch the score range and distort the relative weights among safe trajectories (and a fast unsafe path could still outscore a slow safe path).
3. **Why do we separate planning collision radius (`+0.10 m` margin) from the emergency stop radius, and why anchor the margin band at `band_entry_min` with a `0.03 m` floor buffer?**
   Without a planning margin, deceleration inertia and discrete LiDAR angles push the robot over the hard-stop line. Anchoring at `band_entry_min` (with no downward tolerance) and stopping margin relaxation `0.03 m` above the hard stop prevents 15 Hz step-by-step creep into `Collision`.
4. **Why do 4 goals behind the central walls in `hard_maze.world` end in `Timeout`?**
   Regular DWA is a purely local planner: its heading term points straight at the goal through the wall, creating a local minimum where every forward motion hits the wall. Solving those requires a global planner (e.g. A* / NF1) providing intermediate waypoints.
5. **Why does Task 2 replace `vel` with `vel'` and add `dist_target`, trail aiming, and stuck recovery?**
   Raw `vel` would make the robot rush past or collide with a slow target; `vel'` slows smoothly below 1.2 m, and `dist_target` holds ~0.8 m with the target centered in the ±33° camera FoV. Aiming at the target's trail keeps the heading around corners, and progress-based stuck recovery breaks local minima in narrow corridors without relaxing collision checks.
6. **What does `median3_filter` fix, and what is its trade-off?**
   It removes single-ray inward LiDAR spikes (both mid-wall and on wall edges next to `inf`) that otherwise cause false `Collision` stops. The trade-off is that an obstacle narrower than one ray (~8 mm at 0.3 m, ~2.7 cm at 1 m) would be ignored — much thinner than arena walls, boxes, or human legs.
7. **On the real LDS-02, why set `nan_policy: 'max'` and still enforce `max_nan_ratio: 0.95`?**
   The real LDS-02 outputs `NaN` in open space; treating `NaN` as `min` would trigger an immediate false collision. Under `'max'`, however, a broken all-`NaN` scan looks identical to empty space, so `max_nan_ratio` stops the robot if >95 % of rays are `NaN`.
8. **Why does `real_robot.yaml` use `window_source: 'last_cmd'` instead of `'odom'`?**
   At 15 Hz with `max_lin_acc = 0.4 m/s²`, the first speed command from rest is only ~0.027 m/s, which may not overcome static friction. If odom stays at 0, an `'odom'` window can never grow; `'last_cmd'` ramps smoothly up until the wheels break free.
9. **Why add `landmark_goal_standoff = 0.5 m` and `require_target_id = true` for the real robot?**
   An AprilTag is usually on a wall or stand, so commanding the tag's exact coordinate would hit the `0.305 m` emergency stop before reaching `goal_dist_tol = 0.25 m`; backing off 0.5 m lets the robot arrive cleanly. `require_target_id` prevents fixed arena localization tags from hijacking follow mode.
10. **Why does `Collision` lock the robot until `/reset_nav`, while `Lost` only publishes a notice without stopping?**
    After a close-obstacle stop, auto-restarting when a person or moving target shifts could cause a crash, so explicit `/reset_nav` confirmation is required. By contrast, when the robot briefly falls >3 m behind around a tight corner, stopping would prevent it from ever catching up.

---

## 🇨🇳 中文

**快速跳转：** [R01 Bump&Go（`lab02_pkg`）](#r01--bumpgo-控制器lab02_pkg) · [R02 EKF（`lab04_pkg`）](#r02--ekf-定位lab04_pkg) · [R03 DWA（`lab07_pkg`）](#r03--dwa-避障与目标跟随lab07_pkg)

三个包采用统一的 ROS 2 分层：**纯 Python 算法模块（不依赖 ROS）**、**薄 ROS 2 节点**、**YAML 参数**、**launch 启动文件**和**单元测试**。

---

## R01 — Bump&Go 控制器（`lab02_pkg`）

### 1. 包的结构

按 ROS 2 常见的分法：**算法不依赖 ROS**、**ROS 节点只做连接**、**参数放 YAML**、**用 launch 文件启动**、**配单元测试**。

```
src/lab02_pkg/
├── lab02_pkg/
│   ├── controller.py         # Bump&Go 状态机（纯 Python，不依赖 ROS）
│   ├── lidar_utils.py        # 雷达扇区：最近点、中位数、无效光束（纯 Python）
│   └── bump_and_go_node.py   # ROS 2 节点：参数、话题、定时器、安全停车
├── config/
│   ├── sim.yaml              # 仿真迷宫用的参数
│   └── real_robot.yaml       # 真车用的参数
├── launch/
│   └── bump_and_go.launch.py # 选 sim.yaml 或 real_robot.yaml 启动节点
├── test/
│   ├── test_controller.py    # 状态机测试，不需要 ROS
│   ├── test_lidar_utils.py   # 雷达工具测试，不需要 ROS
│   └── test_node_safety.py   # 传感器缺失/超时 → 零速度
├── package.xml               # 包信息和依赖
├── setup.py                  # 安装 config/ 和 launch/，登记 "bump_and_go"
├── setup.cfg, resource/      # ROS 2 需要（不要改）
```

每个控制周期（10 Hz）各部分怎么配合：

```
/scan ─► lidar_utils：front = sector_min(0°±15°)
                      left / right = side_metric(±90°±30°)（仿真取最近点，真车取中位数）
/odom ─► x, y, yaw                                     │
                                                       ▼
          controller.step(x, y, yaw, front, left, right) ─► (v, ω) ─► /cmd_vel
```

运行：

```bash
ros2 launch lab02_pkg bump_and_go.launch.py              # 仿真（config/sim.yaml）
ros2 launch lab02_pkg bump_and_go.launch.py robot:=real  # 真车（config/real_robot.yaml）
```

`ros2 run lab02_pkg bump_and_go` 仍然能用，用的是默认值（= 仿真参数）。

### 2. `lidar_utils.py`

```python
angle = math.degrees(scan.angle_min + i * scan.angle_increment)   # 第 i 条光束的角度
diff = (angle - center_deg + 180.0) % 360.0 - 180.0              # 角度差，范围 [-180, 180)
```

- 每条光束的角度用消息里的字段现算，光束多少条都能用（仿真 230 条，真车 LDS-02 约 213 条）。
- `diff` 处理「绕一圈」：355° 和 0° 只差 5°，不是 355°。
- 角度方向：0° 正前方，+90° 左边，−90° 右边（逆时针为正）。
- 正无穷 `inf` = 量程内没打到东西 → 当作 `range_max`（畅通）。
- `NaN`、负无穷、小于 5 cm 和大于 `range_max` 的读数都丢弃。真车 LDS-02 打空时返回的是 **NaN**，而且报的 `range_min = 0.0`；不设 5 cm 下限的话，读数 0.0 会被当成 0 米处有墙。
- `sector_min`（前方）：取**最近点**，为了安全。
- 左右：用 `side_metric` 选 `sector_min` 或 `sector_median`。真车上**中位数**更好：用最近点的话，一个近处的墙角就会把后面一整条长通道「挡住」，车会转错方向。仿真里保留**最近点**：仿真雷达打空返回 `inf`（按 3.5 m 算），空旷处两边的中位数都是 3.5 m，分不出哪边更空。
- 扇区没有有效数据时返回 `None`，控制器停车。

### 3. `controller.py`：状态机

```
GO      前方 > 阈值          → 以 max_linear_vel 前进
        前方 ≤ 阈值          → 选中位数更大的一边，进入 ROTATE
ROTATE  转到目标角度          → 回到 GO
        （turn_until_clear）已转 > 20° 且前方 > 阈值 + clear_margin → 回到 GO
        否则                 → ω = 限幅(2·误差, ±max_angular_vel)，且不小于 min_angular_vel
DONE    （stop_at_goal）到达目标附近 → 速度为 0
```

关键代码：

```python
self.target_yaw = yaw + turn                     # turn = ±turn_angle_deg
if c.snap_to_axes:
    self.target_yaw = round(self.target_yaw / (math.pi / 2)) * (math.pi / 2)
```

取整到 0/90/180/270° 能让车和仿真迷宫对齐，因为仿真里的墙和里程计坐标轴平行。真车的坐标轴取决于开机时车头朝哪（歪 50° 就会变成一次转 130°），所以真车上关掉。

```python
error = wrap_angle(self.target_yaw - yaw)        # atan2(sin, cos) → [-π, π]
w = max(-c.max_angular_vel, min(c.max_angular_vel, 2.0 * error))   # P 控制
if abs(w) < c.min_angular_vel:                                     # 电机死区
    w = math.copysign(c.min_angular_vel, w)
```

`wrap_angle` 让车从 −179° 到 +179° 只转 2°，而不是 358°。最小转速是预防措施：快转到位时，P 控制给出的转速很小，如果电机有死区，车会一直卡在 ROTATE。`scripts/deadzone_filter.py` 的仿真测试用假设的 0.25 rad/s 死区演示了这一点，这个值没有在真车上测过。

`step()` 返回 `(v, ω)`，并把这一步的决定写进 `self.event`（例如 `wall at 0.49 m (left 0.79, right 0.23) -> turn left`），由节点打印出来。

### 4. `bump_and_go_node.py`：ROS 2 节点

| 代码 | 作用 |
|---|---|
| `for f in fields(ControllerConfig): self.declare_parameter(f.name, f.default)` | 控制器的每个设置都是同名的 ROS 参数，YAML 文件就能设置它们 |
| `create_subscription(LaserScan, '/scan', ..., qos_profile_sensor_data)` | 雷达；真车上必须用传感器 QoS |
| `create_subscription(Odometry, '/odom', ..., 10)` | 位置和朝向（四元数用 `euler_from_quaternion` 转成 yaw） |
| `create_publisher(Twist, '/cmd_vel', 10)` | 发速度指令 |
| `create_timer(period, self.control_loop)` | 每 `1 / control_frequency` 秒执行一次控制 |
| `sensors_ready()` | 雷达和里程计都要比 `sensor_timeout_sec` 新，否则发零速度 |
| `status_log()`（每 2 秒） | `state=GO front=1.85 m v=0.15 m/s w=0.00 rad/s odom=OK`，或者哪个传感器缺了、缺了多久 |
| `stop_robot()` + `SignalHandlerOptions.NO` | 按 `Ctrl+C` 时，ROS 关闭前我们的代码还能连发三次零速度 |

回调函数**不带括号**传入（写 `self.scan_callback`）：把函数本身交给 ROS，消息到了由 ROS 来调用。

### 5. ⚙️ 参数

在 `config/sim.yaml` 和 `config/real_robot.yaml` 里设置（改完要 `colcon build`）。临时试一下可以用：`ros2 run lab02_pkg bump_and_go --ros-args -p 参数名:=值`。

| 参数 | 仿真 | 真车 | 单位 | 含义 |
|---|---|---|---|---|
| `max_linear_vel` | 0.22 | **0.15** | m/s | 前进速度。**设 0.22 时真车只转不走**（10/29） |
| `max_angular_vel` | 1.5 | 1.5 | rad/s | 最大转速（机器人上限 2.84） |
| `obstacle_threshold` | 0.5 | 0.5 | m | 前方小于它就转弯 |
| `stop_at_goal` | true | false | – | 到目标附近停车（只用于仿真） |
| `goal_x`、`goal_y`、`goal_tolerance` | 6.5、3.0、0.8 | – | m | Lab02 迷宫绿块 |
| `rotation_tolerance_deg` | 5 | 5 | deg | 偏航角误差小于它就算转完 |
| `min_angular_vel` | 0.3 | 0.3 | rad/s | 预防电机死区 |
| `turn_angle_deg` | 90 | **45** | deg | 每次最多转多少；墙角里会再触发一次接着转 |
| `snap_to_axes` | true | **false** | – | 转向角对齐迷宫坐标轴 |
| `turn_until_clear` | false | **true** | – | 前方一空出来就停止转向 |
| `clear_margin` | 0.3 | 0.3 | m | 前方 > `obstacle_threshold + clear_margin` 算空 |
| `side_metric` | min | **median** | – | 用什么衡量左右哪边更空 |
| `control_frequency` | 10 | 10 | Hz | 控制频率 |
| `sensor_timeout_sec` | 1.0 | 1.0 | s | 雷达或里程计超过这个时间没更新就停车 |

### 6. 测试

```bash
colcon build --symlink-install && colcon test --packages-select lab02_pkg && colcon test-result --verbose
```

`test_controller.py` 和 `test_lidar_utils.py` 不需要 ROS，直接给纯算法模块喂数字。这也是把算法从节点里拆出来的主要原因。

### 7. `scripts/plot_bag.py`

| 段 | 作用 |
|---|---|
| 1. 读 bag | `rosbag2_py.SequentialReader` 逐条读取消息；`deserialize_message` 把字节还原成 ROS 消息；时间用录制时间戳（秒） |
| `pose_series` | 从 Odometry 消息里取出 `t, x, y, yaw`（平面转动时 yaw = `2·atan2(z, w)`） |
| 2. 轨迹图 | `/odom` 和 `/ground_truth` 的 XY 轨迹，标出起点终点，两轴等比例 |
| 3. 速度图 | `/cmd_vel` 的线速度、角速度随时间变化 |
| 4. 里程计误差 | 用 `np.interp` 把 ground truth 插值到里程计的时间点，误差 = 两点的欧氏距离；bag 里没有 `/ground_truth`（真车）时跳过 |

所有图都保存为 **PDF**（矢量图，放报告）和 **PNG**（预览），坐标轴带国际单位和图例。

### 8. 答辩准备问题（R01）

1. 为什么回调函数传进去时不带括号？
2. 为什么小车停下的位置比 `obstacle_threshold` 更靠近墙？
3. 为什么仿真里目标角度要取整到 90° 的倍数，真车上却不取整？
4. `atan2(sin(e), cos(e))` 做了什么？为什么需要它？
5. 为什么要用最小转速？死区测试能证明什么、不能证明什么？
6. 为什么前方用最近点，真车的左右却用中位数？仿真里为什么不用？
7. 为什么 `inf` 当作畅通，`NaN` 却丢弃？
8. 为什么里程计和真实位置会越差越多？为什么真车上误差更大？
9. 为什么要把算法单独放在一个不依赖 ROS 的模块里？
10. 车架空时永远转不完一次弯，为什么？（提示：TurtleBot3 驱动里 `use_imu: true`）

---

## R02 — EKF 定位（`lab04_pkg`）

### 1. 文件结构

```
src/lab04_pkg/
├── lab04_pkg/
│   ├── probabilistic_models.py   # 速度运动模型、地标观测模型及雅可比（Task 0/1/2，纯 Python）
│   ├── ekf.py                    # RobotEKF 预测与 Joseph 形式更新（纯 Python）
│   └── ekf_node.py               # ROS 2 节点：20 Hz 预测、地标/里程计/IMU 更新、发布 /ekf
├── config/
│   ├── sim.yaml                  # Gazebo Lab04 参数
│   ├── real_robot.yaml           # 真车参数
│   └── landmarks_thu1211.yaml    # 真车场地标签坐标示例
├── launch/
│   └── ekf.launch.py             # 参数：robot (sim|real)、fusion (true|false)、use_sim_time
├── scripts/
│   ├── task0_models.py           # Task 0 采样画图 + sympy 雅可比校验（无需 ROS）
│   ├── evaluate_ekf.py           # 从 bag 出轨迹图与相对 /ground_truth 的 RMSE/MAE
│   ├── replay_filters.py         # 用同一个 bag 离线对比 Task 1 与 Task 2
│   ├── run_sim_experiment.sh     # 自动跑 150 s 仿真 + Bump&Go + 录包
│   └── robot_task3.sh            # 真车一键检查话题 + 编译 + 启动 EKF + 录包
├── test/
│   ├── test_models.py            # 数值雅可比核对、角度环绕、滤波器收敛性测试
│   └── test_{copyright,flake8,pep257}.py
├── LICENSE, package.xml, setup.py, setup.cfg, resource/
```

数据流：

```
/odom (v, ω) ──► 20 Hz 定时器：predict(g, G, V, M) [fusion:=true 时再融合 /odom 与 /imu] ─► /ekf
/landmarks（或 /camera/landmarks）──► 逐标签 update(h, H, Q_landmark, angle_idx=1) ────────┘
```

### 2. 核心算法（`probabilistic_models.py`、`ekf.py`、`ekf_node.py`）

- **速度运动模型（`motion_g`、`motion_G`、`motion_V`）**：
  - $|\omega| \ge 10^{-6}\text{ rad/s}$ 时走圆弧（半径 $r = v/\omega$）：$x' = x - r\sin\theta + r\sin(\theta + \omega\Delta t)$，$y' = y + r\cos\theta - r\cos(\theta + \omega\Delta t)$，$\theta' = \theta + \omega\Delta t$。
  - $|\omega| < 10^{-6}\text{ rad/s}$（`SMALL_W`）时切到精确直线极限分支，避免除零，并配对给出状态雅可比 $G$ 和控制雅可比 $V$。
- **地标观测模型（`landmark_h`、`landmark_H`）**：
  - 已知标签坐标 $(m_x, m_y)$，预测距离与方位 $z = [\sqrt{\Delta x^2 + \Delta y^2},\; \text{wrap}(\text{atan2}(\Delta y, \Delta x) - \theta)]^T$。
  - `landmark_H(mu, landmark, dim=3|5)` 输出 $2\times 3$ 或 $2\times 5$ 雅可比（速度列全为 0）。
- **Task 2 传感器融合（`fusion:=true`，状态 $[x, y, \theta, v, \omega]^T$）**：
  - `extended_g` 用状态里的 $(v, \omega)$ 按常速度模型推演位姿；`extended_G` 左上 $3\times 3$ 为 `motion_G`、右上 $3\times 2$ 为 `motion_V`；`extended_V` 把过程噪声加到速度分量上。
  - 每次 20 Hz 预测后，`fuse_velocities()` 用新到的 `/odom`（观测 $[v, \omega]$）和 `/imu`（观测 $[\omega]$）各做一次线性更新。
- **滤波器更新（`RobotEKF`）**：
  - 方位残差按 `angle_idx=1` 用 `atan2(sin, cos)` 折叠到 $[-\pi, \pi]$，状态航向 $\theta$（索引 `2`）每次预测和更新后也做折叠。
  - 协方差更新采用 **Joseph 形式** $\Sigma = (I-KH)\Sigma(I-KH)^T + KQK^T$，保证数值对称与正半定。
- **ROS 2 节点（`ekf_node.py`）**：
  - `dt` 取自 ROS 时钟，`dt <= 0` 时跳过（仿真暂停时不更新）；`landmark_callback` 对一帧内的每个已知标签依次更新，未知 ID 报一次 warning；发布 `/ekf` 时直接按平面四元数写出（`z = sin(θ/2)`，`w = cos(θ/2)`，不依赖 `tf_transformations`），并把 $3\times 3$ 协方差填入 $6\times 6$ 矩阵的 `(0, 1, 5)` 位置。

### 3. ⚙️ 参数表（`sim.yaml` vs `real_robot.yaml`）

| 参数 | `sim.yaml` | `real_robot.yaml` | 单位 | 含义 |
|---|---|---|---|---|
| `use_sim_time` | `true` | `false` | – | 回放真车 bag（`--clock`）时传 `use_sim_time:=true` |
| `landmarks_topic` | `/landmarks` | `/camera/landmarks` | – | 仿真地标 / 真车 AprilTag 话题 |
| `landmarks_file` |（`turtlebot3_perception` 默认）| `landmarks_thu1211.yaml` | – | 纯文件名在 `lab04_pkg/config/` 查找，也支持绝对路径 |
| `init_pose` | `[-2.0, -0.5, 0.0]` | `[0.0, 0.0, 0.0]` | m, m, rad | map 坐标系下的初始位姿（真车每轮现量） |
| `init_std` | `[0.1, 0.1, 0.1]` | `[0.3, 0.3, 0.3]` | m, m, rad | 初始位姿标准差（真车不确定度更大） |
| `prediction_rate` | `20.0` | `20.0` | Hz | 固定预测频率 |
| `std_v` / `std_w` | `0.05` / `0.1` | `0.05` / `0.1` | m/s, rad/s | 线速度、角速度过程噪声 |
| `std_range` / `std_bearing_deg` | `0.05` / `2.0` | `0.1` / `3.0` | m, deg | 地标测距与测角噪声（大于仿真标称 0.01 m / 1°） |
| `std_odom_v` / `std_odom_w` | `0.01` / `0.02` | `0.02` / `0.05` | m/s, rad/s | Task 2 `/odom` 速度观测噪声 |
| `std_imu_w` | `0.02` | `0.02` | rad/s | Task 2 `/imu` 角速度观测噪声 |

### 4. 测试与校验

```bash
colcon test --packages-select lab04_pkg && colcon test-result --verbose
```

- `test_models.py`：用中心数值差分核对 `motion_G`、`motion_V`、`landmark_H`、`extended_G`（含 $\omega = 0$）；验证直线与小转弯连续性、四分之一圆运动学、`sample_pose_from_landmark` 逆向一致、方位残差跨 $\pm\pi$ 环绕、以及 400 步 EKF 收敛与协方差对称。
- `test_copyright.py`、`test_flake8.py`、`test_pep257.py`：GPL-3.0 版权头与 ROS 2 代码规范检查。

### 5. 口试准备问题（R02，附简短答案）

1. **为什么速度运动模型要对 $|\omega| < 10^{-6}$ 单独写直线分支？**
   圆弧公式含 $r = v/\omega$，角速度趋近 0 时会除零发散；直线分支是它的精确极限，和小转弯结果连续衔接。
2. **为什么协方差更新用 Joseph 形式 $\Sigma = (I-KH)\Sigma(I-KH)^T + KQK^T$，而不是 $(I-KH)\Sigma$？**
   $(I-KH)\Sigma$ 在浮点减法下可能破坏对称性甚至出现负特征值；Joseph 形式是两个对称半正定矩阵之和，数值更稳。
3. **为什么方位残差 $y_\phi = z_\phi - \hat{z}_\phi$ 必须折叠到 $[-\pi, \pi]$？**
   若预测为 $+\pi - 0.01$、观测为 $-\pi + 0.01$，直接相减得 $-2\pi + 0.02$ 而非 $+0.02$，会把估计朝向错误拉转近一圈。
4. **Task 1（`fusion:=false`）和 Task 2（`fusion:=true`）的区别是什么？**
   Task 1 状态只有 3 维 $[x, y, \theta]$，直接把 `/odom` 速度当作控制输入；Task 2 扩到 5 维 $[x, y, \theta, v, \omega]$，用常速度预测，再把 `/odom`（$v, \omega$）和 `/imu`（$\omega$）当作观测量融合更新。
5. **仿真里用同一段 bag 对比，Task 2 让航向误差降了约 20–30 %，位置 RMSE 却仍是 0.010 m，为什么？**
   IMU 只观测量角速度 $\omega$，能在两次看到标签之间把航向稳住；而位置精度已经主要由高频、准确的标签距离/方位观测决定。
6. **为什么 `sim.yaml` 里的 `std_range`（0.05 m）和 `std_bearing_deg`（2.0°）比仿真器标称噪声（0.01 m、1°）大？**
   $Q$ 设得过小会让滤波器过度自信，放大线性化误差和时间戳延迟带来的抖动；适度放大 $Q$ 能让协方差更真实、估计更平滑。
7. **为什么真车起步要让相机至少同时看到两个 AprilTag？**
   初始朝向不准时，只看 1 个标签的距离和方位在标签周围的圆上存在多解/双解；看到 2 个以上标签就能立刻消除几何歧义。
8. **真车跑完 172 s 后 `/ekf` 和 `/odom` 相差 0.28 m，能说明 EKF 误差就是 0.28 m 吗？**
   不能。真车没有动捕真值，0.28 m 只是 EKF 和会漂移的轮式里程计之间的差值，只说明估计与标签自洽，不代表绝对精度。

---

## R03 — DWA 避障与目标跟随（`lab07_pkg`）

### 1. 文件结构

```
src/lab07_pkg/
├── lab07_pkg/
│   ├── scan_utils.py         # 三点中值滤波、清洗、健康检查、24 扇区下采样、独立急停
│   ├── dwa.py                # 动态窗口、轨迹推演、Task 1/2 打分、余量带锚定、卡住恢复
│   ├── navigation.py         # 目标/跟随状态机、轨迹点、卡住恢复、Lost、碰撞锁定与复位
│   └── dwa_node.py           # ROS 2 节点：15 Hz 定时、传感器 QoS、/reset_nav、退出零速
├── config/
│   ├── sim.yaml              # Gazebo hard_maze.world 参数
│   └── real_robot.yaml       # 真车 TurtleBot3 Burger 参数
├── launch/
│   └── dwa.launch.py         # 参数：robot、mode、goal_source、target_id、use_sim_time
├── scripts/
│   ├── evaluate_nav.py       # rosbag2 指标与轨迹/速度图（支持 sim 与 real）
│   ├── run_goal_trials.sh    # 批量重复 Gazebo Task 1 实验（进程组清理 + CSV）
│   ├── run_single_trial.py   # 先订阅确认再发目标，按仿真时钟记用时
│   └── offline_sim.py        # 纯 Python 2D 快速辅助仿真（不替代 Gazebo）
├── test/
│   ├── test_{scan_utils,dwa,navigation,follow_mode,evaluate_nav}.py
│   ├── test_{review_must_fix,review_should_fix,review2_fixes,review4_fixes}.py
│   ├── test_{task5_review3,task7_standoff,task8_follow_recovery,task9_margin_and_lost,task10_median_filter}.py
│   └── test_{copyright,flake8,pep257}.py
├── LICENSE, package.xml, setup.py, setup.cfg, resource/
```

15 Hz 控制数据流：

```
/scan ─► scan_utils：median3_filter ─► 健康检查 ─► 独立急停（车中心距 < r + tol）
                                                └─► 24 扇区障碍点（按扫描时刻 odom 投影）
/odom ─► odom_history + (v, ω) ──► navigation.py（目标 / 轨迹点 / 卡住恢复 / Lost / 锁定）
/goal_pose | /dynamic_goal_pose | /camera/landmarks ──┘
                                                       ▼
                     dwa.py：动态窗口 ─► 210 条推演 ─► 先过滤再打分 ─► /cmd_vel
```

### 2. 核心算法

- **`scan_utils.py`**：
  - `median3_filter(ranges)` 在清洗前做 360° 首尾相邻的三点中值滤波（开关 `scan_median_filter: true`）：非有限值（`NaN`、`±inf`）不参加中值并保持原值；两邻都无效保原值；墙边单侧有限邻取上中值（`[0.435, 0.293, inf] -> 0.435`）。代价：只占 1 条射线宽的物体（0.3 m 处约 8 mm、1 m 处约 2.7 cm）会被滤掉，场地箱子、墙和人腿不受影响。
  - `clean_scan_ranges` 把 `+inf` 映射为 `eff_max`，按 `nan_policy`（仿真 `'min'`、真车 `'max'`）处理 `NaN`，丢弃 `< min_valid_range`（`0.05 m`，滤掉 LDS-02 的 `0.0` 异常值），截断超量程值。
  - `check_scan_health` 在有效射线 `< min_valid_ratio`（`20%`）或 `NaN` 占比 `> max_nan_ratio`（`95%`）时停车报警，防止 `nan_policy='max'` 下全 `NaN` 盲开。
  - `downsample_scan` 将 360° 分为 `num_ranges = 24` 个扇区，取每扇区最小距离并**保留真实射线角度**，再按 `laser_offset_x = -0.032 m` 换算到车中心与 odom 坐标系；`check_safety_stop` 独立从车中心判断急停。
- **`dwa.py`**（改写自 `SESASR-Course/planning_control_methods`，GPL-3.0）：
  - **动态窗口**：将速度上下限与单步加速度窗口求交，采样 `v_samples = 10` × `w_samples = 21`（奇数强制含 $\omega = 0$）。真车用 `window_source = 'last_cmd'` 避免起步静摩擦卡死。
  - **轨迹推演**：按 `time_granularity = 0.1 s` 向前推演 `sim_time = 1.5 s`（与控制周期 `dt = 1/15 s` 解耦）；`max_ang_vel = 1.5 rad/s` 使 $\omega \cdot 1.5 = 2.25 < \pi$，防止推演航向绕过 180°。
  - **双层安全与余量带锚定**：
    - 规划碰撞半径为 `collision_radius + planning_margin`（仿真 `0.385 m`，真车 `0.405 m`）；**先剔除碰撞轨迹再归一化打分**。
    - 在余量带 `[collision_radius + band_floor_buffer, planning_collision_radius)`（`band_floor_buffer = 0.03 m`）内，进入时锁定 `band_entry_min`（离开或复位时清零），只允许 `future_min >= max(band_entry_min, band_floor_radius) - 1e-6` 的轨迹，杜绝 15 Hz 步进累加向急停线蠕动。
  - **目标函数**：
    - Task 1（`goal`）：$J = \alpha\cdot\text{heading} + \beta\cdot\text{vel} + \gamma\cdot\text{dist}_{\text{obst}}$。
    - Task 2（`follow`）：$J = \alpha\cdot\text{heading} + \beta\cdot\text{vel}' + \gamma\cdot\text{dist}_{\text{obst}} + \delta\cdot\text{dist}_{\text{target}}$，其中 `vel'` 在距离 `< 1.2 m` 时线性减速，`dist_target` 结合围绕 `0.8 m` 的高斯距离项与匹配 ±33° 视场的高斯方位项。
    - 安全轨迹净距极差 `< obs_norm_min_span`（`0.05 m`）时改用线性 `clearance / obstacle_max_dist`，不在窄走廊放大毫米级噪声。
- **`navigation.py` 与 `dwa_node.py`**：
  - 用 `odom_history` 按消息头时间戳匹配最近位姿投影激光与标签（带 `max_stamp_skew_sec = 0.5 s` 保护）。
  - `goal_source = 'landmark'` 在静态目标模式下锁定标签并沿「车→标签」回退 `landmark_goal_standoff = 0.5 m`。
  - 跟随轨迹点与卡住恢复：每隔 `trail_spacing = 0.10 m` 记录目标路径点（跳变 `> 2.5 m` 清空，走近 `<= 0.40 m` 弹出），`heading` 瞄准最早未到达点而 `vel'`/`dist_target` 仍看目标当前位置；连续 `2.0 s` 满足 `v < 0.03 m/s` 且离目标较远时进入进展恢复（只在安全轨迹里选，最长 `5.0 s` 或走出 `0.3 m` 退出，原地转向用 `0.003 / 0.08 / 0.25 m` 前瞻选向）。
  - `Lost` 机制：连续 `10.0 s` 距离 `> 3.0 m` 仅向 `/nav_result` 发一次 `Lost`、不停车继续追，回到 `<= 3.0 m` 后重新布防。
  - 碰撞锁定与 `/reset_nav`：`Collision` 后锁定直至调用 `std_srvs/Trigger` 服务 `/reset_nav`；复位后允许受保护脱困（物理接触和净距恶化仍会急停）。

### 3. ⚙️ 关键参数表（`sim.yaml` vs `real_robot.yaml`）

| 参数 | `sim.yaml` | `real_robot.yaml` | 单位 | 含义 |
|---|---|---|---|---|
| `control_rate` / `sim_time` / `time_granularity` | `15` / `1.5` / `0.1` | `15` / `1.5` / `0.1` | Hz, s, s | 控制频率、推演时长与步长 |
| `v_samples` / `w_samples` | `10` / `21` | `10` / `21` | – | 速度采样（21 保证含 $\omega = 0$） |
| `max_lin_vel` / `max_ang_vel` | `0.22` / `1.5` | **`0.15`** / `1.5` | m/s, rad/s | 速度上限（真车实测 0.15） |
| `max_lin_acc` / `max_ang_acc` | `0.5` / `3.2` | `0.4` / `2.0` | m/s², rad/s² | 动态窗口加速度上限 |
| `window_source` | `'odom'` | **`'last_cmd'`** | – | 动态窗口基准速度来源 |
| `robot_radius` / `collision_tol` | `0.105` / `0.18` | `0.105` / `0.20` | m | 急停半径（`0.285` vs `0.305 m`） |
| `planning_margin` / `band_floor_buffer` | `0.10` / `0.03` | `0.10` / `0.03` | m | 规划额外余量与余量带下限缓冲 |
| `weight_heading` / `vel` / `obs` / `target` | `0.40` / `0.30` / `0.30` / `0.35` | 同左 | – | 目标函数权重 $\alpha, \beta, \gamma, \delta$ |
| `obs_norm_min_span` / `obstacle_max_dist` | `0.05` / `1.0` | `0.05` / `1.0` | m | 防噪归一化阈值与障碍距离截断 |
| `nan_policy` / `scan_median_filter` | `'min'` / `true` | **`'max'`** / `true` | – | NaN 策略与三点中值滤波开关 |
| `min_valid_ratio` / `max_nan_ratio` | `0.20` / `0.95` | `0.20` / `0.95` | – | 激光健康检查阈值 |
| `follow_source` / `require_target_id` | `'dynamic_goal'` / `false` | `'landmarks'` / **`true`** | – | 跟随源；真车强制指定 `target_id` |
| `goal_source` / `landmark_goal_standoff` | `'topic'` / `0.5` | `'topic'` / `0.5` | –, m | 静态目标来源与标签退后距离 |
| `desired_target_dist` / `slowdown_dist_threshold` | `0.8` / `1.2` | `0.8` / `1.2` | m | 期望跟随距离与 `vel'` 减速距离 |
| `follow_use_trail` / `stuck_time_sec` | `true` / `2.0` | `true` / `2.0` | –, s | 轨迹点瞄准与卡住恢复触发时间 |
| `lost_dist` / `lost_dist_time_sec` | `3.0` / `10.0` | `3.0` / `10.0` | m, s | 只报告不停车的 `Lost` 阈值 |

### 4. 测试与校验

```bash
colcon test --packages-select lab07_pkg && colcon test-result --verbose
```

ROS 2 下 `pytest` 共 **78 项测试全部通过**：包含 14 个测试文件里的 75 项纯 Python 单元与回归测试（覆盖激光清洗/中值滤波/健康检查、DWA 动态窗口/打分/余量带防蠕动、Task 1 与 Task 2 状态机、轨迹点与走廊卡住恢复、标签退后距离、非停车 `Lost`、碰撞锁定与 `/reset_nav`、以及 `evaluate_nav.py` 指标与绘图），以及 `test_copyright.py`、`test_flake8.py`、`test_pep257.py`（均为 0 警告）。

### 5. 口试准备问题（R03，附简短答案）

1. **为什么把控制周期 `dt = 1/15 s` 和推演步长 `time_granularity = 0.1 s` 解耦，且把 `max_ang_vel` 限制在 `1.5 rad/s`？**
   `dt` 决定动态窗口一拍内能加减速的范围，`time_granularity` 决定 1.5 s 轨迹推演的积分步数（15 步，计算快）；$\omega \le 1.5\text{ rad/s}$ 保证 $1.5 \times 1.5 = 2.25 < \pi$，推演末端朝向不会绕过 180° 引起航向打分跳变。
2. **为什么必须在归一化之前先过滤掉碰撞轨迹？**
   不先剔除的话，碰撞轨迹会拉大 min-max 极差、扭曲安全轨迹之间的权重比例，甚至可能让一条高速碰撞轨迹得分高于低速安全轨迹。
3. **为什么要把规划碰撞半径比急停半径多留 `0.10 m`，且在余量带内锚定 `band_entry_min` 并加 `0.03 m` 下限缓冲？**
   如果两者相同，刹车惯性和激光扇区离散误差很容易把车送进急停线；入口锚定且去掉向下容差、并把余量带下限抬到急停线上方 3 cm，彻底杜绝 15 Hz 步进蠕动误撞。
4. **为什么 `hard_maze.world` 里中心墙后的 4 个点会以 `Timeout` 结束？**
   经典 DWA 是纯局部规划器，`heading` 项直接指向墙后目标，车会在墙前陷入局部极小并安全停住；解决这类场景需要全局规划器（如 A*、NF1）提供中间路径点。
5. **Task 2 为什么把 `vel` 换成 `vel'`、新增 `dist_target`，还要加轨迹点瞄准和卡住恢复？**
   原始 `vel` 会让车冲过慢速目标；`vel'` 在 1.2 m 内平滑减速，`dist_target` 维持约 0.8 m 并把目标留在 ±33° 视场中央；瞄准轨迹点帮车绕过拐角，进展恢复在不放松碰撞检查的前提下脱出走廊局部极小。
6. **`median3_filter` 解决了什么问题？代价是什么？**
   它滤除了墙中或墙边（邻接 `inf`）的单条激光射线毛刺，消除假 `Collision` 急停；代价是只占 1 条射线宽的物体（0.3 m 处约 8 mm、1 m 处约 2.7 cm）会被滤掉，但远小于场地里的箱子、墙和人腿。
7. **真车 LDS-02 为什么设 `nan_policy: 'max'`，又为什么要配 `max_nan_ratio: 0.95`？**
   真车激光打空返回 `NaN`，若按 `'min'` 会立刻假急停；但在 `'max'` 下，传感器故障全发 `NaN` 和空旷场地无法区分，所以靠 `max_nan_ratio` 兜底停车。
8. **为什么 `real_robot.yaml` 用 `window_source: 'last_cmd'` 而不是 `'odom'`？**
   15 Hz 下从静止起步第一拍指令只有约 0.027 m/s，真车静摩擦可能推不动；若窗口按 `'odom'` 算就会一直锁在 0 附近，用 `'last_cmd'` 能逐拍爬升直到克服静摩擦。
9. **真车为什么要加 `landmark_goal_standoff = 0.5 m` 和 `require_target_id = true`？**
   AprilTag 常贴在墙或支架上，直接开到标签坐标会在到达 `0.25 m` 容差前先触发 `0.305 m` 急停，退后 0.5 m 才能正常报 `Goal`；强制 `target_id` 防止场地里的固定定位标签抢走跟随目标。
10. **为什么 `Collision` 会锁定直到调 `/reset_nav`，而 `Lost` 只发通知不停车？**
    近障急停后如果障碍或目标稍动就自动重跑很危险，必须人工确认后通过 `/reset_nav` 解锁；而跟随时在弯道暂时落后超过 3 m，停车反而永远追不回来，所以只发通知继续追。
