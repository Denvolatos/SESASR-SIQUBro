# 📖 Code Guide / 代码介绍指南

**Language / 语言：** [🇬🇧 English](#-english) · [🇨🇳 中文](#-中文) · [⬅️ README](../README.md)

---

## 🇬🇧 English

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
                      left  = sector_median(+90°±30°), right = sector_median(−90°±30°)
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
- `sector_median` (sides): the **median**, to judge how open a side is. With the minimum, one close corner hides a long free corridor and the robot turns the wrong way.
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

`wrap_angle` makes the robot turn 2° (not 358°) from −179° to +179°. The minimum speed is needed on the real robot: near the target the P controller asks for less than the motors can do, and the robot stays stuck in ROTATE.

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
| `min_angular_vel` | 0.3 | 0.3 | rad/s | Motor dead zone |
| `turn_angle_deg` | 90 | **45** | deg | Largest turn per bump; in a corner the robot bumps again and turns more |
| `snap_to_axes` | true | **false** | – | Round the turn to the maze axes |
| `turn_until_clear` | false | **true** | – | Stop turning as soon as the front is free |
| `clear_margin` | 0.3 | 0.3 | m | Front is free above `obstacle_threshold + clear_margin` |
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

### 8. Questions to prepare for the oral exam

1. Why are the callbacks passed without parentheses?
2. Why does the robot stop closer to the wall than `obstacle_threshold`?
3. Why is the target yaw rounded to multiples of 90° in simulation but not on the real robot?
4. What does `atan2(sin(e), cos(e))` do, and why is it needed?
5. Why is a minimum angular velocity needed on the real robot?
6. Why the minimum for the front sector but the median for the sides?
7. Why are `inf` treated as free space and `NaN` ignored?
8. Why do odometry and ground truth diverge over time? Why is the error larger on the real robot?
9. Why is the algorithm in a separate module without ROS?
10. Lifted on a stand, the robot never finishes a turn. Why? (Hint: `use_imu: true` in the TurtleBot3 driver.)

---

## 🇨🇳 中文

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
                      left  = sector_median(+90°±30°)，right = sector_median(−90°±30°)
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
- `diff` 处理"绕一圈"：355° 和 0° 只差 5°，不是 355°。
- 角度方向：0° 正前方，+90° 左边，−90° 右边（逆时针为正）。
- 正无穷 `inf` = 量程内没打到东西 → 当作 `range_max`（畅通）。
- `NaN`、负无穷、小于 5 cm 和大于 `range_max` 的读数都丢弃。真车 LDS-02 打空时返回的是 **NaN**，而且报的 `range_min = 0.0`；不设 5 cm 下限的话，读数 0.0 会被当成 0 米处有墙。
- `sector_min`（前方）：取**最近点**，为了安全。
- `sector_median`（左右）：取**中位数**，判断哪边更空。用最近点的话，一个近处的墙角就会把后面一整条长通道"挡住"，车会转错方向。
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

`wrap_angle` 让车从 −179° 到 +179° 只转 2°，而不是 358°。最小转速是真车需要的：快转到位时，P 控制给出的转速低于电机能动的最小速度，车会一直卡在 ROTATE。

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
| `min_angular_vel` | 0.3 | 0.3 | rad/s | 电机死区 |
| `turn_angle_deg` | 90 | **45** | deg | 每次最多转多少；墙角里会再触发一次接着转 |
| `snap_to_axes` | true | **false** | – | 转向角对齐迷宫坐标轴 |
| `turn_until_clear` | false | **true** | – | 前方一空出来就停止转向 |
| `clear_margin` | 0.3 | 0.3 | m | 前方 > `obstacle_threshold + clear_margin` 算空 |
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

### 8. 答辩准备问题

1. 为什么回调函数传进去时不带括号？
2. 为什么小车停下的位置比 `obstacle_threshold` 更靠近墙？
3. 为什么仿真里目标角度要取整到 90° 的倍数，真车上却不取整？
4. `atan2(sin(e), cos(e))` 做了什么？为什么需要它？
5. 为什么真车需要最小转速？
6. 为什么前方用最近点，左右用中位数？
7. 为什么 `inf` 当作畅通，`NaN` 却丢弃？
8. 为什么里程计和真实位置会越差越多？为什么真车上误差更大？
9. 为什么要把算法单独放在一个不依赖 ROS 的模块里？
10. 车架空时永远转不完一次弯，为什么？（提示：TurtleBot3 驱动里 `use_imu: true`）
