# 📖 Code Guide / 代码介绍指南

**Language / 语言：** [🇬🇧 English](#-english) · [🇨🇳 中文](#-中文) · [⬅️ README](../README.md)

---

## 🇬🇧 English

### 1. Package layout

```
lab02_pkg/
├── package.xml          # package metadata and dependencies (rclpy, geometry_msgs, sensor_msgs, nav_msgs)
├── setup.py             # installation; registers the executable "bump_and_go"
├── setup.cfg            # needed by ros2 run (do not modify)
├── resource/lab02_pkg   # marker file (do not modify)
└── lab02_pkg/
    ├── __init__.py      # makes the folder a Python module
    └── bump_and_go.py   # the controller node
```

The executable is registered in `setup.py`:

```python
'console_scripts': [
    'bump_and_go = lab02_pkg.bump_and_go:main',
],
```

Read it as: *the command `bump_and_go` runs the function `main` in the file `lab02_pkg/bump_and_go.py`*.

### 2. `bump_and_go.py`, section by section

The file is split into five sections, marked with `# ---------- N. ... ----------`.

#### Section 1 — Setup (`__init__`)

Runs once when the node starts.

| Code | Purpose |
|---|---|
| `super().__init__('bump_and_go')` | Creates the ROS 2 node; the string is the node name shown by `ros2 node list` |
| `declare_parameter(...)` / `get_parameter(...).value` | Defines the tunable values (speeds, frequency, threshold, goal) and reads them |
| `create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)` | Receives the LiDAR; sensor QoS is required on the real robot |
| `create_subscription(Odometry, '/odom', self.odom_callback, 10)` | Receives the odometry (position + orientation) |
| `create_publisher(Twist, '/cmd_vel', 10)` | Sends velocity commands to the robot |
| `create_timer(period, self.control_loop)` | Calls `control_loop` every `1 / control_frequency` seconds |
| `self.state = 'GO'` | Initial state of the state machine |

Callbacks are passed **without parentheses** (`self.scan_callback`, not `self.scan_callback()`): we hand the function to ROS, which calls it when a message arrives.

#### Section 2 — Sensor callbacks

They only **store** data; decisions are taken in the control loop.

```python
def odom_callback(self, msg):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    _, _, self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
```

The orientation arrives as a **quaternion**; `euler_from_quaternion` returns `(roll, pitch, yaw)`. A ground robot only needs the yaw (rotation around z).

#### Section 3 — `sector_distance(center_deg, half_width_deg)`

Returns the closest obstacle inside an angular sector.

```python
angle = math.degrees(s.angle_min + i * s.angle_increment)   # angle of ray i
diff = (angle - center_deg + 180.0) % 360.0 - 180.0        # difference in [-180, 180)
```

- The angle of ray `i` is computed from the message fields, so the code works with any number of rays (230 in simulation, a different number on the real sensor).
- `diff` handles the wrap-around: a ray at 355° is 5° away from 0°, not 355°.
- `inf` or values above `range_max` → treated as `range_max` (free). Values below `range_min` → ignored (invalid).
- Angles: 0° = front, +90° = left, −90° (= 270°) = right (counter-clockwise positive).

#### Section 4 — `control_loop` (state machine)

```
GO      front > threshold  → drive forward at max_linear_vel
        front ≤ threshold  → choose side, set target_yaw, go to ROTATE
ROTATE  |error| ≥ 2°       → ω = clip(2·error, ±max_angular_vel)
        |error| < 2°       → back to GO
DONE    (simulation only) near the goal → velocity 0
```

Key lines:

```python
turn = math.pi / 2 if left > right else -math.pi / 2
self.target_yaw = round((self.yaw + turn) / (math.pi / 2)) * (math.pi / 2)
```

The target is snapped to 0°, 90°, 180° or 270° because the maze walls are axis-aligned.

```python
error = math.atan2(math.sin(self.target_yaw - self.yaw),
                   math.cos(self.target_yaw - self.yaw))
```

`atan2(sin, cos)` wraps the error to [−π, π]: from −179° to +179° the robot turns 2°, not 358°.

A new `Twist()` has all velocities equal to 0, so in `DONE` (or when nothing is set) the published command stops the robot.

### 2b. ⚙️ Parameters

Change them without editing the code: `--ros-args -p name:=value`

| Parameter | Default | Unit | Meaning |
|---|---|---|---|
| `max_linear_vel` | `0.22` | m/s | Forward speed (TurtleBot3 Burger limit: 0.22) |
| `max_angular_vel` | `1.5` | rad/s | Maximum turning speed (robot limit 2.84, 1.5 suggested) |
| `control_frequency` | `10.0` | Hz | Frequency of the control loop |
| `obstacle_threshold` | `0.5` | m | Distance in front that triggers a turn |
| `stop_at_goal` | `true` | – | Stop near the goal; **set `false` on the real robot** |
| `goal_x`, `goal_y` | `6.5`, `3.0` | m | Position of the green cube in the Lab02 maze |
| `goal_tolerance` | `0.8` | m | Distance from the goal at which the robot stops |
| `rotation_tolerance_deg` | `5.0` | deg | A turn is finished when the yaw error is below this |
| `min_angular_vel` | `0.3` | rad/s | Minimum turning speed: the real motors do not move below it (dead zone) |

Example: `ros2 run lab02_pkg bump_and_go --ros-args -p obstacle_threshold:=0.6`

#### Section 5 — Diagnostics and safe stop

- `status_log` runs every 2 s and prints what the controller sees and sends:
  `state=GO front=1.85 m  v=0.22 m/s  w=0.00 rad/s  odom=OK`. Before the first LiDAR message it prints `waiting for /scan ...`. Use it to tell apart "the controller does not send forward" from "the controller sends forward but the robot does not move".
- `stop_robot` publishes a zero `Twist` three times. `main` calls it when you press `Ctrl+C`, so the robot does not keep driving with the last command. For this, `rclpy.init(..., signal_handler_options=SignalHandlerOptions.NO)` lets our code handle `Ctrl+C` while ROS is still running.

### 3. `scripts/plot_bag.py`

| Section | What it does |
|---|---|
| 1. Read the bag | `rosbag2_py.SequentialReader` reads every message; `deserialize_message` turns bytes into ROS messages; time is the recording time stamp in seconds |
| `pose_series` | Extracts `t, x, y, yaw` from Odometry messages (yaw from the quaternion: `2·atan2(z, w)` for a planar rotation) |
| 2. Trajectory | XY plot of `/odom` and `/ground_truth`, with start and end markers, equal axis scale |
| 3. Velocity commands | `/cmd_vel` linear and angular velocity vs time |
| 4. Odometry error | Ground truth is interpolated (`np.interp`) at the odometry time stamps; the error is the Euclidean distance. Skipped if `/ground_truth` is not in the bag (real robot) |

All figures are saved as **PDF** (vector, for the report) and **PNG** (preview), with axis labels in SI units and legends.

### 4. Questions to prepare for the oral exam

1. Why are the callbacks passed without parentheses?
2. Why does the robot stop closer to the wall than `obstacle_threshold`?
3. Why is the target yaw rounded to multiples of 90°?
4. What does `atan2(sin(e), cos(e))` do, and why is it needed?
5. Why does a constant angular velocity make the robot oscillate around the target?
6. Why is `inf` treated as free space?
7. Why do odometry and ground truth diverge over time? Why is the error larger on the real robot?
8. What changes between simulation and the real robot, and why (QoS, no ground truth, ray indices)?

---

## 🇨🇳 中文

### 1. 包的结构

```
lab02_pkg/
├── package.xml          # 包的信息和依赖（rclpy、geometry_msgs、sensor_msgs、nav_msgs）
├── setup.py             # 安装说明，登记可执行程序 "bump_and_go"
├── setup.cfg            # ros2 run 需要（不要改）
├── resource/lab02_pkg   # 标记文件（不要改）
└── lab02_pkg/
    ├── __init__.py      # 让这个文件夹成为 Python 模块
    └── bump_and_go.py   # 控制器节点
```

可执行程序在 `setup.py` 里登记：

```python
'console_scripts': [
    'bump_and_go = lab02_pkg.bump_and_go:main',
],
```

意思是：*命令 `bump_and_go` 运行 `lab02_pkg/bump_and_go.py` 文件里的 `main` 函数*。

### 2. `bump_and_go.py` 逐段讲解

文件分 5 段，每段开头有 `# ---------- N. ... ----------` 标记。

#### 第 1 段：初始化（`__init__`）

节点启动时只运行一次。

| 代码 | 作用 |
|---|---|
| `super().__init__('bump_and_go')` | 创建 ROS 2 节点；字符串是节点名，`ros2 node list` 显示的就是它 |
| `declare_parameter(...)` / `get_parameter(...).value` | 定义并读取可调参数（速度、频率、阈值、目标） |
| `create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)` | 接收雷达数据；真车上必须用传感器 QoS |
| `create_subscription(Odometry, '/odom', self.odom_callback, 10)` | 接收里程计（位置 + 朝向） |
| `create_publisher(Twist, '/cmd_vel', 10)` | 向机器人发送速度指令 |
| `create_timer(period, self.control_loop)` | 每 `1 / control_frequency` 秒调用一次 `control_loop` |
| `self.state = 'GO'` | 状态机的初始状态 |

回调函数**不带括号**传入（写 `self.scan_callback`，不写 `self.scan_callback()`）：把函数本身交给 ROS，消息到了由 ROS 来调用。

#### 第 2 段：传感器回调

只负责**记下数据**，决策在控制循环里做。

```python
def odom_callback(self, msg):
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    _, _, self.yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
```

朝向以**四元数**给出；`euler_from_quaternion` 返回 `(roll, pitch, yaw)`。地面机器人只需要 yaw（绕 z 轴的转角）。

#### 第 3 段：`sector_distance(中心角, 半宽)`

返回某个扇形区域内最近的障碍物距离。

```python
angle = math.degrees(s.angle_min + i * s.angle_increment)   # 第 i 条光束的角度
diff = (angle - center_deg + 180.0) % 360.0 - 180.0        # 角度差，范围 [-180, 180)
```

- 每条光束的角度都用消息里的字段现算，所以光束有多少条都能用（仿真 230 条，真雷达数量不同）。
- `diff` 处理"绕一圈"的问题：355° 和 0° 只差 5°，不是 355°。
- `inf` 或大于 `range_max` 的值当作 `range_max`（畅通）；小于 `range_min` 的值丢弃（无效）。
- 角度方向：0° 正前方，+90° 左边，−90°（= 270°）右边（逆时针为正）。

#### 第 4 段：`control_loop`（状态机）

```
GO      前方 > 阈值   → 以 max_linear_vel 前进
        前方 ≤ 阈值   → 选择转向方向，设定 target_yaw，进入 ROTATE
ROTATE  |误差| ≥ 2°  → ω = 限幅(2·误差, ±max_angular_vel)
        |误差| < rotation_tolerance_deg  → 回到 GO
DONE    （仅仿真）到达目标附近 → 速度为 0
```

关键代码：

```python
turn = math.pi / 2 if left > right else -math.pi / 2
self.target_yaw = round((self.yaw + turn) / (math.pi / 2)) * (math.pi / 2)
```

目标角度取整到 0°、90°、180°、270°，因为迷宫的墙都和坐标轴平行。

```python
error = math.atan2(math.sin(self.target_yaw - self.yaw),
                   math.cos(self.target_yaw - self.yaw))
```

`atan2(sin, cos)` 把误差限制在 [−π, π]：从 −179° 转到 +179° 只需转 2°，而不是 358°。

新建的 `Twist()` 所有速度都是 0，所以在 `DONE` 状态（或没有设置速度时）发出去的指令就是停车。

### 2b. ⚙️ 参数

不改代码就能调：`--ros-args -p 参数名:=值`

| 参数 | 默认值 | 单位 | 含义 |
|---|---|---|---|
| `max_linear_vel` | `0.22` | m/s | 前进速度（TurtleBot3 Burger 上限 0.22） |
| `max_angular_vel` | `1.5` | rad/s | 最大转速（机器人上限 2.84，建议 1.5） |
| `control_frequency` | `10.0` | Hz | 控制循环频率 |
| `obstacle_threshold` | `0.5` | m | 前方距离小于它就转弯 |
| `stop_at_goal` | `true` | – | 到目标附近停车；**真车上设为 `false`** |
| `goal_x`, `goal_y` | `6.5`, `3.0` | m | Lab02 迷宫里绿块的位置 |
| `goal_tolerance` | `0.8` | m | 离目标多近就停车 |
| `rotation_tolerance_deg` | `5.0` | deg | 偏航角误差小于它就算转完 |
| `min_angular_vel` | `0.3` | rad/s | 最小转速：真车电机低于这个速度转不动（死区） |

例子：`ros2 run lab02_pkg bump_and_go --ros-args -p obstacle_threshold:=0.6`

#### 第 5 段：诊断信息和安全停车

- `status_log` 每 2 秒打印一次控制器看到了什么、发出了什么：
  `state=GO front=1.85 m  v=0.22 m/s  w=0.00 rad/s  odom=OK`。还没收到雷达数据时打印 `waiting for /scan ...`。用它可以分清"控制器没发前进指令"和"发了前进指令但车没动"。
- `stop_robot` 连发三次零速度。按 `Ctrl+C` 时 `main` 会调用它，车就不会继续执行最后一条指令。为此 `rclpy.init(..., signal_handler_options=SignalHandlerOptions.NO)` 让我们自己的代码来处理 `Ctrl+C`，此时 ROS 还没关闭，停车指令还能发出去。

### 3. `scripts/plot_bag.py`

| 段 | 作用 |
|---|---|
| 1. 读 bag | `rosbag2_py.SequentialReader` 逐条读取消息；`deserialize_message` 把字节还原成 ROS 消息；时间用录制时间戳（秒） |
| `pose_series` | 从 Odometry 消息里取出 `t, x, y, yaw`（平面转动时 yaw = `2·atan2(z, w)`） |
| 2. 轨迹图 | `/odom` 和 `/ground_truth` 的 XY 轨迹，标出起点终点，两轴等比例 |
| 3. 速度图 | `/cmd_vel` 的线速度、角速度随时间变化 |
| 4. 里程计误差 | 用 `np.interp` 把 ground truth 插值到里程计的时间点，误差 = 两点的欧氏距离；bag 里没有 `/ground_truth`（真车）时跳过 |

所有图都保存为 **PDF**（矢量图，放报告）和 **PNG**（预览），坐标轴带国际单位和图例。

### 4. 答辩准备问题

1. 为什么回调函数传进去时不带括号？
2. 为什么小车停下的位置比 `obstacle_threshold` 更靠近墙？
3. 为什么目标角度要取整到 90° 的倍数？
4. `atan2(sin(e), cos(e))` 做了什么？为什么需要它？
5. 为什么用恒定角速度转弯，车会在目标角度附近来回抖？
6. 为什么把 `inf` 当作畅通？
7. 为什么里程计和真实位置会越差越多？为什么真车上误差更大？
8. 仿真和真车之间改了什么，为什么（QoS、没有 ground truth、光束下标）？
