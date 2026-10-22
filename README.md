# 🐢 SESASR-SIQUBro

**Bump&Go controller for TurtleBot3** — group repository for the labs of *Sensors, Embedded Systems and Algorithms for Service Robotics* (SESASR, Politecnico di Torino).

**Language / 语言：** [🇬🇧 English](#-english) · [🇨🇳 中文](#-中文)

| 📚 Guides / 指南 | |
|---|---|
| 🚀 [Operation Guide / 操作流程](docs/OPERATION_GUIDE.md) | Step by step from zero, simulation + real robot · 零基础分步操作，仿真 + 真车 |
| 📖 [Code Guide / 代码介绍指南](docs/CODE_GUIDE.md) | How the code works, section by section · 逐段讲解代码原理 |

> 🆕 **New to ROS 2? / 第一次用 ROS 2？** Start with the [Operation Guide](docs/OPERATION_GUIDE.md). / 从[操作流程](docs/OPERATION_GUIDE.md)开始。

![Trajectory in the Lab02 maze](docs/images/lab02_sim_trajectory.png)

---

## 🇬🇧 English

### 📌 What is in this repository

| Path | Content |
|---|---|
| `lab02_pkg/` | ROS 2 Python package with the `bump_and_go` node (Lab02 simulation, Lab03/Lab04 real robot) |
| `scripts/plot_bag.py` | Script that turns a recorded rosbag into report-ready plots (PDF + PNG, with units and legends) |
| `docs/images/` | Example plots from a simulation run |

Recorded rosbags are **not** stored here (they are large); share them through cloud storage.

### 🧰 Requirements

- Ubuntu 22.04 + **ROS 2 Humble**
- Python packages: `tf_transformations`
  ```bash
  sudo apt install ros-humble-tf-transformations
  ```
- For simulation: the course package [`SESASR-Course/turtlebot3_simulations`](https://github.com/SESASR-Course/turtlebot3_simulations), built in a workspace (see Lab02 document).
  - **Gazebo Classic** (Intel/AMD PCs): `turtlebot3_gazebo`
  - **Gazebo Fortress** (Apple Silicon Macs, M1/M2/M3…): `turtlebot3_ignition` (Lab02 document, Appendix B)

### 📥 Install

Clone the repository **inside the `src/` folder** of your ROS 2 workspace and build:

```bash
cd ~/ros2_ws/src                 # on the robot: ~/group_NNN/src
git clone https://github.com/Denvolatos/SESASR-SIQUBro.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash        # in EVERY new terminal
```

> `colcon build` must be run in the workspace root, not inside `src/`.

### 🖥️ Run in simulation (Lab02)

Terminal 1 – start the maze world:

```bash
source ~/ros_ws/install/setup.bash          # workspace with turtlebot3_simulations
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_ignition lab02.launch.py   # or: turtlebot3_gazebo
```

Terminal 2 – start the controller:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run lab02_pkg bump_and_go
```

The robot starts at `(x, y, yaw) = (0, 0, 0)` and stops when it is within `goal_tolerance` of the green cube.

### 🤖 Run on the real TurtleBot3 (Lab03 / Lab04)

1. Connect to `SESASR_WiFi`, then open a shell on the robot PC: `ssh students@192.168.10.1XX` (`XX` = robot number).
2. Check `~/.bashrc` on the robot (`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, sourced workspaces) as described in the Lab03 document.
3. Start the robot drivers (one `rmw_zenohd` only):
   ```bash
   ros2 run rmw_zenoh_cpp rmw_zenohd
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
4. Clone and build this repository in `~/group_NNN/src` (see *Install*).
5. **Test first with the robot lifted from the table**, then on the test field:
   ```bash
   ros2 run lab02_pkg bump_and_go --ros-args -p stop_at_goal:=false
   ```
   ⚠️ `Ctrl+C` stops the node but the robot may keep the last command — keep someone ready to pick it up.

The code already contains the three changes required by Lab03:

| Lab03 requirement | Where in `bump_and_go.py` |
|---|---|
| No `/ground_truth`, only odometry | `odom_callback` subscribes to `/odom` only |
| `/scan` with sensor QoS | `create_subscription(..., qos_profile_sensor_data)` |
| Ray direction from `angle_min` / `angle_increment` (no fixed indices) | `sector_distance` computes the angle of every ray |

### ⚙️ Parameters

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

Example: `ros2 run lab02_pkg bump_and_go --ros-args -p obstacle_threshold:=0.6`

### 🧠 How the controller works

The node is a small state machine updated by a timer at `control_frequency`:

```
            front > threshold
        ┌───────────────────────┐
        ▼                       │
     ┌──────┐  front ≤ threshold  ┌────────┐
     │  GO  │ ───────────────────▶│ ROTATE │
     └──────┘ ◀─────────────────── └────────┘
        │       |yaw error| < 2°
        │ near goal (simulation only)
        ▼
     ┌──────┐
     │ DONE │  (velocity = 0)
     └──────┘
```

1. **Sensors.** `scan_callback` stores the latest `LaserScan`; `odom_callback` stores `x`, `y` and the **yaw** obtained from the odometry quaternion with `tf_transformations.euler_from_quaternion`.
2. **Obstacle detection (Go).** `sector_distance(center, half_width)` returns the minimum valid range in an angular sector. The front sector is ±15°, the side sectors ±30° around ±90°. `inf` (nothing within 3.5 m) counts as free space; values below `range_min` are discarded.
3. **Turn decision (Bump).** When the front distance is below the threshold, the robot compares the left and right sectors and turns **90° towards the side with more free space**. The target yaw is rounded to the closest multiple of 90°, because the maze walls are aligned with the x and y axes; this keeps small turning errors from accumulating.
4. **Rotation.** A proportional controller `ω = 2·(target − yaw)`, saturated at `max_angular_vel`, turns the robot until the error is below 2°. The error is wrapped to [−π, π] with `atan2(sin, cos)`. (A constant ω would overshoot and oscillate: at 1.5 rad/s and 10 Hz each step is 8.6°, larger than the 2° tolerance.)

### 📊 Record and plot data

Record while the robot moves (start recording **before** the controller):

```bash
mkdir -p ~/ros2_ws/bags && cd ~/ros2_ws/bags
ros2 bag record -o lab02_sim /odom /ground_truth /cmd_vel /scan   # real robot: no /ground_truth
```

Stop with `Ctrl+C`, check with `ros2 bag info lab02_sim`, then plot:

```bash
source /opt/ros/humble/setup.bash
python3 ~/ros2_ws/src/SESASR-SIQUBro/scripts/plot_bag.py lab02_sim
```

Plots are saved in `bags/plots/`:

| File | Content |
|---|---|
| `*_trajectory` | XY path: odometry vs ground truth (if available) |
| `*_cmd_vel` | Linear and angular velocity commands over time |
| `*_odom_error` | Distance between odometry and ground truth (simulation only) |

### ✅ Example result (simulation, Gazebo Fortress)

- Goal reached at about `(5.87, 3.45)` m after 8 turns (2 left, 6 right), ~4 min of simulated driving.
- Odometry error vs ground truth: **0.022 m at the end, 0.089 m maximum**.

| Velocity commands | Odometry error |
|---|---|
| ![cmd_vel](docs/images/lab02_sim_cmd_vel.png) | ![odom error](docs/images/lab02_sim_odom_error.png) |

### 🛠️ Troubleshooting

| Problem | Cause → Fix |
|---|---|
| `Package 'lab02_pkg' not found` | Workspace not sourced in this terminal → `source ~/ros2_ws/install/setup.bash` |
| Gazebo crashes with `Segmentation fault` / `OpenGL 3.3 is not supported` (virtual machine) | VM graphics too old → `export LIBGL_ALWAYS_SOFTWARE=1` before launching |
| No topics after restarting Gazebo | An old Gazebo server is still running → wait for the prompt after `Ctrl+C`, or `pkill -f gazebo` |
| No `/scan` messages on the real robot | Wrong QoS → the subscription must use `qos_profile_sensor_data` (already done) |
| No topics on the real robot | `pkill -9 -f ros && ros2 daemon stop`, then restart the bringup |
| Robot stops too close to the wall | Reaction delay + inertia → increase `obstacle_threshold` (e.g. `0.6`) |
| Lidar rays not visible in Gazebo Fortress | Add the *Visualize Lidar* plugin (⋮ menu), refresh, select `/scan` |

### 👥 Team workflow

```bash
git pull                     # before you start
# ... edit, test ...
git add -A && git commit -m "describe the change"
git push
```

Every member must be able to explain the code at the oral exam: read it, run it, and change the parameters yourself.

### 🙏 Acknowledgements

The code and documentation of this repository were developed with the help of **[Claude Code](https://claude.com/claude-code)** (Anthropic's AI coding assistant), which was used for explanations, code review and writing the guides. All group members have read, run and tested the code and are responsible for it.

---

## 🇨🇳 中文

### 📌 仓库内容

| 路径 | 内容 |
|---|---|
| `lab02_pkg/` | ROS 2 Python 包，包含 `bump_and_go` 节点（Lab02 仿真，Lab03/Lab04 真车） |
| `scripts/plot_bag.py` | 把录好的 rosbag 画成报告可用的图（PDF + PNG，带单位和图例） |
| `docs/images/` | 一次仿真运行的示例图 |

录好的 rosbag **不放在这里**（文件太大），请用网盘共享。

### 🧰 环境要求

- Ubuntu 22.04 + **ROS 2 Humble**
- Python 包 `tf_transformations`：
  ```bash
  sudo apt install ros-humble-tf-transformations
  ```
- 仿真需要课程提供的 [`SESASR-Course/turtlebot3_simulations`](https://github.com/SESASR-Course/turtlebot3_simulations)，按 Lab02 文档编译到一个工作区里：
  - **Gazebo Classic**（Intel/AMD 电脑）：用 `turtlebot3_gazebo`
  - **Gazebo Fortress**（M1/M2/M3 等苹果芯片 Mac）：用 `turtlebot3_ignition`（Lab02 文档附录 B）

### 📥 安装

把仓库克隆到 ROS 2 工作区的 **`src/` 文件夹里**，再编译：

```bash
cd ~/ros2_ws/src                 # 在机器人上是 ~/group_NNN/src
git clone https://github.com/Denvolatos/SESASR-SIQUBro.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash        # 每个新终端都要运行
```

> `colcon build` 必须在工作区根目录运行，不能在 `src/` 里。

### 🖥️ 仿真运行（Lab02）

终端 1：启动迷宫世界

```bash
source ~/ros_ws/install/setup.bash          # 编译了 turtlebot3_simulations 的工作区
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_ignition lab02.launch.py   # Gazebo Classic 用 turtlebot3_gazebo
```

终端 2：启动控制器

```bash
source ~/ros2_ws/install/setup.bash
ros2 run lab02_pkg bump_and_go
```

小车从 `(x, y, yaw) = (0, 0, 0)` 出发，到绿块 `goal_tolerance` 范围内自动停车。

### 🤖 真车运行（Lab03 / Lab04）

1. 连 `SESASR_WiFi`，登录机器人电脑：`ssh students@192.168.10.1XX`（`XX` 是机器人编号）。
2. 按 Lab03 文档检查机器人上的 `~/.bashrc`（`ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION=rmw_zenoh_cpp`、source 了哪些工作区）。
3. 启动机器人驱动（`rmw_zenohd` 只开一个）：
   ```bash
   ros2 run rmw_zenoh_cpp rmw_zenohd
   ros2 launch turtlebot3_bringup robot.launch.py
   ```
4. 在 `~/group_NNN/src` 里克隆并编译本仓库（见"安装"）。
5. **先把车架空测试**，没问题再放到场地上：
   ```bash
   ros2 run lab02_pkg bump_and_go --ros-args -p stop_at_goal:=false
   ```
   ⚠️ `Ctrl+C` 只会停掉程序，车可能还会按最后一条指令继续动，旁边要有人随时把车拿起来。

代码里已经做好了 Lab03 要求的三处修改：

| Lab03 要求 | 在 `bump_and_go.py` 的位置 |
|---|---|
| 不用 `/ground_truth`，只用里程计 | `odom_callback` 只订阅 `/odom` |
| `/scan` 使用传感器 QoS | `create_subscription(..., qos_profile_sensor_data)` |
| 用 `angle_min` / `angle_increment` 算光束方向（不写死下标） | `sector_distance` 逐条计算每条光束的角度 |

### ⚙️ 参数

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

例子：`ros2 run lab02_pkg bump_and_go --ros-args -p obstacle_threshold:=0.6`

### 🧠 控制器原理

节点是一个由定时器按 `control_frequency` 驱动的小状态机（状态图见英文部分）：

- **GO（前进）**：前方畅通就以最大速度前进
- **ROTATE（转向）**：前方太近就原地转 90°，转到误差小于 2° 回到 GO
- **DONE（结束）**：仅仿真用，到达绿块后速度为 0

1. **传感器**：`scan_callback` 保存最新的雷达数据；`odom_callback` 保存 `x`、`y`，并用 `tf_transformations.euler_from_quaternion` 把里程计的四元数转换成 **yaw（偏航角）**。
2. **障碍检测（Go）**：`sector_distance(中心角, 半宽)` 返回一个扇形区域内最近的有效距离。前方看 ±15°，左右各看 ±90° 附近 ±30°。`inf`（3.5 m 内什么都没有）当作畅通；小于 `range_min` 的值丢弃。
3. **转向决策（Bump）**：前方距离小于阈值时，比较左右两个扇形，**向空间更大的一侧转 90°**。目标角度取整到最近的 90° 倍数，因为迷宫的墙都和 x、y 轴平行，这样每次转弯的小误差不会越积越多。
4. **转动控制**：比例控制 `ω = 2·(目标 − yaw)`，限制在 `max_angular_vel` 以内，误差小于 2° 就停止转动。误差用 `atan2(sin, cos)` 限制在 [−π, π]。（如果用恒定转速会冲过头、来回抖：1.5 rad/s、10 Hz 时每步转 8.6°，比 2° 的容许误差大。）

### 📊 录数据和画图

车动起来之前**先开始录**：

```bash
mkdir -p ~/ros2_ws/bags && cd ~/ros2_ws/bags
ros2 bag record -o lab02_sim /odom /ground_truth /cmd_vel /scan   # 真车上没有 /ground_truth
```

按 `Ctrl+C` 停止录制，用 `ros2 bag info lab02_sim` 检查，然后画图：

```bash
source /opt/ros/humble/setup.bash
python3 ~/ros2_ws/src/SESASR-SIQUBro/scripts/plot_bag.py lab02_sim
```

图保存在 `bags/plots/`：

| 文件 | 内容 |
|---|---|
| `*_trajectory` | XY 轨迹：里程计 vs 真实位置（有的话） |
| `*_cmd_vel` | 线速度和角速度指令随时间的变化 |
| `*_odom_error` | 里程计和真实位置的距离误差（仅仿真） |

### ✅ 示例结果（仿真，Gazebo Fortress）

- 转了 8 次弯（左 2 次、右 6 次），约 4 分钟后到达 `(5.87, 3.45)` m 附近，停在绿块旁。
- 里程计误差（与 ground truth 比）：**结束时 0.022 m，最大 0.089 m**。

### 🛠️ 常见问题

| 问题 | 原因 → 解决 |
|---|---|
| `Package 'lab02_pkg' not found` | 这个终端没 source 工作区 → `source ~/ros2_ws/install/setup.bash` |
| 虚拟机里 Gazebo 崩溃：`Segmentation fault` / `OpenGL 3.3 is not supported` | 虚拟显卡不支持 → 启动前 `export LIBGL_ALWAYS_SOFTWARE=1` |
| 重开 Gazebo 后没有话题 | 旧的 Gazebo 服务器还在跑 → `Ctrl+C` 后等提示符出现再重开，或 `pkill -f gazebo` |
| 真车上收不到 `/scan` | QoS 不对 → 订阅要用 `qos_profile_sensor_data`（已改好） |
| 真车上看不到话题 | `pkill -9 -f ros && ros2 daemon stop`，再重新 bringup |
| 车离墙太近才停 | 反应延迟 + 惯性 → 调大 `obstacle_threshold`（如 `0.6`） |
| Gazebo Fortress 里看不到激光线 | 在 ⋮ 菜单加 *Visualize Lidar* 插件，点刷新，选 `/scan` |

### 👥 小组协作

```bash
git pull                     # 开始前先拉取
# ……修改、测试……
git add -A && git commit -m "写清楚改了什么"
git push
```

答辩时每个人都要能讲清楚这份代码：自己读一遍、自己运行、自己改改参数试试。

### 🙏 致谢

本仓库的代码和文档在 **[Claude Code](https://claude.com/claude-code)**（Anthropic 推出的 AI 编程助手）的帮助下完成，用于讲解原理、检查代码和编写指南。小组成员都阅读、运行并测试过代码，并对代码负责。
