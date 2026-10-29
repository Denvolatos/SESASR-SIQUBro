# 🚀 Operation Guide (step by step, for beginners) / 操作流程（零基础版）

**Language / 语言：** [🇬🇧 English](#-english) · [🇨🇳 中文](#-中文) · [⬅️ README](../README.md)

---

## 🇬🇧 English

This guide assumes **no prior experience**. Follow the steps in order. Each step says **where** to type, **what** to type and **what you should see**.

### Part 0 — Five words you need to know

| Word | Meaning |
|---|---|
| **Terminal** | A window where you type commands. In Ubuntu: press `Ctrl+Alt+T`, or search "Terminal" in the apps menu. Type a command, press `Enter`. |
| **Workspace** | A folder for ROS 2 code. In this guide `~/SESASR-SIQUBro` is the group workspace, with ROS packages in `src/`. (`~` means your home folder.) |
| **Build** | `colcon build` prepares the code so ROS 2 can run it. Always run it in the workspace folder, never inside `src/`. |
| **Source** | `source install/setup.bash` tells *this* terminal where your code is. **Every new terminal needs it** — otherwise you get `Package not found`. |
| **Node / Topic** | A node is a running program. Nodes talk through topics: the controller *reads* `/scan` (laser) and `/odom` (position) and *writes* `/cmd_vel` (velocity). |

Useful keys: `Ctrl+C` stops the program running in a terminal. In the Ubuntu terminal, paste is `Ctrl+Shift+V`.

### Part 1 — Before you start (once)

You need **Ubuntu 22.04 with ROS 2 Humble**. If you do not have it, follow the official course guide: <https://sesasr-course.github.io/ros2_setup_guide/>

Check that ROS 2 works — open a terminal and type:

```bash
ros2 --help
```

✅ You see a list of commands. ❌ `ros2: command not found` → ROS 2 is not installed or not sourced: run `source /opt/ros/humble/setup.bash` and try again.

Install the extra libraries (asks for your password; nothing is shown while you type it — this is normal):

```bash
sudo apt update
sudo apt install -y git ros-humble-tf-transformations
```

### Part 2 — Get the code (once)

```bash
git clone https://github.com/Denvolatos/SESASR-SIQUBro.git ~/SESASR-SIQUBro
cd ~/SESASR-SIQUBro
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

✅ The last line says `Summary: 1 package finished`.

If you work from a fork, replace the clone URL with your fork URL. If you already cloned the repository, keep that checkout and run the build commands inside it. The Git repository itself is the workspace root; `src/` contains the ROS packages.

Later, to get your teammates' latest changes:

```bash
cd ~/SESASR-SIQUBro
git pull
colcon build --symlink-install
```

### Part 3 — Prepare the simulation (once)

The maze world comes from the course package. Install it in a **separate** workspace `~/ros_ws`:

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone https://github.com/SESASR-Course/turtlebot3_simulations.git
```

**Only on Apple Silicon Macs (M1/M2/M3…)** — switch to Gazebo Fortress:

```bash
rm turtlebot3_simulations/turtlebot3_ignition/COLCON_IGNORE
touch turtlebot3_simulations/turtlebot3_gazebo/COLCON_IGNORE
```

Then (everyone):

```bash
cd ~/ros_ws
sudo rosdep init        # only the very first time; an "already exists" error is fine
rosdep update
rosdep install --from-path src --ignore-src -y -r
colcon build --symlink-install
```

✅ `Summary: ... packages finished`.

If the course simulation is already built in another workspace (for example `~/ros2_ws`), use that workspace in Terminal 1 below. Do not source a workspace containing another `lab02_pkg` in the controller terminal.

### Part 4 — Run the simulation (every time)

You need **two terminals**.

**Terminal 1 — the world**

```bash
source ~/ros_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_ignition lab02.launch.py
```

- If you already built the simulation in `~/ros2_ws`, replace `~/ros_ws` in Terminal 1 with `~/ros2_ws`.
- On Intel/AMD PCs with Gazebo Classic use `turtlebot3_gazebo` instead of `turtlebot3_ignition`.
- **In a virtual machine** (UTM, VirtualBox…), first run `export LIBGL_ALWAYS_SOFTWARE=1`, otherwise Gazebo may crash.

✅ After ~20 s a Gazebo window shows the blue maze, a green cube and the small robot.

**Terminal 2 — the controller**

```bash
source ~/SESASR-SIQUBro/install/setup.bash
ros2 launch lab02_pkg bump_and_go.launch.py
```

✅ The robot drives forward; at each wall the terminal prints a line like
`wall at 0.50 m (left 1.08, right 0.91) -> turn left`.
After about 4 minutes: `goal reached at x=5.87 y=3.45`.

**To restart from the beginning:** `Ctrl+C` in both terminals, wait until the prompt (`$`) is back, then start Terminal 1 and Terminal 2 again.

### Part 5 — Record data and make plots

Open a **third terminal** *after* the world is running and *before* starting the controller:

```bash
mkdir -p ~/SESASR-SIQUBro/bags && cd ~/SESASR-SIQUBro/bags
ros2 bag record -o lab02_sim /odom /ground_truth /cmd_vel /scan
```

Start the controller (Terminal 2). When it prints `goal reached`, press `Ctrl+C` in the third terminal. Check the recording:

```bash
ros2 bag info lab02_sim
```

✅ You see the 4 topics with a message count each. ❌ `already exists` → choose another name, e.g. `-o lab02_sim_2`.

Make the plots:

```bash
python3 ~/SESASR-SIQUBro/scripts/plot_bag.py lab02_sim
```

✅ The PDF/PNG files are in `~/SESASR-SIQUBro/bags/plots/`. Use the PDFs in the report. The `bags/` directory is ignored by Git.

### Part 6 — Real robot (Lab03 / Lab04)

**Before going to the lab:** know your **robot number `XX`** and **group number `NNN`**.

| # | Where | What to do | ✅ Check |
|---|---|---|---|
| 1 | Your PC | Connect to Wi-Fi `SESASR_WiFi` (password in the Lab03 document) | Internet icon connected |
| 2 | Your PC terminal | `ssh students@192.168.10.1XX` (password in the Lab03 document); answer `yes` the first time | Prompt becomes `students@nucXX` |
| 3 | Robot (SSH) | `nano ~/.bashrc`: check `ROS_DOMAIN_ID=XX` (no leading 0), `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, comment other groups' workspaces with `#`. Save: `Ctrl+O`, `Enter`, `Ctrl+X`. Then `source ~/.bashrc`. If another group's workspace stays in `.bashrc` (it may also contain a `lab02_pkg`), run `source ~/group_NNN/install/setup.bash` in every new terminal | — |
| 4 | Robot (SSH) | `ros2 run rmw_zenoh_cpp rmw_zenohd` (keep it running; only one!) | No error |
| 5 | Robot (new SSH) | `ros2 launch turtlebot3_bringup robot.launch.py` | Log shows `LDS-02 started successfully` and `Run!` |
| 6 | Robot (new SSH) | `mkdir -p ~/group_NNN/src && cd ~/group_NNN/src && git clone https://github.com/Denvolatos/SESASR-SIQUBro.git && cd .. && colcon build --symlink-install && source install/setup.bash` | `1 package finished` |
| 7 | Desk | **Robot lifted, wheels in the air.** Power on; then in step 6's terminal: `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"` | Wheels turn; `Ctrl+C` to stop. (Do not test the controller lifted: the yaw comes from the IMU, so a turn never finishes.) |
| 8 | Test field | Put the robot down, start recording in another SSH: `ros2 bag record -o lab04_real /odom /cmd_vel /scan /imu`, then in step 6's terminal: `ros2 launch lab02_pkg bump_and_go.launch.py robot:=real` | Robot avoids walls |
| 9 | After the run | `Ctrl+C` the recorder, copy the bag to your PC (see below) | `ros2 bag info lab03_real` works |

If the robot PC has no internet (git clone fails), copy the code from your PC instead:

```bash
scp -r ~/SESASR-SIQUBro/src/lab02_pkg students@192.168.10.1XX:~/group_NNN/src/
```

Copy the recording back to your PC (run on **your PC**):

```bash
scp -r students@192.168.10.1XX:~/group_NNN/bags/lab03_real ~/SESASR-SIQUBro/bags/
```

(Adjust the path to where you started `ros2 bag record`.)

⚠️ Safety: stay next to the robot. `Ctrl+C` stops the program, but the robot may keep its last command — lift it if needed. Emergency stop from any terminal: `pkill -f "lib/lab02_pkg/bump_and_go"; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`. To change speed, turn angle or threshold, edit `src/lab02_pkg/config/real_robot.yaml` and run `colcon build` again.

### Part 7 — Optional: see the robot's laser in RViz on your PC

On your PC (same Wi-Fi), in a new terminal:

```bash
export ROS_DOMAIN_ID=XX
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.10.1XX:7447"]'
ros2 launch turtlebot3_bringup rviz2.launch.py
```

(In a virtual machine add `export LIBGL_ALWAYS_SOFTWARE=1` first.)

### Part 8 — Troubleshooting


| Problem | Cause → Fix |
|---|---|
| `Package 'lab02_pkg' not found` | Group workspace not sourced in this terminal → `source ~/SESASR-SIQUBro/install/setup.bash` |
| Gazebo crashes with `Segmentation fault` / `OpenGL 3.3 is not supported` (virtual machine) | VM graphics too old → `export LIBGL_ALWAYS_SOFTWARE=1` before launching |
| No topics after restarting Gazebo | An old Gazebo server is still running → wait for the prompt after `Ctrl+C`, or `pkill -f gazebo` |
| No `/scan` messages on the real robot | Wrong QoS → the subscription must use `qos_profile_sensor_data` (already done) |
| No topics on the real robot | `pkill -9 -f ros && ros2 daemon stop`, then restart the bringup |
| Robot stops too close to the wall | Reaction delay + inertia → increase `obstacle_threshold` (e.g. `0.6`) |
| Lidar rays not visible in Gazebo Fortress | Add the *Visualize Lidar* plugin (⋮ menu), refresh, select `/scan` |

### Part 9 — Working as a team (git)


```bash
cd ~/SESASR-SIQUBro
git pull                     # before you start
# ... edit, test ...
git add -A && git commit -m "describe the change"
git push
```

Every member must be able to explain the code at the oral exam: read it, run it, and change the parameters yourself.

### Part 10 — Real-robot debugging notes and Lab04 checklist

What we learned on 22/10 (robot 3) and in simulation tests:

| Test (simulation, motor dead zone added with `scripts/deadzone_filter.py`) | Result |
|---|---|
| A: old parameters (2° tolerance, no minimum ω) | Drives 4 m, then **stuck forever in the first turn**: the P controller asks for ω≈0.17 rad/s, below the dead zone |
| B: new parameters (5° tolerance, minimum ω 0.3) | **Reaches the goal in 160 s** |
| C: extra node publishing zero velocity | Still drives forward → not the cause of "not moving at start" |
| D: obstacle in front at start | Turns immediately, then stuck → does not match what we saw |

Conclusion: the controller logic is correct; "not moving at start" on the real robot was most likely a communication problem (the Zenoh router stopped several times; nodes started before a router restart cannot reach the robot).

**Lab04 results (29/10, robot 3)** — these are now the defaults in `config/real_robot.yaml`:

| Finding | Evidence | Fix |
|---|---|---|
| **At 0.22 m/s the robot only turns, it never drives forward** (also the "only turns" of 22/10) | Controller sends `v=0.22` for 15 s, front distance unchanged; 0.1 and 0.15 m/s drive normally | `max_linear_vel: 0.15` |
| Choosing the side by the closest point picks the wrong way (one corner hides a corridor) → the robot went back the way it came | Turn log of `lab04_real_3` | Median of the side sector (real robot only; `side_metric`) |
| Turns snapped to 90° multiples are too large: the odometry axes depend on how the robot was placed | — | `snap_to_axes: false`, `turn_until_clear: true`, `turn_angle_deg: 45` |
| The LDS-02 reports **NaN** for no return and `range_min = 0.0` | `ros2 topic echo /scan --once` | NaN ignored, readings below 5 cm ignored |
| Lifted, a turn never finishes | Driver uses `use_imu: true` | Test turning on the floor only |
| The robot PC reboots when the power cable is pulled | `uptime` | Use the battery before moving the robot; restart router and bringup |

**Lab04 checklist**

1. Start in this order, each in its own terminal, and never close them: **router → bringup → (rosbag) → controller**. If the router is restarted, restart bringup and the controller too.
2. Check the router is listening: `ss -ltn | grep 7447`
3. Test the motors alone (robot lifted): `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"` (not 0.22, see below)
4. Run the controller and read its status line every 2 s (`state=... v=... odom=...`).
5. In another terminal: `ros2 topic info /cmd_vel` → `Publisher count: 1`, `Subscription count: 1`.
6. If bringup prints `There is no status packet` / `stack smashing detected`: power the motor board off for 10 s, on again (wait for the music), restart bringup.
7. From the repository root, test runner: `SIM_WS=~/ros_ws scripts/run_dz_test.sh <label> <tolerance_deg> <min_w> <seconds> [none|zero|box]` (simulation, Gazebo Classic; set `SIM_WS=~/ros2_ws` if that is your simulation workspace).

---

## 🇨🇳 中文

本指南假设你**完全没有基础**。请按顺序做，每一步都写了**在哪里**敲、**敲什么**、**正常情况应该看到什么**。

### 第 0 部分：先认识五个词

| 词 | 意思 |
|---|---|
| **终端（Terminal）** | 输入命令的窗口。Ubuntu 里按 `Ctrl+Alt+T`，或在应用菜单搜 "Terminal"。敲完命令按 `Enter` 回车。 |
| **工作区（Workspace）** | 放 ROS 2 代码的文件夹。本指南以 `~/SESASR-SIQUBro` 为小组工作区，ROS 包放在其中的 `src/`。（`~` 表示你的家目录。） |
| **编译（Build）** | `colcon build` 把代码准备好，让 ROS 2 能运行它。一定要在工作区根目录运行，不能在 `src/` 里面。 |
| **Source** | `source install/setup.bash` 告诉**当前这个**终端你的代码在哪里。**每开一个新终端都要做一次**，否则会报 `Package not found`。 |
| **节点 / 话题** | 节点是一个正在运行的程序，节点之间通过话题通信：控制器**读** `/scan`（激光）和 `/odom`（位置），**写** `/cmd_vel`（速度）。 |

常用按键：`Ctrl+C` 停止终端里正在运行的程序。Ubuntu 终端里粘贴用 `Ctrl+Shift+V`。

### 第 1 部分：开始之前（只做一次）

需要 **Ubuntu 22.04 + ROS 2 Humble**。还没装的话，按课程官方指南装：<https://sesasr-course.github.io/ros2_setup_guide/>

检查 ROS 2 能不能用，打开终端输入：

```bash
ros2 --help
```

✅ 出现一串命令列表。❌ 显示 `ros2: command not found` → ROS 2 没装好或没 source：先运行 `source /opt/ros/humble/setup.bash` 再试。

安装额外的库（会要求输入密码，输入时屏幕上什么都不显示，这是正常的）：

```bash
sudo apt update
sudo apt install -y git ros-humble-tf-transformations
```

### 第 2 部分：下载代码（只做一次）

```bash
git clone https://github.com/Denvolatos/SESASR-SIQUBro.git ~/SESASR-SIQUBro
cd ~/SESASR-SIQUBro
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

✅ 最后一行显示 `Summary: 1 package finished`。

如果你使用自己的 fork，把克隆 URL 换成 fork 地址。已经克隆过的，保留现有目录，直接在仓库根目录编译。Git 仓库本身就是工作区根目录，ROS 包放在 `src/`。

以后要拿到组员的最新修改：

```bash
cd ~/SESASR-SIQUBro
git pull
colcon build --symlink-install
```

### 第 3 部分：准备仿真环境（只做一次）

迷宫场景来自课程提供的包，把它装在**另一个**工作区 `~/ros_ws` 里：

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone https://github.com/SESASR-Course/turtlebot3_simulations.git
```

**只有苹果芯片 Mac（M1/M2/M3…）要做**：切换到 Gazebo Fortress

```bash
rm turtlebot3_simulations/turtlebot3_ignition/COLCON_IGNORE
touch turtlebot3_simulations/turtlebot3_gazebo/COLCON_IGNORE
```

然后（所有人）：

```bash
cd ~/ros_ws
sudo rosdep init        # 只有第一次需要；如果提示 already exists 也没关系
rosdep update
rosdep install --from-path src --ignore-src -y -r
colcon build --symlink-install
```

✅ 显示 `Summary: ... packages finished`。

如果课程仿真已经编译在其他工作区（例如 `~/ros2_ws`），下面的终端 1 改为加载那个工作区。运行控制器的终端不要加载含有另一个 `lab02_pkg` 的工作区。

### 第 4 部分：运行仿真（每次）

需要开**两个终端**。

**终端 1：仿真世界**

```bash
source ~/ros_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_ignition lab02.launch.py
```

- 如果已有仿真编译在 `~/ros2_ws`，终端 1 把 `~/ros_ws` 换成 `~/ros2_ws`。
- Intel/AMD 电脑用 Gazebo Classic 的话，把 `turtlebot3_ignition` 换成 `turtlebot3_gazebo`。
- **在虚拟机里**（UTM、VirtualBox 等）先运行 `export LIBGL_ALWAYS_SOFTWARE=1`，否则 Gazebo 可能会崩溃。

✅ 大约 20 秒后弹出 Gazebo 窗口，能看到蓝色的迷宫、绿色方块和小车。

**终端 2：控制器**

```bash
source ~/SESASR-SIQUBro/install/setup.bash
ros2 launch lab02_pkg bump_and_go.launch.py
```

✅ 小车开始往前走，每遇到一面墙，终端打印一行，比如
`wall at 0.50 m (left 1.08, right 0.91) -> turn left`。
大约 4 分钟后显示 `goal reached at x=5.87 y=3.45`。

**想从头再跑一遍**：两个终端都按 `Ctrl+C`，等提示符 `$` 重新出现，再按顺序启动终端 1 和终端 2。

### 第 5 部分：录数据和画图

在仿真世界**已经打开**、控制器**还没启动**的时候，开**第三个终端**：

```bash
mkdir -p ~/SESASR-SIQUBro/bags && cd ~/SESASR-SIQUBro/bags
ros2 bag record -o lab02_sim /odom /ground_truth /cmd_vel /scan
```

然后启动控制器（终端 2）。看到 `goal reached` 后，在第三个终端按 `Ctrl+C`。检查录到的数据：

```bash
ros2 bag info lab02_sim
```

✅ 列出 4 个话题，每个都有消息条数。❌ 提示 `already exists` → 换个名字，比如 `-o lab02_sim_2`。

画图：

```bash
python3 ~/SESASR-SIQUBro/scripts/plot_bag.py lab02_sim
```

✅ PDF/PNG 图片在 `~/SESASR-SIQUBro/bags/plots/` 里，报告里用 PDF 版。`bags/` 已被 Git 忽略。

### 第 6 部分：真车（Lab03 / Lab04）

**去实验室之前**：问清楚你们的**机器人编号 `XX`** 和**组号 `NNN`**。

| # | 在哪里 | 做什么 | ✅ 检查 |
|---|---|---|---|
| 1 | 自己电脑 | 连 WiFi `SESASR_WiFi`（密码见 Lab03 文档） | 网络已连接 |
| 2 | 自己电脑的终端 | `ssh students@192.168.10.1XX`（密码见 Lab03 文档），第一次连接时回答 `yes` | 提示符变成 `students@nucXX` |
| 3 | 机器人（SSH） | `nano ~/.bashrc`：检查 `ROS_DOMAIN_ID=XX`（前面不带 0）、`RMW_IMPLEMENTATION=rmw_zenoh_cpp`，其他组的工作区那几行前面加 `#` 注释掉。保存：`Ctrl+O`、回车、`Ctrl+X`。然后 `source ~/.bashrc`。如果 `.bashrc` 里还留着别的组的工作区（里面可能也有 `lab02_pkg`），每个新终端都先运行 `source ~/group_NNN/install/setup.bash` | — |
| 4 | 机器人（SSH） | `ros2 run rmw_zenoh_cpp rmw_zenohd`（保持运行，只开一个！） | 没有报错 |
| 5 | 机器人（新开 SSH） | `ros2 launch turtlebot3_bringup robot.launch.py` | 日志里有 `LDS-02 started successfully` 和 `Run!` |
| 6 | 机器人（新开 SSH） | `mkdir -p ~/group_NNN/src && cd ~/group_NNN/src && git clone https://github.com/Denvolatos/SESASR-SIQUBro.git && cd .. && colcon build --symlink-install && source install/setup.bash` | `1 package finished` |
| 7 | 桌上 | **把车架空，轮子离开桌面**。打开电源，在第 6 步的终端里运行：`ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"` | 轮子会转；`Ctrl+C` 停止。（不要架空测控制器：朝向来自 IMU，车身不转，转弯永远转不完。） |
| 8 | 场地上 | 把车放到地上，另开一个 SSH 开始录：`ros2 bag record -o lab04_real /odom /cmd_vel /scan /imu`，再在第 6 步的终端里运行：`ros2 launch lab02_pkg bump_and_go.launch.py robot:=real` | 车能自己绕开墙 |
| 9 | 跑完之后 | 录制终端按 `Ctrl+C`，把录像拷回自己电脑（见下面） | `ros2 bag info lab03_real` 正常 |

如果机器人电脑上不了网（git clone 失败），就从自己电脑把代码拷过去：

```bash
scp -r ~/SESASR-SIQUBro/src/lab02_pkg students@192.168.10.1XX:~/group_NNN/src/
```

把录像拷回自己电脑（在**自己电脑**上运行）：

```bash
scp -r students@192.168.10.1XX:~/group_NNN/bags/lab03_real ~/SESASR-SIQUBro/bags/
```

（路径改成你运行 `ros2 bag record` 时所在的文件夹。）

⚠️ 安全：人要站在车旁边。`Ctrl+C` 会停掉程序，但车可能还会按最后一条指令继续动，必要时直接把车拿起来。任何终端都能急停：`pkill -f "lib/lab02_pkg/bump_and_go"; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`。要改速度、转弯角度或阈值，就改 `src/lab02_pkg/config/real_robot.yaml`，然后重新 `colcon build`。

### 第 7 部分（可选）：在自己电脑上用 RViz 看真车的激光

自己电脑（连同一个 WiFi），新开一个终端：

```bash
export ROS_DOMAIN_ID=XX
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.10.1XX:7447"]'
ros2 launch turtlebot3_bringup rviz2.launch.py
```

（在虚拟机里要先加 `export LIBGL_ALWAYS_SOFTWARE=1`。）

### 第 8 部分：常见问题


| 问题 | 原因 → 解决 |
|---|---|
| `Package 'lab02_pkg' not found` | 这个终端没加载小组工作区 → `source ~/SESASR-SIQUBro/install/setup.bash` |
| 虚拟机里 Gazebo 崩溃：`Segmentation fault` / `OpenGL 3.3 is not supported` | 虚拟显卡不支持 → 启动前 `export LIBGL_ALWAYS_SOFTWARE=1` |
| 重开 Gazebo 后没有话题 | 旧的 Gazebo 服务器还在跑 → `Ctrl+C` 后等提示符出现再重开，或 `pkill -f gazebo` |
| 真车上收不到 `/scan` | QoS 不对 → 订阅要用 `qos_profile_sensor_data`（已改好） |
| 真车上看不到话题 | `pkill -9 -f ros && ros2 daemon stop`，再重新 bringup |
| 车离墙太近才停 | 反应延迟 + 惯性 → 调大 `obstacle_threshold`（如 `0.6`） |
| Gazebo Fortress 里看不到激光线 | 在 ⋮ 菜单加 *Visualize Lidar* 插件，点刷新，选 `/scan` |

### 第 9 部分：小组协作（git）


```bash
cd ~/SESASR-SIQUBro
git pull                     # 开始前先拉取
# ……修改、测试……
git add -A && git commit -m "写清楚改了什么"
git push
```

答辩时每个人都要能讲清楚这份代码：自己读一遍、自己运行、自己改改参数试试。

### 第 10 部分：真车排障记录和 Lab04 检查清单

10/22（3 号机器人）和之后仿真测试的结论：

| 测试（仿真，用 `scripts/deadzone_filter.py` 加入电机死区） | 结果 |
|---|---|
| A：旧参数（容差 2°、没有最小转速） | 走了 4 m 后**卡在第一个弯**：P 控制给出的转速约 0.17 rad/s，低于死区 |
| B：新参数（容差 5°、最小转速 0.3） | **160 秒到达终点** |
| C：另有程序一直发零速度 | 车照样往前走，不是"一开始不动"的原因 |
| D：开局车前就有障碍 | 立刻转弯后卡住，和当天现象不符 |

结论：控制器逻辑没问题；真车"一开始不动"最可能是通信问题（当天 zenoh 路由停过好几次，路由重启前启动的节点连不上车）。

**Lab04 结果（10/29，3 号机器人）**，现在都已经是 `config/real_robot.yaml` 的默认值：

| 发现 | 证据 | 解决 |
|---|---|---|
| **速度 0.22 m/s 时车只转不走**（10/22 的"只转不走"也是这个原因） | 控制器发 `v=0.22` 发了 15 秒，前方距离不变；0.1 和 0.15 m/s 都能正常前进 | `max_linear_vel: 0.15` |
| 用最近点选左右会选错（一个墙角挡住整条通道），车走回头路 | `lab04_real_3` 的转弯记录 | 左右改用扇区中位数（只用于真车；`side_metric`） |
| 转向角对齐 90° 倍数转得太多：里程计坐标轴取决于开机时车怎么放 | — | `snap_to_axes: false`、`turn_until_clear: true`、`turn_angle_deg: 45` |
| LDS-02 打空返回 **NaN**，且 `range_min = 0.0` | `ros2 topic echo /scan --once` | 丢弃 NaN 和小于 5 cm 的读数 |
| 车架空时转弯永远转不完 | 驱动设置 `use_imu: true` | 只在地上测转弯 |
| 拔电源线时机器人电脑会重启 | `uptime` | 搬车前先换电池；重启后重开路由和 bringup |

**Lab04 检查清单**

1. 按顺序启动，每个一个终端，开了就别关：**路由 → bringup →（录制）→ 控制器**。路由一旦重启，bringup 和控制器也要重启。
2. 确认路由在监听：`ss -ltn | grep 7447`
3. 单独测电机（车架空）：`ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"`（不要用 0.22，见下面）
4. 运行控制器，看它每 2 秒打印的状态（`state=... v=... odom=...`）。
5. 另一个终端：`ros2 topic info /cmd_vel` → `Publisher count: 1`、`Subscription count: 1`。
6. bringup 报 `There is no status packet` / `stack smashing detected`：电机板断电 10 秒再开（等音乐响完），重启 bringup。
7. 在仓库根目录运行测试脚本：`SIM_WS=~/ros_ws scripts/run_dz_test.sh <标签> <容差°> <最小转速> <秒> [none|zero|box]`（仿真，Gazebo Classic；如果仿真在 `~/ros2_ws`，把 `SIM_WS` 改成 `~/ros2_ws`）。

