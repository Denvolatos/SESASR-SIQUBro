# 🚀 Operation Guide (step by step, for beginners) / 操作流程（零基础版）

**Language / 语言：** [🇬🇧 English](#-english) · [🇨🇳 中文](#-中文) · [⬅️ README](../README.md)

---

## 🇬🇧 English

This guide assumes **no prior experience**. Follow the steps in order. Each step says **where** to type, **what** to type and **what you should see**.

### Part 0 — Five words you need to know

| Word | Meaning |
|---|---|
| **Terminal** | A window where you type commands. In Ubuntu: press `Ctrl+Alt+T`, or search "Terminal" in the apps menu. Type a command, press `Enter`. |
| **Workspace** | A folder for ROS 2 code, e.g. `~/ros2_ws`. Source code goes in its `src/` sub-folder. (`~` means your home folder.) |
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
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Denvolatos/SESASR-SIQUBro.git
cd ~/ros2_ws
colcon build --symlink-install
```

✅ The last line says `Summary: 1 package finished`.

Later, to get your teammates' latest changes:

```bash
cd ~/ros2_ws/src/SESASR-SIQUBro && git pull
cd ~/ros2_ws && colcon build --symlink-install
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

### Part 4 — Run the simulation (every time)

You need **two terminals**.

**Terminal 1 — the world**

```bash
source ~/ros_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_ignition lab02.launch.py
```

- On Intel/AMD PCs with Gazebo Classic use `turtlebot3_gazebo` instead of `turtlebot3_ignition`.
- **In a virtual machine** (UTM, VirtualBox…), first run `export LIBGL_ALWAYS_SOFTWARE=1`, otherwise Gazebo may crash.

✅ After ~20 s a Gazebo window shows the blue maze, a green cube and the small robot.

**Terminal 2 — the controller**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run lab02_pkg bump_and_go
```

✅ The robot drives forward; at each wall the terminal prints a line like
`wall at 0.50 m (left 1.08, right 0.91) -> turn left`.
After about 4 minutes: `goal reached at x=5.87 y=3.45`.

**To restart from the beginning:** `Ctrl+C` in both terminals, wait until the prompt (`$`) is back, then start Terminal 1 and Terminal 2 again.

### Part 5 — Record data and make plots

Open a **third terminal** *after* the world is running and *before* starting the controller:

```bash
mkdir -p ~/ros2_ws/bags && cd ~/ros2_ws/bags
ros2 bag record -o lab02_sim /odom /ground_truth /cmd_vel /scan
```

Start the controller (Terminal 2). When it prints `goal reached`, press `Ctrl+C` in the third terminal. Check the recording:

```bash
ros2 bag info lab02_sim
```

✅ You see the 4 topics with a message count each. ❌ `already exists` → choose another name, e.g. `-o lab02_sim_2`.

Make the plots:

```bash
python3 ~/ros2_ws/src/SESASR-SIQUBro/scripts/plot_bag.py lab02_sim
```

✅ The PDF/PNG files are in `~/ros2_ws/bags/plots/`. Use the PDFs in the report.

### Part 6 — Real robot (Lab03 / Lab04)

**Before going to the lab:** know your **robot number `XX`** and **group number `NNN`**.

| # | Where | What to do | ✅ Check |
|---|---|---|---|
| 1 | Your PC | Connect to Wi-Fi `SESASR_WiFi` (password in the Lab03 document) | Internet icon connected |
| 2 | Your PC terminal | `ssh students@192.168.10.1XX` (password in the Lab03 document); answer `yes` the first time | Prompt becomes `students@nucXX` |
| 3 | Robot (SSH) | `nano ~/.bashrc`: check `ROS_DOMAIN_ID=XX` (no leading 0), `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, comment other groups' workspaces with `#`. Save: `Ctrl+O`, `Enter`, `Ctrl+X`. Then `source ~/.bashrc` | — |
| 4 | Robot (SSH) | `ros2 run rmw_zenoh_cpp rmw_zenohd` (keep it running; only one!) | No error |
| 5 | Robot (new SSH) | `ros2 launch turtlebot3_bringup robot.launch.py` | Log shows `LDS-02 started successfully` and `Run!` |
| 6 | Robot (new SSH) | `mkdir -p ~/group_NNN/src && cd ~/group_NNN/src && git clone https://github.com/Denvolatos/SESASR-SIQUBro.git && cd .. && colcon build --symlink-install && source install/setup.bash` | `1 package finished` |
| 7 | Desk | **Robot lifted, wheels in the air.** Power on; then in step 6's terminal: `ros2 run lab02_pkg bump_and_go --ros-args -p stop_at_goal:=false` | Wheels turn; `Ctrl+C` to stop |
| 8 | Test field | Put the robot down, start recording in another SSH: `ros2 bag record -o lab03_real /odom /cmd_vel /scan`, then run the controller again | Robot avoids walls |
| 9 | After the run | `Ctrl+C` the recorder, copy the bag to your PC (see below) | `ros2 bag info lab03_real` works |

If the robot PC has no internet (git clone fails), copy the code from your PC instead:

```bash
scp -r ~/ros2_ws/src/SESASR-SIQUBro students@192.168.10.1XX:~/group_NNN/src/
```

Copy the recording back to your PC (run on **your PC**):

```bash
scp -r students@192.168.10.1XX:~/group_NNN/bags/lab03_real ~/ros2_ws/bags/
```

(Adjust the path to where you started `ros2 bag record`.)

⚠️ Safety: stay next to the robot. `Ctrl+C` stops the program, but the robot may keep its last command — lift it if needed. If it gets too close to walls, add `-p obstacle_threshold:=0.6`.

### Part 7 — Optional: see the robot's laser in RViz on your PC

On your PC (same Wi-Fi), in a new terminal:

```bash
export ROS_DOMAIN_ID=XX
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.10.1XX:7447"]'
ros2 launch turtlebot3_bringup rviz2.launch.py
```

(In a virtual machine add `export LIBGL_ALWAYS_SOFTWARE=1` first.)

More problems? See the **Troubleshooting** table in the [README](../README.md).

---

## 🇨🇳 中文

本指南假设你**完全没有基础**。请按顺序做，每一步都写了**在哪里**敲、**敲什么**、**正常情况应该看到什么**。

### 第 0 部分：先认识五个词

| 词 | 意思 |
|---|---|
| **终端（Terminal）** | 输入命令的窗口。Ubuntu 里按 `Ctrl+Alt+T`，或在应用菜单搜 "Terminal"。敲完命令按 `Enter` 回车。 |
| **工作区（Workspace）** | 放 ROS 2 代码的文件夹，比如 `~/ros2_ws`，源代码放在它的 `src/` 子文件夹里。（`~` 表示你的家目录。） |
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
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Denvolatos/SESASR-SIQUBro.git
cd ~/ros2_ws
colcon build --symlink-install
```

✅ 最后一行显示 `Summary: 1 package finished`。

以后要拿到组员的最新修改：

```bash
cd ~/ros2_ws/src/SESASR-SIQUBro && git pull
cd ~/ros2_ws && colcon build --symlink-install
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

### 第 4 部分：运行仿真（每次）

需要开**两个终端**。

**终端 1：仿真世界**

```bash
source ~/ros_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_ignition lab02.launch.py
```

- Intel/AMD 电脑用 Gazebo Classic 的话，把 `turtlebot3_ignition` 换成 `turtlebot3_gazebo`。
- **在虚拟机里**（UTM、VirtualBox 等）先运行 `export LIBGL_ALWAYS_SOFTWARE=1`，否则 Gazebo 可能会崩溃。

✅ 大约 20 秒后弹出 Gazebo 窗口，能看到蓝色的迷宫、绿色方块和小车。

**终端 2：控制器**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run lab02_pkg bump_and_go
```

✅ 小车开始往前走，每遇到一面墙，终端打印一行，比如
`wall at 0.50 m (left 1.08, right 0.91) -> turn left`。
大约 4 分钟后显示 `goal reached at x=5.87 y=3.45`。

**想从头再跑一遍**：两个终端都按 `Ctrl+C`，等提示符 `$` 重新出现，再按顺序启动终端 1 和终端 2。

### 第 5 部分：录数据和画图

在仿真世界**已经打开**、控制器**还没启动**的时候，开**第三个终端**：

```bash
mkdir -p ~/ros2_ws/bags && cd ~/ros2_ws/bags
ros2 bag record -o lab02_sim /odom /ground_truth /cmd_vel /scan
```

然后启动控制器（终端 2）。看到 `goal reached` 后，在第三个终端按 `Ctrl+C`。检查录到的数据：

```bash
ros2 bag info lab02_sim
```

✅ 列出 4 个话题，每个都有消息条数。❌ 提示 `already exists` → 换个名字，比如 `-o lab02_sim_2`。

画图：

```bash
python3 ~/ros2_ws/src/SESASR-SIQUBro/scripts/plot_bag.py lab02_sim
```

✅ PDF/PNG 图片在 `~/ros2_ws/bags/plots/` 里，报告里用 PDF 版。

### 第 6 部分：真车（Lab03 / Lab04）

**去实验室之前**：问清楚你们的**机器人编号 `XX`** 和**组号 `NNN`**。

| # | 在哪里 | 做什么 | ✅ 检查 |
|---|---|---|---|
| 1 | 自己电脑 | 连 WiFi `SESASR_WiFi`（密码见 Lab03 文档） | 网络已连接 |
| 2 | 自己电脑的终端 | `ssh students@192.168.10.1XX`（密码见 Lab03 文档），第一次连接时回答 `yes` | 提示符变成 `students@nucXX` |
| 3 | 机器人（SSH） | `nano ~/.bashrc`：检查 `ROS_DOMAIN_ID=XX`（前面不带 0）、`RMW_IMPLEMENTATION=rmw_zenoh_cpp`，其他组的工作区那几行前面加 `#` 注释掉。保存：`Ctrl+O`、回车、`Ctrl+X`。然后 `source ~/.bashrc` | — |
| 4 | 机器人（SSH） | `ros2 run rmw_zenoh_cpp rmw_zenohd`（保持运行，只开一个！） | 没有报错 |
| 5 | 机器人（新开 SSH） | `ros2 launch turtlebot3_bringup robot.launch.py` | 日志里有 `LDS-02 started successfully` 和 `Run!` |
| 6 | 机器人（新开 SSH） | `mkdir -p ~/group_NNN/src && cd ~/group_NNN/src && git clone https://github.com/Denvolatos/SESASR-SIQUBro.git && cd .. && colcon build --symlink-install && source install/setup.bash` | `1 package finished` |
| 7 | 桌上 | **把车架空，轮子离开桌面**。打开电源，在第 6 步的终端里运行：`ros2 run lab02_pkg bump_and_go --ros-args -p stop_at_goal:=false` | 轮子会转；`Ctrl+C` 停止 |
| 8 | 场地上 | 把车放到地上，另开一个 SSH 开始录：`ros2 bag record -o lab03_real /odom /cmd_vel /scan`，再运行一次控制器 | 车能自己绕开墙 |
| 9 | 跑完之后 | 录制终端按 `Ctrl+C`，把录像拷回自己电脑（见下面） | `ros2 bag info lab03_real` 正常 |

如果机器人电脑上不了网（git clone 失败），就从自己电脑把代码拷过去：

```bash
scp -r ~/ros2_ws/src/SESASR-SIQUBro students@192.168.10.1XX:~/group_NNN/src/
```

把录像拷回自己电脑（在**自己电脑**上运行）：

```bash
scp -r students@192.168.10.1XX:~/group_NNN/bags/lab03_real ~/ros2_ws/bags/
```

（路径改成你运行 `ros2 bag record` 时所在的文件夹。）

⚠️ 安全：人要站在车旁边。`Ctrl+C` 会停掉程序，但车可能还会按最后一条指令继续动，必要时直接把车拿起来。车离墙太近才转弯的话，加 `-p obstacle_threshold:=0.6`。

### 第 7 部分（可选）：在自己电脑上用 RViz 看真车的激光

自己电脑（连同一个 WiFi），新开一个终端：

```bash
export ROS_DOMAIN_ID=XX
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.10.1XX:7447"]'
ros2 launch turtlebot3_bringup rviz2.launch.py
```

（在虚拟机里要先加 `export LIBGL_ALWAYS_SOFTWARE=1`。）

还有其他问题？看 [README](../README.md) 里的**常见问题**表。
