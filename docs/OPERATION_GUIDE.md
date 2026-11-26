# 🚀 Operation Guide (step by step, for beginners) / 操作流程（零基础版）

**Language / 语言：** [🇬🇧 English](#-english) · [🇨🇳 中文](#-中文) · [⬅️ README](../README.md)

---

## 🇬🇧 English

**Jump to:** [Basics & Setup (Parts 0–3)](#part-0--five-words-you-need-to-know) · [R01 Bump&Go (Parts 4–10)](#part-4--r01-run-the-simulation-every-time) · [R02 EKF (Part 11)](#part-11--r02-ekf-localization-lab04_pkg) · [R03 DWA (Part 12)](#part-12--r03-dwa-obstacle-avoidance-and-following-lab07_pkg)

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

✅ The last line says `Summary: 3 packages finished` (all three report packages: `lab02_pkg`, `lab04_pkg`, `lab07_pkg`). If `landmark_msgs` is not installed yet, build a single package with `colcon build --packages-select lab02_pkg --symlink-install` (or build `turtlebot3_perception` first in Part 3).

If you work from a fork, replace the clone URL with your fork URL. If you already cloned the repository, keep that checkout and run the build commands inside it. The Git repository itself is the workspace root; `src/` contains the ROS packages.

Later, to get your teammates' latest changes:

```bash
cd ~/SESASR-SIQUBro
git pull
colcon build --symlink-install
```

### Part 3 — Prepare the simulation (once)

The simulation worlds and perception packages come from the course repositories. Install them in a **separate** workspace `~/ros_ws`:

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone https://github.com/SESASR-Course/turtlebot3_simulations.git
git clone https://github.com/SESASR-Course/turtlebot3_perception.git
```

(`turtlebot3_perception` provides `landmark_msgs` and the simulated/real landmark nodes needed by R02 `lab04_pkg` and R03 `lab07_pkg`.)

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

If the course simulation is already built in another workspace (for example `~/ros2_ws`), use that workspace in Terminal 1 below. Do not source a workspace containing another copy of `lab02_pkg`, `lab04_pkg` or `lab07_pkg` in the controller terminal.

### Part 4 — R01: Run the simulation (every time)

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

### Part 5 — R01: Record data and make plots

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

### Part 6 — R01: Real robot (Lab03 / Lab04)

**Before going to the lab:** know your **robot number `XX`** (from the lab handout, together with the robot address `<robot-ip>`) and **group number `NNN`** (for our group: `203`).

| # | Where | What to do | ✅ Check |
|---|---|---|---|
| 1 | Your PC | Connect to Wi-Fi `SESASR_WiFi` (credentials in the lab handout) | Internet icon connected |
| 2 | Your PC terminal | `ssh students@<robot-ip>` (credentials in the lab handout); answer `yes` the first time | Prompt becomes `students@nucXX` |
| 3 | Robot (SSH) | `nano ~/.bashrc`: check `ROS_DOMAIN_ID=XX` (no leading 0), `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, comment other groups' workspaces with `#`. Save: `Ctrl+O`, `Enter`, `Ctrl+X`. Then `source ~/.bashrc`. If another group's workspace stays in `.bashrc` (it may also contain a `lab02_pkg`), run `source ~/group_NNN/install/setup.bash` in every new terminal | — |
| 4 | Robot (SSH) | `ros2 run rmw_zenoh_cpp rmw_zenohd` (keep it running; only one!) | No error |
| 5 | Robot (new SSH) | `ros2 launch turtlebot3_bringup robot.launch.py` | Log shows `LDS-02 started successfully` and `Run!` |
| 6 | Robot (new SSH) | `mkdir -p ~/group_NNN/src && cd ~/group_NNN/src && git clone https://github.com/Denvolatos/SESASR-SIQUBro.git && cd .. && colcon build --symlink-install && source install/setup.bash` | Packages finished |
| 7 | Desk | **Robot lifted, wheels in the air.** Power on; then in step 6's terminal: `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"` | Wheels turn; `Ctrl+C` to stop. (Do not test the controller lifted: the yaw comes from the IMU, so a turn never finishes.) |
| 8 | Test field | Put the robot down, start recording in another SSH: `ros2 bag record -o lab04_real /odom /cmd_vel /scan /imu`, then in step 6's terminal: `ros2 launch lab02_pkg bump_and_go.launch.py robot:=real` | Robot avoids walls |
| 9 | After the run | `Ctrl+C` the recorder, copy the bag to your PC (see below) | `ros2 bag info lab03_real` works |

If the robot PC has no internet (git clone fails), copy the code from your PC instead:

```bash
scp -r ~/SESASR-SIQUBro/src/lab02_pkg students@<robot-ip>:~/group_NNN/src/
```

Copy the recording back to your PC (run on **your PC**):

```bash
scp -r students@<robot-ip>:~/group_NNN/bags/lab03_real ~/SESASR-SIQUBro/bags/
```

(Adjust the path to where you started `ros2 bag record`.)

⚠️ Safety: stay next to the robot. `Ctrl+C` stops the program, but the robot may keep its last command — lift it if needed. Emergency stop from any terminal: `pkill -f "lib/lab02_pkg/bump_and_go"; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`. To change speed, turn angle or threshold, edit `src/lab02_pkg/config/real_robot.yaml` and run `colcon build` again.

### Part 7 — Optional: see the robot's laser in RViz on your PC

On your PC (same Wi-Fi), in a new terminal:

```bash
export ROS_DOMAIN_ID=XX
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<robot-ip>:7447"]'
ros2 launch turtlebot3_bringup rviz2.launch.py
```

(In a virtual machine add `export LIBGL_ALWAYS_SOFTWARE=1` first.)

### Part 8 — R01: Troubleshooting

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

### Part 10 — R01: Real-robot debugging notes and Lab04 checklist

What we learned on 22/10 (robot 3) and in simulation tests:

| Test (simulation, assumed dead zone of 0.25 rad/s added with `scripts/deadzone_filter.py`) | Result |
|---|---|
| A: old parameters (2° tolerance, no minimum ω) | Drives 4 m, then **stuck forever in the first turn**: the P controller asks for ω≈0.22 rad/s, inside the dead zone |
| B: new parameters (5° tolerance, minimum ω 0.3) | **Reaches the goal: 10 turns, 225 s** |
| C: extra node publishing zero velocity | Still drives forward → not the cause of "not moving at start" |
| D: obstacle in front at start | Turns immediately, then stuck → does not match what we saw |

Conclusion: tests A and B show what would happen if the motors had such a dead zone, so the minimum ω is a precaution; the dead zone was not measured, and the real-robot bags do not show this stall. Tests C and D were run with the code of 23/10. "Not moving" on the real robot matches the 0.22 m/s problem below; its root cause was not found.

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
3. Test the motors alone (robot lifted): `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"` (not 0.22, see above)
4. Run the controller and read its status line every 2 s (`state=... v=... odom=...`).
5. In another terminal: `ros2 topic info /cmd_vel` → `Publisher count: 1`, `Subscription count: 1`.
6. If bringup prints `There is no status packet` / `stack smashing detected`: power the motor board off for 10 s, on again (wait for the music), restart bringup.
7. From the repository root, test runner: `SIM_WS=~/ros_ws scripts/run_dz_test.sh <label> <tolerance_deg> <min_w> <seconds> [none|zero|box]` (simulation, Gazebo Classic; set `SIM_WS=~/ros2_ws` if that is your simulation workspace).

---

### Part 11 — R02: EKF localization (`lab04_pkg`)

`lab04_pkg` estimates the robot pose with an Extended Kalman Filter at 20 Hz, publishing `/ekf` (`nav_msgs/Odometry`). Make sure `turtlebot3_perception` (providing `landmark_msgs`) is built and sourced in `~/ros_ws` (Part 3).

#### 11.1 Simulation (Task 0, Task 1, Task 2)

**Task 0 (offline plots & Jacobians, no ROS needed):**

```bash
cd ~/SESASR-SIQUBro/src/lab04_pkg
python3 scripts/task0_models.py /tmp/ekf_task0
```

✅ Saves PDF/PNG plots of the sampled motion and landmark models and checks Jacobians with `sympy`.

**Task 1 (`[x, y, θ]`) and Task 2 (`[x, y, θ, v, ω]` with `fusion:=true`) in Gazebo:**

You need **three terminals** (plus a fourth if recording a bag).

**Terminal 1 — the Lab04 world (with landmark simulator):**

```bash
source ~/ros_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_ignition lab04.launch.py
```

(In a VM add `export LIBGL_ALWAYS_SOFTWARE=1` first; on Gazebo Classic use `turtlebot3_gazebo`.)

**Terminal 2 — the EKF node:**

```bash
source ~/ros_ws/install/setup.bash
source ~/SESASR-SIQUBro/install/setup.bash
ros2 launch lab04_pkg ekf.launch.py                  # Task 1: state [x, y, θ]
# or:
ros2 launch lab04_pkg ekf.launch.py fusion:=true     # Task 2: state [x, y, θ, v, ω]
```

✅ Terminal prints `EKF started: state size 3, 9 landmarks` (or `state size 5`).

**Terminal 3 — move the robot (Bump&Go or teleop):**

```bash
source ~/SESASR-SIQUBro/install/setup.bash
ros2 run lab02_pkg bump_and_go --ros-args \
  --params-file ~/SESASR-SIQUBro/install/lab02_pkg/share/lab02_pkg/config/sim.yaml \
  -p stop_at_goal:=false -p use_sim_time:=true
```

**Optional Terminal 4 — record & evaluate:**

```bash
mkdir -p ~/SESASR-SIQUBro/bags && cd ~/SESASR-SIQUBro/bags
ros2 bag record -o ekf_sim_t1 /ground_truth /odom /ekf /landmarks /cmd_vel /imu
```

After stopping with `Ctrl+C`, compute RMSE/MAE and plots:

```bash
python3 ~/SESASR-SIQUBro/src/lab04_pkg/scripts/evaluate_ekf.py \
  ~/SESASR-SIQUBro/bags/ekf_sim_t1 ~/SESASR-SIQUBro/bags/ekf_plots "Task 1"
```

Compare Task 1 and Task 2 on the **same** recorded bag:

```bash
python3 ~/SESASR-SIQUBro/src/lab04_pkg/scripts/replay_filters.py \
  ~/SESASR-SIQUBro/bags/ekf_sim_t1 ~/SESASR-SIQUBro/src/lab04_pkg/config/sim.yaml
```

There is also an automated 150 s simulation runner:

```bash
bash ~/SESASR-SIQUBro/src/lab04_pkg/scripts/run_sim_experiment.sh sim_t1 false 150
bash ~/SESASR-SIQUBro/src/lab04_pkg/scripts/run_sim_experiment.sh sim_t2 true 150
```

Reference simulation results (150 s run): position RMSE vs `/ground_truth` dropped from **0.018 m** (`/odom`) to **0.010 m** (`/ekf`) in Task 1, and from **0.020 m** to **0.010 m** in Task 2. On the same recorded data, Task 2 reduced heading error by ~20–30 % without changing position RMSE.

#### 11.2 Real robot (Task 3)

1. **Copy `lab04_pkg` to the robot PC** (no internet on the robot):
   ```bash
   scp -r ~/SESASR-SIQUBro/src/lab04_pkg students@<robot-ip>:~/group_203/src/
   ```
2. **Start the 4 robot base launches** (4 separate SSH terminals on the robot):
   - Terminal 1: `ros2 run rmw_zenoh_cpp rmw_zenohd`
   - Terminal 2: `ros2 launch turtlebot3_bringup robot.launch.py`
   - Terminal 3: `ros2 launch turtlebot3_perception camera.launch.py`
   - Terminal 4: `ros2 launch turtlebot3_perception apriltag.launch.py`
3. **Prepare today's landmark file:** the AprilTags are moved every lab session. Put the `id x y` coordinates of the day into `~/tag_positions.txt` on the robot (or a YAML file matching `config/landmarks_thu1211.yaml`).
4. **Place the robot where the camera sees at least 2 tags** (with a rough start pose and only 1 tag in view, localization has two geometric solutions) and measure its start pose `X Y THETA` [m, m, rad] in the map frame.
5. **Run the helper script from `~/group_203`** (Terminal 5):
   ```bash
   cd ~/group_203
   LANDMARKS=$HOME/tag_positions.txt bash src/lab04_pkg/scripts/robot_task3.sh run1 X Y THETA
   # add "fusion" at the end for Task 2:
   LANDMARKS=$HOME/tag_positions.txt bash src/lab04_pkg/scripts/robot_task3.sh run2 X Y THETA fusion
   ```
   The script verifies `/odom` and `/camera/landmarks`, builds `lab04_pkg`, converts `tag_positions.txt` to YAML, starts `ekf_node`, and records `/odom /imu /scan /camera/landmarks /tf /tf_static /cmd_vel /ekf` into `~/g203_bags/<RUN_NAME>`. Drive the robot (teleop or Bump&Go), then press `Ctrl+C` to stop cleanly. Even if the EKF fails to start, the script waits 10 s and still records the raw topics for offline replay.
6. **Copy the bag back** (on your laptop):
   ```bash
   scp -r students@<robot-ip>:~/g203_bags/run1 ~/SESASR-SIQUBro/bags/
   ```
7. **Replay a real-robot bag on your PC** (re-run the EKF offline with different noise settings):
   ```bash
   # Terminal 1
   ros2 bag play --clock 1000 --remap /ekf:=/old_ekf -- ~/SESASR-SIQUBro/bags/run1
   # Terminal 2
   source ~/SESASR-SIQUBro/install/setup.bash
   ros2 launch lab04_pkg ekf.launch.py robot:=real use_sim_time:=true
   ```

Reference real-robot run (19 Nov, robot 3): a 172 s drive observed 7 of the 9 tags, and `/ekf` and `/odom` ended **0.28 m** apart (there is no ground truth on the real robot, so this shows agreement between the estimates and the tags rather than absolute accuracy). Remember: changing the battery reboots the robot PC, so all four base launches must be restarted.

---

### Part 12 — R03: DWA obstacle avoidance and following (`lab07_pkg`)

`lab07_pkg` runs a 15 Hz Dynamic Window Approach (DWA) controller (`dwa_node`). It supports:
- **Task 1 (`mode:=goal`)**: navigate to a static goal (`/goal_pose` or one-shot AprilTag) while avoiding obstacles.
- **Task 2 (`mode:=follow`)**: follow a moving target (`/dynamic_goal_pose` in simulation, `/camera/landmarks` on the real robot) at ~0.8 m.
- Status results are published on `/nav_result` (`Goal`, `Collision`, `Timeout`, `Lost`) and remaining distance on `/distance_to_goal` every 50 steps.

#### 12.1 Simulation (Task 1 & Task 2)

**Terminal 1 — the DWA maze world (`hard_maze.world`):**

```bash
source ~/ros_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_ignition project.launch.py
```

(In a VM add `export LIBGL_ALWAYS_SOFTWARE=1` first; on Gazebo Classic use `turtlebot3_gazebo`.)

**Terminal 2 — the DWA node:**

```bash
source ~/SESASR-SIQUBro/install/setup.bash
ros2 launch lab07_pkg dwa.launch.py                  # Task 1: static goal
# or:
ros2 launch lab07_pkg dwa.launch.py mode:=follow     # Task 2: follow red cube
```

**Terminal 3 — send a static goal (Task 1 only):**

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 1.5, y: -0.5}}}"
```

✅ Terminal 2 prints `Accepted goal`, logs distance every 50 steps, and ends with `Navigation finished with result: Goal`.

**Record & evaluate a bag:**

```bash
# Start recording before sending the goal (or before starting follow mode)
mkdir -p ~/SESASR-SIQUBro/bags && cd ~/SESASR-SIQUBro/bags
ros2 bag record -o r03_sim_goal /ground_truth /odom /scan /cmd_vel /goal_pose /dynamic_goal_pose /nav_result /distance_to_goal

# Evaluate (generates metrics + trajectory/velocity plots)
python3 ~/SESASR-SIQUBro/src/lab07_pkg/scripts/evaluate_nav.py \
  ~/SESASR-SIQUBro/bags/r03_sim_goal --robot sim --mode goal --out-dir ~/SESASR-SIQUBro/bags/eval_sim
```

(For Task 2 pass `--mode follow`.)

**Automated repeated Gazebo trials (Task 1):**

```bash
bash ~/SESASR-SIQUBro/src/lab07_pkg/scripts/run_goal_trials.sh                # 8 goals × 3 runs
TRIALS=1 bash ~/SESASR-SIQUBro/src/lab07_pkg/scripts/run_goal_trials.sh       # quick 1-run check
INCLUDE_HARD=1 bash ~/SESASR-SIQUBro/src/lab07_pkg/scripts/run_goal_trials.sh # include 4 local-min goals
```

Reference simulation results (`hard_maze.world`):
- **Task 1**: 8 reachable goals × 3 runs = **24 / 24 reached, no collision** (4 goals behind the central walls end safely in `Timeout`, a known local-minimum limit of pure DWA).
- **Task 2**: **3 / 3 runs followed the whole first lap, no collision** (mean distance 2.0–2.8 m, max 3.5–4.9 m where the red cube cuts 0.115 m from a wall corner and the robot catches up afterwards).

#### 12.2 Real robot (Task 3 — self-sufficient runbook)

Follow these steps in order — no extra help needed.

**⚠️ Safety rules before you press Enter**
1. **Lift first, drive second:** on the very first run, lift the robot (wheels off the ground) and confirm wheel direction and `Ctrl+C` stop before putting it on the floor.
2. **Always stand beside the robot** so you can lift it immediately if needed.
3. **Real-robot limits:** `max_lin_vel = 0.15 m/s`, emergency stop radius = `0.305 m` (`0.105 + 0.20`), DWA planning radius = `0.405 m` (`+0.10 m` margin), 3-point median LiDAR filter enabled.
4. **Stop anytime:** press `Ctrl+C` in the DWA terminal (the node publishes zero velocity on exit). Hard emergency stop from any terminal:
   ```bash
   pkill -f "lib/lab07_pkg/dwa_node"; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
   ```
5. **Battery:** make sure the battery is charged before starting (swapping the battery reboots the robot PC and requires restarting Terminals 1–4).

**Step 1 — Copy code to `~/group_203/src` and build (robot PC has no internet)**

On **your laptop** (connected to `SESASR_WiFi`):

```bash
scp -r ~/SESASR-SIQUBro/src/lab07_pkg students@<robot-ip>:~/group_203/src/
```

On the **robot (SSH)**:

```bash
cd ~/group_203
source /opt/ros/humble/setup.bash
colcon build --packages-select lab07_pkg --symlink-install
source install/setup.bash
```

✅ Last line says `Summary: 1 package finished`.

**Step 2 — Start the robot base stack (4 SSH terminals on the robot)**

| Terminal | Command | ✅ Wait until you see |
|---|---|---|
| 1 | `ros2 run rmw_zenoh_cpp rmw_zenohd` | Running without error (keep only one!) |
| 2 | `ros2 launch turtlebot3_bringup robot.launch.py` | `LDS-02 started successfully` and `Run!` |
| 3 | `ros2 launch turtlebot3_perception camera.launch.py` | Camera streaming (needed for AprilTag goal or follow) |
| 4 | `ros2 launch turtlebot3_perception apriltag.launch.py` | Publishing `/camera/landmarks` (needed for AprilTag goal or follow) |

> **Start pose & coordinate frame (important for static goals!):** `/odom` sets `(0, 0)` and `+x` (forward) at **wherever the robot stands when `robot.launch.py` starts**. Place the robot on the start mark, **restart Terminal 2 (`robot.launch.py`)**, then measure the goal's `(X, Y)` in meters relative to that start pose (`+x` straight ahead, `+y` left).

**Step 3A — Run static goal by coordinates (`mode:=goal`, default `goal_source:=topic`)**

- **Terminal 5 (DWA):**
  ```bash
  source ~/group_203/install/setup.bash
  ros2 launch lab07_pkg dwa.launch.py robot:=real mode:=goal
  ```
- **Terminal 6 (record bag into `~/group_203/bags`, change `r03_goal1` each run):**
  ```bash
  mkdir -p ~/group_203/bags
  ros2 bag record -o ~/group_203/bags/r03_goal1 \
    /odom /scan /camera/landmarks /cmd_vel /goal_pose /nav_result /distance_to_goal /tf /tf_static
  ```
- **Terminal 7 (send goal in meters in `odom` frame):**
  ```bash
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: odom}, pose: {position: {x: 1.5, y: 0.0}}}"
  ```
- Wait for `Goal` (or `Collision` / `Timeout`) in Terminal 5, then press `Ctrl+C` in Terminal 6 (bag recorder) and verify `~/group_203/bags/r03_goal1/metadata.yaml` exists. For the next goal, place the robot back at the start mark and restart Terminal 2 (`robot.launch.py`).

**Step 3B — Run static goal using a stationary AprilTag (`goal_source:=landmark`)**

If the static goal is marked with a stationary AprilTag of ID `<N>`:

- **Terminal 5 (DWA):**
  ```bash
  source ~/group_203/install/setup.bash
  ros2 launch lab07_pkg dwa.launch.py robot:=real mode:=goal goal_source:=landmark target_id:=<N>
  ```
- **Terminal 6 (record bag):** same `ros2 bag record` command as Step 3A (with a new bag name).
- Point the robot so the camera sees tag `<N>`. On first detection, the node locks a single static goal **`landmark_goal_standoff` (`0.5 m`) in front of the tag** along the robot→tag line so the robot reaches `Goal` cleanly without hitting the `0.305 m` emergency stop.

**Step 3C — Run dynamic target following (`mode:=follow`)**

- **Terminal 5 (DWA — `target_id` is mandatory on the real robot!):**
  ```bash
  source ~/group_203/install/setup.bash
  ros2 launch lab07_pkg dwa.launch.py robot:=real mode:=follow target_id:=<N>
  ```
  (`real_robot.yaml` sets `require_target_id: true` so fixed arena tags cannot hijack the robot.)
- **Terminal 6 (record bag):**
  ```bash
  mkdir -p ~/group_203/bags
  ros2 bag record -o ~/group_203/bags/r03_follow1 \
    /odom /scan /camera/landmarks /cmd_vel /nav_result /distance_to_goal /tf /tf_static
  ```

**How the person holding the AprilTag should walk (read before running!)**

| Guideline | Why it matters |
|---|---|
| Hold the tag upright at **robot camera height**, facing the lens | Keeps `/camera/landmarks` detections steady without dropouts |
| Start **0.8–1.0 m** directly in front of the robot | Matches `desired_target_dist = 0.8 m` and stays well outside the `0.305 m` emergency stop |
| Walk **slowly at 0.10–0.12 m/s** | Real robot `max_lin_vel` is `0.15 m/s`; faster walking leaves the robot behind |
| Make **wide, gentle turns** (avoid sharp 90° pivots right in front of the robot) | Real camera horizontal half-FoV is only **±33°** (`0.576 rad`); if the tag leaves the view for `> 2.0 s`, the robot stops and waits until the tag re-enters |
| Never step backward into the robot or stop closer than **0.6 m** | The LiDAR sees your legs as an obstacle; getting within `0.305 m` triggers a locked `Collision` stop |
| If the robot stops on `Collision`, clear the area and call `/reset_nav` | `ros2 service call /reset_nav std_srvs/srv/Trigger` unlocks navigation safely |

**Step 4 — Copy bags back and run evaluation**

On **your laptop** (before leaving the lab, check every bag has `metadata.yaml`):

```bash
scp -r students@<robot-ip>:~/group_203/bags/r03_goal1 ~/SESASR-SIQUBro/bags/
```

Run evaluation on your Ubuntu VM / PC:

```bash
# Static goals (pass 1 or more bags)
python3 ~/SESASR-SIQUBro/src/lab07_pkg/scripts/evaluate_nav.py \
  ~/SESASR-SIQUBro/bags/r03_goal1 ~/SESASR-SIQUBro/bags/r03_goal2 ~/SESASR-SIQUBro/bags/r03_goal3 \
  --robot real --mode goal --out-dir ~/SESASR-SIQUBro/bags/eval_real_goal

# Dynamic following (pass the tag ID you followed)
python3 ~/SESASR-SIQUBro/src/lab07_pkg/scripts/evaluate_nav.py \
  ~/SESASR-SIQUBro/bags/r03_follow1 --robot real --mode follow --target-id <N> \
  --out-dir ~/SESASR-SIQUBro/bags/eval_real_follow
```

#### 12.3 R03 Troubleshooting lookup table

| Symptom | Likely cause → Fix |
|---|---|
| Immediate `Collision` right after launch | Something (wall, box, shoes) is within `0.305 m` of robot center, or `robot:=real` was omitted (sim `nan_policy: min` treats LDS-02 `NaN` as 0.05 m) → verify `robot:=real`, move to a clearer spot |
| `Too few valid LiDAR rays` or `LiDAR NaN ratio ... exceeds max_nan_ratio` | LiDAR stopped or returning all `0.0` / all `NaN` → check `robot.launch.py` and `ros2 topic echo /scan --once` |
| Follow mode refuses to start (`require_target_id`) | Forgot `target_id:=<N>` → launch with `ros2 launch lab07_pkg dwa.launch.py robot:=real mode:=follow target_id:=<N>` |
| Ignores new goals after a `Collision` | Intentional collision lock so the robot cannot self-restart → clear obstacles and run `ros2 service call /reset_nav std_srvs/srv/Trigger` |
| Stands still (`All candidate trajectories collide`) | Start pose is inside the `0.405 m` planning radius of a wall, or goal is directly behind an obstacle (local minimum) → move robot farther from the wall, pick a reachable goal, or check `planning_margin` below |
| Wheels hum but robot barely creeps forward | Static friction or low battery → confirm `robot:=real` (`window_source: "last_cmd"` is already set in `real_robot.yaml`), check battery |
| Robot stops during follow (`Target lost > 2.0 s`) | AprilTag left the ±33° camera FoV or was occluded → bring the tag back into view; the robot resumes automatically |
| `/nav_result` publishes `Lost` during follow | Distance stayed `> 3.0 m` for 10 s → notification only; the robot keeps chasing and re-arms once back within 3.0 m |
| Static AprilTag goal stops ~0.5 m before the tag | Expected behavior (`landmark_goal_standoff = 0.5 m`) to stop safely outside the `0.305 m` hard-stop radius |

#### 12.4 Parameters most likely to tune on site (`src/lab07_pkg/config/real_robot.yaml`)

With `--symlink-install`, edits to `~/group_203/src/lab07_pkg/config/real_robot.yaml` apply on the next `ros2 launch` (if in doubt, run `colcon build --packages-select lab07_pkg --symlink-install`).

| Parameter | Default (`real_robot.yaml`) | When & how to tune on site |
|---|---|---|
| `planning_margin` | `0.10 m` (planning `0.405 m`) | The real arena is narrower than Gazebo; if the robot refuses to pass through a gap, reduce to `0.06–0.08 m` (must stay `> band_floor_buffer = 0.03 m`) |
| `collision_tol` | `0.20 m` (stop `0.305 m`) | Lab guide allows `0.15–0.25 m`; lower to `0.16–0.18 m` only if narrow passages require it |
| `max_lin_vel` | `0.15 m/s` | Lower to `0.12 m/s` in tight cluttered spaces; **do not** raise to `0.22 m/s` (stalled forward motion on 10/29) |
| `desired_target_dist` | `0.8 m` | Increase to `0.9–1.0 m` if the robot brakes too early when seeing the tag holder's legs |
| `landmark_goal_standoff` | `0.5 m` | Adjust for static AprilTag goals (must stay `> robot_radius + collision_tol = 0.305 m`) |
| `scan_median_filter` | `true` | Filters single-ray LDS-02 spikes; keep `true` unless testing very thin poles (`< 3 cm`) |
| `window_source` | `"last_cmd"` | Prevents static-friction stall at low initial speeds; keep `"last_cmd"` on the real robot |

---

## 🇨🇳 中文

**快速跳转：** [基础与准备（第 0–3 部分）](#第-0-部分先认识五个词) · [R01 Bump&Go（第 4–10 部分）](#第-4-部分r01-运行仿真每次) · [R02 EKF（第 11 部分）](#第-11-部分r02-ekf-定位lab04_pkg) · [R03 DWA（第 12 部分）](#第-12-部分r03-dwa-避障与目标跟随lab07_pkg)

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

✅ 最后一行显示 `Summary: 3 packages finished`（包含三份报告的包：`lab02_pkg`、`lab04_pkg`、`lab07_pkg`）。如果还没装 `landmark_msgs`，可先用 `colcon build --packages-select lab02_pkg --symlink-install` 单编，或先按第 3 部分编译 `turtlebot3_perception`。

如果你使用自己的 fork，把克隆 URL 换成 fork 地址。已经克隆过的，保留现有目录，直接在仓库根目录编译。Git 仓库本身就是工作区根目录，ROS 包放在 `src/`。

以后要拿到组员的最新修改：

```bash
cd ~/SESASR-SIQUBro
git pull
colcon build --symlink-install
```

### 第 3 部分：准备仿真环境（只做一次）

仿真世界和感知包来自课程提供的仓库，装在**另一个**工作区 `~/ros_ws` 里：

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone https://github.com/SESASR-Course/turtlebot3_simulations.git
git clone https://github.com/SESASR-Course/turtlebot3_perception.git
```

（`turtlebot3_perception` 提供 `landmark_msgs` 以及仿真/真车地标检测节点，R02 `lab04_pkg` 和 R03 `lab07_pkg` 都会用到。）

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

如果课程仿真已经编译在其他工作区（例如 `~/ros2_ws`），下面的终端 1 改为加载那个工作区。运行控制器的终端不要加载含有另一份 `lab02_pkg`、`lab04_pkg` 或 `lab07_pkg` 的工作区。

### 第 4 部分：R01 运行仿真（每次）

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

### 第 5 部分：R01 录数据和画图

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

### 第 6 部分：R01 真车（Lab03 / Lab04）

**去实验室之前**：问清楚你们的**机器人编号 `XX`**（连同机器人的地址 `<robot-ip>`，见实验指导书）和**组号 `NNN`**（我们组是 `203`）。

| # | 在哪里 | 做什么 | ✅ 检查 |
|---|---|---|---|
| 1 | 自己电脑 | 连 WiFi `SESASR_WiFi`（账号密码见实验指导书） | 网络已连接 |
| 2 | 自己电脑的终端 | `ssh students@<robot-ip>`（密码见实验指导书），第一次连接时回答 `yes` | 提示符变成 `students@nucXX` |
| 3 | 机器人（SSH） | `nano ~/.bashrc`：检查 `ROS_DOMAIN_ID=XX`（前面不带 0）、`RMW_IMPLEMENTATION=rmw_zenoh_cpp`，其他组的工作区那几行前面加 `#` 注释掉。保存：`Ctrl+O`、回车、`Ctrl+X`。然后 `source ~/.bashrc`。如果 `.bashrc` 里还留着别的组的工作区（里面可能也有 `lab02_pkg`），每个新终端都先运行 `source ~/group_NNN/install/setup.bash` | — |
| 4 | 机器人（SSH） | `ros2 run rmw_zenoh_cpp rmw_zenohd`（保持运行，只开一个！） | 没有报错 |
| 5 | 机器人（新开 SSH） | `ros2 launch turtlebot3_bringup robot.launch.py` | 日志里有 `LDS-02 started successfully` 和 `Run!` |
| 6 | 机器人（新开 SSH） | `mkdir -p ~/group_NNN/src && cd ~/group_NNN/src && git clone https://github.com/Denvolatos/SESASR-SIQUBro.git && cd .. && colcon build --symlink-install && source install/setup.bash` | 编译通过 |
| 7 | 桌上 | **把车架空，轮子离开桌面**。打开电源，在第 6 步的终端里运行：`ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"` | 轮子会转；`Ctrl+C` 停止。（不要架空测控制器：朝向来自 IMU，车身不转，转弯永远转不完。） |
| 8 | 场地上 | 把车放到地上，另开一个 SSH 开始录：`ros2 bag record -o lab04_real /odom /cmd_vel /scan /imu`，再在第 6 步的终端里运行：`ros2 launch lab02_pkg bump_and_go.launch.py robot:=real` | 车能自己绕开墙 |
| 9 | 跑完之后 | 录制终端按 `Ctrl+C`，把录像拷回自己电脑（见下面） | `ros2 bag info lab03_real` 正常 |

如果机器人电脑上不了网（git clone 失败），就从自己电脑把代码拷过去：

```bash
scp -r ~/SESASR-SIQUBro/src/lab02_pkg students@<robot-ip>:~/group_NNN/src/
```

把录像拷回自己电脑（在**自己电脑**上运行）：

```bash
scp -r students@<robot-ip>:~/group_NNN/bags/lab03_real ~/SESASR-SIQUBro/bags/
```

（路径改成你运行 `ros2 bag record` 时所在的文件夹。）

⚠️ 安全：人要站在车旁边。`Ctrl+C` 会停掉程序，但车可能还会按最后一条指令继续动，必要时直接把车拿起来。任何终端都能急停：`pkill -f "lib/lab02_pkg/bump_and_go"; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"`。要改速度、转弯角度或阈值，就改 `src/lab02_pkg/config/real_robot.yaml`，然后重新 `colcon build`。

### 第 7 部分（可选）：在自己电脑上用 RViz 看真车的激光

自己电脑（连同一个 WiFi），新开一个终端：

```bash
export ROS_DOMAIN_ID=XX
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<robot-ip>:7447"]'
ros2 launch turtlebot3_bringup rviz2.launch.py
```

（在虚拟机里要先加 `export LIBGL_ALWAYS_SOFTWARE=1`。）

### 第 8 部分：R01 常见问题

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

### 第 10 部分：R01 真车排障记录和 Lab04 检查清单

10/22（3 号机器人）和之后仿真测试的结论：

| 测试（仿真，用 `scripts/deadzone_filter.py` 加入假设的 0.25 rad/s 死区） | 结果 |
|---|---|
| A：旧参数（容差 2°、没有最小转速） | 走了 4 m 后**卡在第一个弯**：P 控制给出的转速约 0.22 rad/s，落在死区里 |
| B：新参数（容差 5°、最小转速 0.3） | **到达终点：10 次转向，225 秒** |
| C：另有程序一直发零速度 | 车照样往前走，不是「一开始不动」的原因 |
| D：开局车前就有障碍 | 立刻转弯后卡住，和当天现象不符 |

结论：测试 A、B 说明的是「如果电机有这样的死区会怎样」，所以最小转速是预防措施；死区没有实测过，真车 bag 里也没有这种卡住。测试 C、D 用的是 10/23 的代码。真车「不走」和下面 0.22 m/s 的问题相符，根本原因没有查明。

**Lab04 结果（10/29，3 号机器人）**，现在都已经是 `config/real_robot.yaml` 的默认值：

| 发现 | 证据 | 解决 |
|---|---|---|
| **速度 0.22 m/s 时车只转不走**（10/22 的「只转不走」也是这个原因） | 控制器发 `v=0.22` 发了 15 秒，前方距离不变；0.1 和 0.15 m/s 都能正常前进 | `max_linear_vel: 0.15` |
| 用最近点选左右会选错（一个墙角挡住整条通道），车走回头路 | `lab04_real_3` 的转弯记录 | 左右改用扇区中位数（只用于真车；`side_metric`） |
| 转向角对齐 90° 倍数转得太多：里程计坐标轴取决于开机时车怎么放 | — | `snap_to_axes: false`、`turn_until_clear: true`、`turn_angle_deg: 45` |
| LDS-02 打空返回 **NaN**，且 `range_min = 0.0` | `ros2 topic echo /scan --once` | 丢弃 NaN 和小于 5 cm 的读数 |
| 车架空时转弯永远转不完 | 驱动设置 `use_imu: true` | 只在地上测转弯 |
| 拔电源线时机器人电脑会重启 | `uptime` | 搬车前先换电池；重启后重开路由和 bringup |

**Lab04 检查清单**

1. 按顺序启动，每个一个终端，开了就别关：**路由 → bringup →（录制）→ 控制器**。路由一旦重启，bringup 和控制器也要重启。
2. 确认路由在监听：`ss -ltn | grep 7447`
3. 单独测电机（车架空）：`ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"`（不要用 0.22，见上）
4. 运行控制器，看它每 2 秒打印的状态（`state=... v=... odom=...`）。
5. 另一个终端：`ros2 topic info /cmd_vel` → `Publisher count: 1`、`Subscription count: 1`。
6. bringup 报 `There is no status packet` / `stack smashing detected`：电机板断电 10 秒再开（等音乐响完），重启 bringup。
7. 在仓库根目录运行测试脚本：`SIM_WS=~/ros_ws scripts/run_dz_test.sh <标签> <容差°> <最小转速> <秒> [none|zero|box]`（仿真，Gazebo Classic；如果仿真在 `~/ros2_ws`，把 `SIM_WS` 改成 `~/ros2_ws`）。

---

### 第 11 部分：R02 EKF 定位（`lab04_pkg`）

`lab04_pkg` 用扩展卡尔曼滤波以 20 Hz 估计机器人位姿并发布 `/ekf`（`nav_msgs/Odometry`）。开始前请确保 `~/ros_ws` 已编译并加载 `turtlebot3_perception`（提供 `landmark_msgs`，见第 3 部分）。

#### 11.1 仿真（Task 0、Task 1、Task 2）

**Task 0（纯 Python 出图与雅可比校验，不需要 ROS）：**

```bash
cd ~/SESASR-SIQUBro/src/lab04_pkg
python3 scripts/task0_models.py /tmp/ekf_task0
```

✅ 生成速度运动模型与地标观测模型的采样图（PDF/PNG），并用 `sympy` 核对雅可比矩阵。

**Task 1（状态 `[x, y, θ]`）与 Task 2（状态 `[x, y, θ, v, ω]`，`fusion:=true`）：**

需要开**三个终端**（录 bag 再开第四个）。

**终端 1：Lab04 仿真世界（含地标模拟器）**

```bash
source ~/ros_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_ignition lab04.launch.py
```

（虚拟机里先加 `export LIBGL_ALWAYS_SOFTWARE=1`；Gazebo Classic 把 `turtlebot3_ignition` 换成 `turtlebot3_gazebo`。）

**终端 2：EKF 节点**

```bash
source ~/ros_ws/install/setup.bash
source ~/SESASR-SIQUBro/install/setup.bash
ros2 launch lab04_pkg ekf.launch.py                  # Task 1：状态 [x, y, θ]
# 或：
ros2 launch lab04_pkg ekf.launch.py fusion:=true     # Task 2：状态 [x, y, θ, v, ω]
```

✅ 终端打印 `EKF started: state size 3, 9 landmarks`（Task 2 为 `state size 5`）。

**终端 3：让小车动起来（用 Bump&Go 或键盘遥控）**

```bash
source ~/SESASR-SIQUBro/install/setup.bash
ros2 run lab02_pkg bump_and_go --ros-args \
  --params-file ~/SESASR-SIQUBro/install/lab02_pkg/share/lab02_pkg/config/sim.yaml \
  -p stop_at_goal:=false -p use_sim_time:=true
```

**终端 4（可选）：录包与评估**

```bash
mkdir -p ~/SESASR-SIQUBro/bags && cd ~/SESASR-SIQUBro/bags
ros2 bag record -o ekf_sim_t1 /ground_truth /odom /ekf /landmarks /cmd_vel /imu
```

按 `Ctrl+C` 结束录制后，计算 RMSE/MAE 并出图：

```bash
python3 ~/SESASR-SIQUBro/src/lab04_pkg/scripts/evaluate_ekf.py \
  ~/SESASR-SIQUBro/bags/ekf_sim_t1 ~/SESASR-SIQUBro/bags/ekf_plots "Task 1"
```

用**同一个** bag 对比 Task 1 和 Task 2：

```bash
python3 ~/SESASR-SIQUBro/src/lab04_pkg/scripts/replay_filters.py \
  ~/SESASR-SIQUBro/bags/ekf_sim_t1 ~/SESASR-SIQUBro/src/lab04_pkg/config/sim.yaml
```

也可以直接用一键 150 s 仿真脚本：

```bash
bash ~/SESASR-SIQUBro/src/lab04_pkg/scripts/run_sim_experiment.sh sim_t1 false 150
bash ~/SESASR-SIQUBro/src/lab04_pkg/scripts/run_sim_experiment.sh sim_t2 true 150
```

仿真参考结果（150 s 运行）：相对 `/ground_truth` 的位置 RMSE，Task 1 从 `/odom` 的 **0.018 m** 降到 `/ekf` 的 **0.010 m**，Task 2 从 **0.020 m** 降到 **0.010 m**；同一段录包上，Task 2 让航向误差降低约 20–30 %，位置误差基本不变。

#### 11.2 真车（Task 3）

1. **把 `lab04_pkg` 拷到机器人电脑**（真车不能上网）：
   ```bash
   scp -r ~/SESASR-SIQUBro/src/lab04_pkg students@<robot-ip>:~/group_203/src/
   ```
2. **启动真车 4 个基础程序**（4 个独立 SSH 终端）：
   - 终端 1：`ros2 run rmw_zenoh_cpp rmw_zenohd`
   - 终端 2：`ros2 launch turtlebot3_bringup robot.launch.py`
   - 终端 3：`ros2 launch turtlebot3_perception camera.launch.py`
   - 终端 4：`ros2 launch turtlebot3_perception apriltag.launch.py`
3. **准备当天标签坐标**：每次实验课标签都会重摆，把当天的 `id x y` 坐标写进机器人的 `~/tag_positions.txt`（或按 `config/landmarks_thu1211.yaml` 格式写 YAML）。
4. **选好起点**：让相机起步就能**同时看到至少 2 个标签**（初始位姿不准且只看到 1 个标签时，定位存在两个对称解），并量出起点在 map 坐标系下的 `X Y THETA`（单位：米、米、弧度）。
5. **在 `~/group_203` 运行一键脚本**（终端 5）：
   ```bash
   cd ~/group_203
   LANDMARKS=$HOME/tag_positions.txt bash src/lab04_pkg/scripts/robot_task3.sh run1 X Y THETA
   # 跑 Task 2 末尾加 fusion：
   LANDMARKS=$HOME/tag_positions.txt bash src/lab04_pkg/scripts/robot_task3.sh run2 X Y THETA fusion
   ```
   脚本会自动检查 `/odom` 和 `/camera/landmarks`、编译 `lab04_pkg`、把 `tag_positions.txt` 转成 YAML、启动 `ekf_node` 并将 `/odom /imu /scan /camera/landmarks /tf /tf_static /cmd_vel /ekf` 录到 `~/g203_bags/<RUN_NAME>`。开车绕场后按 `Ctrl+C` 干净停止；即使 EKF 启动失败，脚本等待 10 秒后仍会录下原始传感器话题供事后回放。
6. **拷回录包**（在自己电脑上运行）：
   ```bash
   scp -r students@<robot-ip>:~/g203_bags/run1 ~/SESASR-SIQUBro/bags/
   ```
7. **在自己电脑回放真车 bag**（离线调参重跑 EKF）：
   ```bash
   # 终端 1
   ros2 bag play --clock 1000 --remap /ekf:=/old_ekf -- ~/SESASR-SIQUBro/bags/run1
   # 终端 2
   source ~/SESASR-SIQUBro/install/setup.bash
   ros2 launch lab04_pkg ekf.launch.py robot:=real use_sim_time:=true
   ```

真车参考记录（11/19，3 号机器人）：172 s 行驶中看到了 9 个标签里的 7 个，结束时 `/ekf` 与 `/odom` 相差 **0.28 m**（真车没有真值，这个数字只说明两者及标签相互一致，不代表绝对精度）。注意：换电池会让机器人电脑重启，4 个基础程序要全部重开。

---

### 第 12 部分：R03 DWA 避障与目标跟随（`lab07_pkg`）

`lab07_pkg` 以 15 Hz 运行动态窗口法（DWA）节点 `dwa_node`，支持两种模式：
- **Task 1（`mode:=goal`）**：避开障碍物驶向静态目标（支持 `/goal_pose` 坐标，也支持锁定一个静止 AprilTag）。
- **Task 2（`mode:=follow`）**：保持约 0.8 m 距离跟随移动目标（仿真订阅 `/dynamic_goal_pose`，真车订阅 `/camera/landmarks`）。
- 结果发到 `/nav_result`（`Goal`、`Collision`、`Timeout`、`Lost`），每 50 步在 `/distance_to_goal` 发布一次剩余距离。

#### 12.1 仿真（Task 1 与 Task 2）

**终端 1：DWA 迷宫世界（`hard_maze.world`）**

```bash
source ~/ros_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_ignition project.launch.py
```

（虚拟机里先加 `export LIBGL_ALWAYS_SOFTWARE=1`；Gazebo Classic 把 `turtlebot3_ignition` 换成 `turtlebot3_gazebo`。）

**终端 2：DWA 节点**

```bash
source ~/SESASR-SIQUBro/install/setup.bash
ros2 launch lab07_pkg dwa.launch.py                  # Task 1：静态目标
# 或：
ros2 launch lab07_pkg dwa.launch.py mode:=follow     # Task 2：跟随红方块
```

**终端 3：发送静态目标（仅 Task 1）**

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 1.5, y: -0.5}}}"
```

✅ 终端 2 打印 `Accepted goal`，每 50 步输出一次距离，最后打印 `Navigation finished with result: Goal`。

**录 bag 与评估：**

```bash
# 在发目标或启动跟随前开始录制
mkdir -p ~/SESASR-SIQUBro/bags && cd ~/SESASR-SIQUBro/bags
ros2 bag record -o r03_sim_goal /ground_truth /odom /scan /cmd_vel /goal_pose /dynamic_goal_pose /nav_result /distance_to_goal

# 出指标和轨迹/速度图
python3 ~/SESASR-SIQUBro/src/lab07_pkg/scripts/evaluate_nav.py \
  ~/SESASR-SIQUBro/bags/r03_sim_goal --robot sim --mode goal --out-dir ~/SESASR-SIQUBro/bags/eval_sim
```

（Task 2 把 `--mode goal` 改成 `--mode follow`。）

**批量重复 Gazebo 实验（Task 1）：**

```bash
bash ~/SESASR-SIQUBro/src/lab07_pkg/scripts/run_goal_trials.sh                # 8 个点 × 3 次
TRIALS=1 bash ~/SESASR-SIQUBro/src/lab07_pkg/scripts/run_goal_trials.sh       # 每个点快速跑 1 次
INCLUDE_HARD=1 bash ~/SESASR-SIQUBro/src/lab07_pkg/scripts/run_goal_trials.sh # 额外跑 4 个局部极小点
```

仿真参考结果（`hard_maze.world`）：
- **Task 1**：8 个可到达目标 × 3 次 = **24 / 24 全部到达、0 碰撞**（中心墙后的 4 个点安全停滞至 `Timeout`，属于纯局部 DWA 已知的局部极小局限）。
- **Task 2**：课程路线 **3 / 3 跟完第一圈、0 碰撞**（第一圈平均距离 2.0–2.8 m，红方块贴墙角 0.115 m 切弯处车会暂时落后到 3.5–4.9 m，随后追上）。

#### 12.2 真车（Task 3 — 组员独立上车流程）

按下面步骤顺序操作，**不需要别人帮忙就能独立跑完**。

**⚠️ 开车前必看安全规则**
1. **先架空、后落地**：第一次运行把车架空（轮子离地），确认转向方向和 `Ctrl+C` 停车都正常，再放回地面。
2. **人始终站在车旁边**，随时能一把拿起车。
3. **真车默认安全参数**：最高线速度 `0.15 m/s`，急停半径 `0.305 m`（车半径 `0.105` + 容差 `0.20`），规划半径 `0.405 m`（多留 `0.10 m` 余量），激光三点中值滤波默认开启。
4. **随时停车**：在 DWA 终端按 `Ctrl+C`（节点退出会自动发零速度）。任意终端强制急停：
   ```bash
   pkill -f "lib/lab07_pkg/dwa_node"; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
   ```
5. **电量检查**：确认电池电量充足（换电池会导致机器人电脑重启，终端 1–4 必须全部重开）。

**第 1 步：把代码拷到 `~/group_203/src` 并编译（车上不能上网）**

在**自己电脑**（连上 `SESASR_WiFi`）运行：

```bash
scp -r ~/SESASR-SIQUBro/src/lab07_pkg students@<robot-ip>:~/group_203/src/
```

在**机器人（SSH）**运行：

```bash
cd ~/group_203
source /opt/ros/humble/setup.bash
colcon build --packages-select lab07_pkg --symlink-install
source install/setup.bash
```

✅ 最后一行显示 `Summary: 1 package finished`。

**第 2 步：启动真车 4 个基础程序（机器人上开 4 个 SSH 终端）**

| 终端 | 命令 | ✅ 正常标志 |
|---|---|---|
| 1 | `ros2 run rmw_zenoh_cpp rmw_zenohd` | 无报错（全机只开一个！） |
| 2 | `ros2 launch turtlebot3_bringup robot.launch.py` | 看到 `LDS-02 started successfully` 和 `Run!` |
| 3 | `ros2 launch turtlebot3_perception camera.launch.py` | 相机正常出流（用标签目标或跟随时必开） |
| 4 | `ros2 launch turtlebot3_perception apriltag.launch.py` | 正常发布 `/camera/landmarks`（用标签目标或跟随时必开） |

> **起点与坐标系（跑静态目标必看）：** `/odom` 的原点 `(0, 0)` 和 `+x`（正前方）就是 **`robot.launch.py` 启动那一刻小车所在的位置和朝向**。所以每轮开始前：**先把车摆到起点 → 重启终端 2（`robot.launch.py`）**，再量目标相对起点的 `(X, Y)`（米，`+x` 正前，`+y` 正左）。

**第 3 步 A：静态目标——发坐标（`mode:=goal`，默认 `goal_source:=topic`）**

- **终端 5（启动 DWA）：**
  ```bash
  source ~/group_203/install/setup.bash
  ros2 launch lab07_pkg dwa.launch.py robot:=real mode:=goal
  ```
- **终端 6（录 bag 到 `~/group_203/bags`，每轮改名 `r03_goal1` / `goal2` / `goal3`）：**
  ```bash
  mkdir -p ~/group_203/bags
  ros2 bag record -o ~/group_203/bags/r03_goal1 \
    /odom /scan /camera/landmarks /cmd_vel /goal_pose /nav_result /distance_to_goal /tf /tf_static
  ```
- **终端 7（发目标，把 `x`、`y` 换成量好的米数）：**
  ```bash
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: odom}, pose: {position: {x: 1.5, y: 0.0}}}"
  ```
- 看到终端 5 打印 `Goal`（或 `Collision` / `Timeout`）后，在终端 6 按 `Ctrl+C` 停止录包，确认 `~/group_203/bags/r03_goal1/metadata.yaml` 已生成。下一轮把车搬回起点、重启终端 2 再跑。

**第 3 步 B：静态目标——看一个不动的 AprilTag（`goal_source:=landmark`）**

如果现场静态目标用一个静止的 AprilTag（编号 `<N>`）标出：

- **终端 5（启动 DWA）：**
  ```bash
  source ~/group_203/install/setup.bash
  ros2 launch lab07_pkg dwa.launch.py robot:=real mode:=goal goal_source:=landmark target_id:=<N>
  ```
- **终端 6（录 bag）：** 同第 3 步 A（换新 bag 名）。
- 让车头朝向标签 `<N>`。首次看到该标签时，节点会自动沿「车→标签」连线**往回退 `landmark_goal_standoff`（`0.5 m`）**锁定为一次性静态目标，保证车安全停在标签前而不触发 `0.305 m` 急停。

**第 3 步 C：移动目标跟随（`mode:=follow`）**

- **终端 5（启动 DWA——真车必须传 `target_id`！）：**
  ```bash
  source ~/group_203/install/setup.bash
  ros2 launch lab07_pkg dwa.launch.py robot:=real mode:=follow target_id:=<N>
  ```
  （`real_robot.yaml` 设了 `require_target_id: true`，防止场地里的固定定位标签干扰。）
- **终端 6（录 bag）：**
  ```bash
  mkdir -p ~/group_203/bags
  ros2 bag record -o ~/group_203/bags/r03_follow1 \
    /odom /scan /camera/landmarks /cmd_vel /nav_result /distance_to_goal /tf /tf_static
  ```

**跟随时拿 AprilTag 的人怎么走（上车前先看！）**

| 要点 | 原因 |
|---|---|
| 标签竖直拿在**小车相机高度**，正面朝向镜头 | 减少丢检，`/camera/landmarks` 持续稳定 |
| 起步站在车正前方 **0.8–1.0 m** | 贴合 `desired_target_dist = 0.8 m`，远离 `0.305 m` 急停圈 |
| **慢走，保持 0.10–0.12 m/s** | 真车最高线速度只有 `0.15 m/s`，走太快车追不上 |
| **转大弯、缓弯**，不要在车前突然 90° 急拐 | 真车相机水平半视场只有 **±33°**（`0.576 rad`）；标签出镜超过 `2.0 s` 车会停车等待，回到画面自动继续 |
| 不要往回倒退靠近小车，急停时与车保持 **≥ 0.6 m** | 激光会把人腿当障碍物，进到 `0.305 m` 内会触发锁定急停 |
| 万一触发 `Collision` 停车，清开障碍后调 `/reset_nav` | `ros2 service call /reset_nav std_srvs/srv/Trigger` 安全解锁 |

**第 4 步：拷走录包并跑评估**

下课前在**自己电脑**把 `~/group_203/bags/` 拷回（确认每个 bag 里都有 `metadata.yaml`）：

```bash
scp -r students@<robot-ip>:~/group_203/bags/r03_goal1 ~/SESASR-SIQUBro/bags/
```

在 Ubuntu 虚拟机 / 电脑上跑评估脚本出指标和图：

```bash
# 静态目标（可一次传多个 bag）
python3 ~/SESASR-SIQUBro/src/lab07_pkg/scripts/evaluate_nav.py \
  ~/SESASR-SIQUBro/bags/r03_goal1 ~/SESASR-SIQUBro/bags/r03_goal2 ~/SESASR-SIQUBro/bags/r03_goal3 \
  --robot real --mode goal --out-dir ~/SESASR-SIQUBro/bags/eval_real_goal

# 移动跟随（传入当时跟的标签编号）
python3 ~/SESASR-SIQUBro/src/lab07_pkg/scripts/evaluate_nav.py \
  ~/SESASR-SIQUBro/bags/r03_follow1 --robot real --mode follow --target-id <N> \
  --out-dir ~/SESASR-SIQUBro/bags/eval_real_follow
```

#### 12.3 R03 出问题速查表

| 现象 | 常见原因 → 处理 |
|---|---|
| 一启动就报 `Collision` | 车身 0.305 m 内有墙/箱子/人脚，或忘了加 `robot:=real`（仿真默认 `nan_policy: min` 会把 LDS-02 的 `NaN` 当成 0.05 m）→ 确认带 `robot:=real`，换空旷一点的起点 |
| `Too few valid LiDAR rays` 或 `LiDAR NaN ratio ... exceeds max_nan_ratio` | 激光没出数或全 0 / 全 NaN → 检查 `robot.launch.py` 和 `ros2 topic echo /scan --once` |
| 跟随模式拒绝启动（报 `require_target_id`） | 忘了传 `target_id:=<N>` → 加上 `target_id:=<N>` 重启 |
| 报 `Collision` 后再发新目标没反应 | 正常行为（碰撞锁定防误启动）→ 移开障碍后运行 `ros2 service call /reset_nav std_srvs/srv/Trigger` |
| 车不动、一直打 `All candidate trajectories collide` | 起点离墙太近（`< 0.405 m`），或目标被障碍正挡住（局部极小）→ 换起点/目标，或见下方调小 `planning_margin` |
| 电机微响但车起步不动 | 静摩擦或电量低 → 确认用了 `robot:=real`（已默认 `window_source: "last_cmd"`），检查电池电量 |
| 跟随时中途停下（`Target lost > 2.0 s`） | 标签移出 ±33° 相机视野或被遮挡 → 把标签拿回镜头前，车会自动恢复，无需复位 |
| 跟随时 `/nav_result` 报了 `Lost` | 连续 10 s 距离超过 3.0 m → 仅提醒不停车，车会继续追，回到 3 m 内自动重新布防 |
| 静止标签目标停在标签前约 0.5 m | 正常行为（`landmark_goal_standoff = 0.5 m`），避免撞进 `0.305 m` 急停区 |

#### 12.4 现场最可能要调的参数（`src/lab07_pkg/config/real_robot.yaml`）

用 `--symlink-install` 编译后，修改 `~/group_203/src/lab07_pkg/config/real_robot.yaml` 下次启动即生效（不放心可再跑一次 `colcon build --packages-select lab07_pkg --symlink-install`）。

| 参数 | 真车默认值 | 现场什么时候调、怎么调 |
|---|---|---|
| `planning_margin` | `0.10 m`（规划半径 `0.405 m`） | 真车场地比仿真窄，若车过不去窄门或空隙，调小到 `0.06–0.08 m`（必须保持 `> band_floor_buffer = 0.03 m`） |
| `collision_tol` | `0.20 m`（急停半径 `0.305 m`） | 指导书允许 `0.15–0.25 m`，场地很窄时可适度降到 `0.16–0.18 m` |
| `max_lin_vel` | `0.15 m/s` | 窄场地可降到 `0.12 m/s`；**不要**调回 `0.22 m/s`（10/29 实测 0.22 m/s 只转不走） |
| `desired_target_dist` | `0.8 m` | 如果人腿太近导致车频繁减速，可调大到 `0.9–1.0 m` |
| `landmark_goal_standoff` | `0.5 m` | 静态标签目标的退后距离（必须保持 `> 0.305 m` 急停半径） |
| `scan_median_filter` | `true` | 滤除 LDS-02 单条射线毛刺，保持 `true`；仅在测试极细杆子（`< 3 cm`）时考虑关闭 |
| `window_source` | `"last_cmd"` | 防止真车低速起步被静摩擦卡死，真车保持 `"last_cmd"` |
