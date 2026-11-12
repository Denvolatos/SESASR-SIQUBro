#!/bin/bash
# Simulation experiment: lab04 world + EKF + Bump&Go to move the robot + rosbag.
# usage: run_sim_experiment.sh <label> <fusion true|false> <seconds>  (workspaces: ~/ros_ws, ~/SESASR-SIQUBro)
LABEL=$1; FUSION=$2; DUR=${3:-150}
export XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash; source ~/ros_ws/install/setup.bash; source ~/SESASR-SIQUBro/install/setup.bash; export TURTLEBOT3_MODEL=burger
OUT=~/ekf_results/$LABEL; rm -rf $OUT; mkdir -p $OUT
setsid ros2 launch turtlebot3_ignition lab04.launch.py > $OUT/sim.log 2>&1 < /dev/null &
for i in $(seq 90); do ros2 topic list 2>/dev/null | grep -q '^/odom$' && break; sleep 2; done
sleep 12
setsid ros2 launch lab04_pkg ekf.launch.py fusion:=$FUSION > $OUT/ekf.log 2>&1 < /dev/null &
sleep 4
setsid ros2 bag record -o $OUT/bag /ground_truth /odom /ekf /landmarks /cmd_vel /imu > $OUT/bag.log 2>&1 < /dev/null &
sleep 3
CFG=~/SESASR-SIQUBro/install/lab02_pkg/share/lab02_pkg/config/sim.yaml
setsid ros2 run lab02_pkg bump_and_go --ros-args --params-file $CFG -p stop_at_goal:=false -p use_sim_time:=true > $OUT/ctrl.log 2>&1 < /dev/null &
sleep $DUR
timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
pkill -INT -f "ros2 bag record"; sleep 3
pkill -f "lib/lab02_pkg/bump_and_go"; pkill -INT -f "ekf.launch.py"; sleep 1; pkill -f "lib/lab04_pkg/ekf_node"
pkill -INT -f "lab04.launch.py"; sleep 5; pkill -9 -f "ign gazebo|ruby /usr/bin/ign|parameter_bridge|robot_state_publisher|landmark_simulator"; sleep 2
grep -c "wall at" $OUT/ctrl.log; grep -iE "error|traceback" $OUT/ekf.log | head -3; echo finished $LABEL
