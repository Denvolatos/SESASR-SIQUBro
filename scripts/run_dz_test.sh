#!/bin/bash
# Dead-zone comparison test in Gazebo Classic.
# usage: run_dz_test.sh <label> <rotation_tolerance_deg> <min_angular_vel> <seconds>
LABEL=$1; TOL=$2; WMIN=$3; DUR=$4; EXTRA=${5:-none}   # none | zero | box
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_WS="${SIM_WS:-$HOME/ros_ws}"
if [ ! -f "$SIM_WS/install/setup.bash" ] || [ ! -f "$PROJECT_ROOT/install/setup.bash" ]; then
  echo "Build both the simulation and group workspaces before running this script." >&2
  exit 1
fi
source /usr/share/gazebo/setup.bash
source /opt/ros/humble/setup.bash
source "$SIM_WS/install/setup.bash"
source "$PROJECT_ROOT/install/setup.bash"
export TURTLEBOT3_MODEL=burger
L=/tmp/dz_$LABEL; mkdir -p $L
echo "DISPLAY=$DISPLAY WAYLAND=$WAYLAND_DISPLAY"
setsid ros2 launch turtlebot3_gazebo lab02.launch.py > $L/gazebo.log 2>&1 < /dev/null &
for i in $(seq 1 40); do ros2 topic list 2>/dev/null | grep -q '^/scan$' && break; sleep 1; done
sleep 3
if [ "$EXTRA" = box ]; then
  # obstacle 0.3 m in front of the robot at start (like feet / a wall next to the robot)
  cat > $L/box.sdf <<'SDF'
<?xml version="1.0"?><sdf version="1.6"><model name="obstacle"><static>true</static><link name="l">
<collision name="c"><geometry><box><size>0.1 0.4 0.4</size></box></geometry></collision>
<visual name="v"><geometry><box><size>0.1 0.4 0.4</size></box></geometry></visual></link></model></sdf>
SDF
  ros2 run gazebo_ros spawn_entity.py -entity obstacle -file $L/box.sdf -x 0.35 -y 0 -z 0.2 > $L/spawn.log 2>&1
  echo "spawned obstacle at x=0.35"
fi
if [ "$EXTRA" = zero ]; then
  # a forgotten terminal that keeps publishing zero velocity at 10 Hz
  setsid ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}" > $L/zero.log 2>&1 < /dev/null &
  echo "extra zero publisher on /cmd_vel at 10 Hz"
fi
setsid python3 "$PROJECT_ROOT/scripts/deadzone_filter.py" > $L/filter.log 2>&1 < /dev/null &
setsid ros2 run lab02_pkg bump_and_go --ros-args -r /cmd_vel:=/cmd_vel_raw \
  -p rotation_tolerance_deg:=$TOL -p min_angular_vel:=$WMIN > $L/ctrl.log 2>&1 < /dev/null &
POSE=/ground_truth; ros2 topic list | grep -q '^/ground_truth$' || POSE=/odom
echo "pose topic: $POSE"
T=0
while [ $T -lt $DUR ]; do
  sleep 10; T=$((T+10))
  P=$(timeout 4 ros2 topic echo --once $POSE --field pose.pose.position 2>/dev/null | head -2 | awk '{printf "%s=%.2f ", $1, $2}')
  W=$(timeout 3 ros2 topic echo --once /cmd_vel_raw --field angular.z 2>/dev/null | head -1)
  V=$(timeout 3 ros2 topic echo --once /cmd_vel_raw --field linear.x 2>/dev/null | head -1)
  echo "t=${T}s $P raw_v=$V raw_w=$W"
  grep -q "goal reached" $L/ctrl.log && { echo "GOAL REACHED"; break; }
done
echo "--- controller log"; sed 's/.*\] //' $L/ctrl.log | grep -v -E "^\s*$" | tail -12
pkill -f "[l]ab02_pkg/bump_and_go"; pkill -f "[t]opic pub -r 10 /cmd_vel"; pkill -f "[d]eadzone_filter"; pkill -f "[t]urtlebot3_gazebo lab02"; pkill -f "[g]zserver"; pkill -f "[g]zclient"
sleep 3; pgrep -fa "[g]zserver|[g]zclient" || echo "clean"
