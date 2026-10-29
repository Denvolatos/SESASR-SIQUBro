#!/bin/bash
# Real robot: stop everything, restart zenoh router + bringup in the right order,
# then drive forward for 3 s and check that the odometry moves.
# usage (on the robot, wheels lifted!): scripts/robot_restart_test.sh [stop|dds]
#   dds: skip zenoh and use FastDDS on localhost only, to tell zenoh problems from driver problems
LOG=~/group_203/logs; mkdir -p $LOG
if [ "$1" = dds ]; then
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=1
  ros2 daemon stop > /dev/null 2>&1
fi

stop_all() {
  pkill -INT -f "[r]os2 topic pub"
  pkill -INT -f "[r]obot.launch.py"
  sleep 3
  pkill -f "[t]urtlebot3_ros|[l]d08_driver|[r]obot_state_publisher"
  pkill -f "[r]mw_zenohd"
  sleep 2
}

stop_all
if [ "$1" = stop ]; then echo "all stopped"; exit 0; fi
if pgrep -af "[r]mw_zenohd|[t]urtlebot3_ros|[l]d08_driver"; then
  echo "FAIL: processes above are still running"; exit 1
fi
echo "env: ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"

if [ "$1" = dds ]; then
  echo "1/4 FastDDS mode: no zenoh router"
else
  echo "1/4 starting zenoh router ..."
  nohup ros2 run rmw_zenoh_cpp rmw_zenohd > $LOG/router.log 2>&1 &
  for i in $(seq 15); do ss -ltn | grep -q 7447 && break; sleep 1; done
  ss -ltn | grep -q 7447 || { echo "FAIL: router not listening on 7447"; exit 1; }
  echo "    router OK"
fi

echo "2/4 starting bringup ..."
nohup ros2 launch turtlebot3_bringup robot.launch.py > $LOG/bringup.log 2>&1 &
for i in $(seq 40); do grep -q "diff_drive_controller\]: Run!" $LOG/bringup.log && break; sleep 1; done
grep -q "diff_drive_controller\]: Run!" $LOG/bringup.log || { echo "FAIL: bringup not ready"; tail -15 $LOG/bringup.log; exit 1; }
echo "    bringup OK"
sleep 2

echo "3/4 driving forward 0.22 m/s for 3 s (wheels must be lifted) ..."
timeout 4 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.22}}" > /dev/null 2>&1 &
PUB=$!
sleep 2.5
timeout 3 ros2 topic echo /odom --once --field twist.twist.linear | head -1
wait $PUB
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1

echo "4/4 bringup errors (if any):"
grep -iE "error|fail|no status|smash" $LOG/bringup.log | tail -5
echo "done. router and bringup keep running in the background (logs in $LOG)."
echo "stop them with: scripts/robot_restart_test.sh stop"
