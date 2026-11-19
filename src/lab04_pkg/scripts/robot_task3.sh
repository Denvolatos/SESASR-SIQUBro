#!/usr/bin/env bash
# Task 3 on the real robot: check the robot topics, start the EKF and record a bag.
# Run it on the robot PC from the repository root, after the four robot launches are up
# (rmw_zenohd, turtlebot3_bringup robot.launch.py, camera.launch.py, apriltag.launch.py):
#
#   bash src/lab04_pkg/scripts/robot_task3.sh RUN_NAME [X Y THETA] [fusion]
#
#   RUN_NAME   name of the bag folder, e.g. run1 (a folder that exists is never overwritten)
#   X Y THETA  start pose of the robot in the map frame [m m rad]; default 0 0 0
#   fusion     add the word "fusion" to run the Task 2 filter instead of Task 1
#
# Landmarks of the day: LANDMARKS=/full/path/tag_positions.txt bash ...
#   (the course file "id x y" per line, or a yaml like config/landmarks_thu1211.yaml)
#
# Stop with Ctrl+C: the bag is closed properly, the EKF is stopped and the bag info is printed.
# If the EKF does not start, the raw topics are recorded anyway, so the run can be replayed later.

set -u
RUN=${1:-}
if [ -z "$RUN" ]; then
    sed -n 2,17p "$0"
    exit 1
fi
X=${2:-0.0}; Y=${3:-0.0}; TH=${4:-0.0}
FUSION=false
[ "${5:-}" = "fusion" ] && FUSION=true
BAG=$HOME/g203_bags/$RUN
ROOT=$(pwd)
say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
fail() { printf '\033[31mSTOP: %s\033[0m\n' "$*"; exit 1; }

[ -d src/lab04_pkg ] || fail "run this from the repository root (the folder that contains src/lab04_pkg)"
[ -e "$BAG" ] && fail "$BAG already exists, choose another RUN_NAME"

say "1/5 ROS environment"
set +u
[ -z "${ROS_DISTRO:-}" ] && source /opt/ros/humble/setup.bash
set -u
echo "ROS_DISTRO=${ROS_DISTRO:-?}  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-default}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
ros2 pkg prefix landmark_msgs >/dev/null 2>&1 \
    || fail "landmark_msgs not found: source the workspace that contains turtlebot3_perception first"

say "2/5 Robot topics"
TOPICS=$(timeout 15 ros2 topic list 2>/dev/null)
MISSING=""
for t in /odom /imu /scan /camera/landmarks; do
    echo "$TOPICS" | grep -qx "$t" || MISSING="$MISSING $t"
done
if [ -n "$MISSING" ]; then
    echo "missing:$MISSING"
    echo "  /odom /imu /scan     -> ros2 launch turtlebot3_bringup robot.launch.py"
    echo "  /camera/landmarks    -> ros2 launch turtlebot3_perception camera.launch.py"
    echo "                          ros2 launch turtlebot3_perception apriltag.launch.py"
    echo "  nothing at all       -> is rmw_zenohd running? (pgrep -af rmw_zenohd)"
    fail "start the missing robot nodes, then run this script again"
fi
echo "ok: /odom /imu /scan /camera/landmarks"
SEEN=$(timeout 5 ros2 topic echo /camera/landmarks --once 2>/dev/null | grep -E '^\s+id:' | awk '{print $2}' | tr '\n' ' ')
if [ -n "$SEEN" ]; then
    echo "landmarks seen now: $SEEN"
else
    echo "warning: no landmark in view right now (fine if the camera is not facing a tag)"
fi

say "3/5 Build lab04_pkg"
set +u
colcon build --packages-select lab04_pkg --symlink-install >/tmp/g203_build.log 2>&1 \
    || { tail -20 /tmp/g203_build.log; fail "build failed (full log /tmp/g203_build.log)"; }
source install/setup.bash
set -u
echo "build ok"

if [ -n "${LANDMARKS:-}" ]; then
    [ -f "$LANDMARKS" ] || fail "LANDMARKS file not found: $LANDMARKS"
    if [[ "$LANDMARKS" == *.txt ]]; then   # course format: one "id x y" per line
        mkdir -p "$HOME/g203_bags"
        YAML=$HOME/g203_bags/$RUN.landmarks.yaml
        python3 - "$LANDMARKS" "$YAML" <<'PY' || fail "could not read $LANDMARKS"
import sys
rows = [l.split() for l in open(sys.argv[1]) if l.strip()]
ids = [int(r[0]) for r in rows]
xs = [float(r[1]) for r in rows]
ys = [float(r[2]) for r in rows]
with open(sys.argv[2], 'w') as f:
    f.write('landmarks:\n  id: %s\n  x: %s\n  y: %s\n' % (ids, xs, ys))
print('landmarks: %d tags, ids %s' % (len(ids), ids))
PY
        LANDMARKS=$YAML
    fi
fi

say "4/5 Start the EKF (fusion=$FUSION, start pose $X $Y $TH)"
mkdir -p "$HOME/g203_bags"
EKF_LOG=$HOME/g203_bags/$RUN.ekf.log
# own process group, so that the stop below reaches ekf_node and not only the "ros2 run" wrapper
setsid ros2 run lab04_pkg ekf_node --ros-args \
    --params-file "$ROOT/install/lab04_pkg/share/lab04_pkg/config/real_robot.yaml" \
    -p fusion:=$FUSION -p "init_pose:=[$X, $Y, $TH]" \
    ${LANDMARKS:+-p landmarks_file:=$LANDMARKS} >"$EKF_LOG" 2>&1 &
EKF_PID=$!
EKF_OK=false
for _ in $(seq 1 15); do
    sleep 1
    kill -0 $EKF_PID 2>/dev/null || break
    if timeout 3 ros2 topic echo /ekf --once >/dev/null 2>&1; then EKF_OK=true; break; fi
done
head -3 "$EKF_LOG"
if $EKF_OK; then
    echo "EKF running, publishing /ekf"
else
    echo -e "\033[31mEKF did not start (log: $EKF_LOG). Recording the raw topics anyway.\033[0m"
    tail -15 "$EKF_LOG"
fi

stop_ekf() {
    kill -TERM -- -$EKF_PID 2>/dev/null || return 0
    for _ in 1 2 3 4 5; do
        pgrep -g $EKF_PID >/dev/null || return 0
        sleep 1
    done
    kill -KILL -- -$EKF_PID 2>/dev/null
}
trap stop_ekf EXIT

say "5/5 Recording to $BAG  -- drive the robot now, Ctrl+C to stop"
ros2 bag record -o "$BAG" /odom /imu /scan /camera/landmarks /tf /tf_static /cmd_vel /ekf
stop_ekf
say "Bag info"
ros2 bag info "$BAG" | grep -E 'Duration|Messages|Topic:'
echo
echo "Copy the folder $BAG to your computer before leaving, e.g. from your laptop:"
echo "  scp -r students@<robot-ip>:g203_bags/$RUN ."
