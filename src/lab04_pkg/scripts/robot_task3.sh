#!/usr/bin/env bash
# Task 3 on the real robot: check the robot topics, start the EKF and record a bag.
# Run it on the robot PC from the repository root, after the four robot launches are up
# (rmw_zenohd, turtlebot3_bringup robot.launch.py, camera.launch.py, apriltag.launch.py):
#
#   bash src/lab04_pkg/scripts/robot_task3.sh RUN_NAME [X Y THETA] [fusion]
#
#   RUN_NAME   name of the bag folder, e.g. run1 (a folder that exists is never overwritten)
#   X Y THETA  start pose of the robot in the map frame [m m rad]; default 0 0 0
#              (any number format: 0, 1.5, -1.57)
#   fusion     add the word "fusion" to run the Task 2 filter instead of Task 1
#
# Landmarks of the day (required, the arena changes every session):
#   LANDMARKS=/full/path/tag_positions.txt bash ...
#   (the course file "id x y" per line, or a yaml like config/landmarks_thu1211.yaml)
#
# Stop with Ctrl+C: the bag is closed properly, the EKF is stopped and the bag info is printed.
# If the EKF does not start, the script waits 10 s (Ctrl+C to fix and retry), then records the
# raw topics anyway, so the run can be replayed later.

set -u
RUN=${1:-}
if [ -z "$RUN" ]; then
    sed -n 2,21p "$0"
    exit 1
fi
X=${2:-0}; Y=${3:-0}; TH=${4:-0}
FUSION=false
[ "${5:-}" = "fusion" ] && FUSION=true
BAG=$HOME/g203_bags/$RUN
ROOT=$(pwd)
say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }
fail() { printf '\033[31mSTOP: %s\033[0m\n' "$*"; exit 1; }

# ROS rejects "[0, 0, 0]" for a float array parameter: always pass floats like 0.0
for v in X Y TH; do
    val=$(python3 -c 'import sys; print(float(sys.argv[1]))' "${!v}" 2>/dev/null) \
        || fail "start pose must be numbers, got '${!v}'"
    printf -v "$v" '%s' "$val"
done
[ -n "${LANDMARKS:-}" ] || fail "set LANDMARKS to the tag file of today, e.g. LANDMARKS=\$HOME/tags_today.txt bash $0 $*"
[ -f "$LANDMARKS" ] || fail "LANDMARKS file not found: $LANDMARKS"
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
for t in /odom /camera/landmarks; do   # needed by the EKF
    echo "$TOPICS" | grep -qx "$t" || MISSING="$MISSING $t"
done
for t in /imu /scan; do                # only recorded: warn, do not stop
    echo "$TOPICS" | grep -qx "$t" || echo "warning: $t not published (recorded if it appears later)"
done
if [ -n "$MISSING" ]; then
    echo "missing:$MISSING"
    echo "  /odom /imu /scan     -> ros2 launch turtlebot3_bringup robot.launch.py"
    echo "  /camera/landmarks    -> ros2 launch turtlebot3_perception camera.launch.py"
    echo "                          ros2 launch turtlebot3_perception apriltag.launch.py"
    echo "  nothing at all       -> is rmw_zenohd running? (pgrep -af rmw_zenohd)"
    fail "start the missing robot nodes, then run this script again"
fi
echo "ok: /odom /camera/landmarks"
SEEN=$(timeout 5 ros2 topic echo /camera/landmarks --once 2>/dev/null | grep -oE '\bid: *-?[0-9]+' | awk '{print $2}' | tr '\n' ' ')
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

mkdir -p "$HOME/g203_bags"
if [[ "$LANDMARKS" != *.yaml && "$LANDMARKS" != *.yml ]]; then
    # course format: one "id x y" per line; lines that do not start with a number are skipped
    YAML=$HOME/g203_bags/$RUN.landmarks.yaml
    python3 - "$LANDMARKS" "$YAML" <<'PY' || fail "could not read $LANDMARKS (expected lines: id x y)"
import sys
rows = []
for line in open(sys.argv[1]):
    r = line.replace(',', ' ').split()
    try:
        rows.append((int(r[0]), float(r[1]), float(r[2])))
    except (IndexError, ValueError):
        if line.strip():
            print('skipped line: ' + line.strip())
if not rows:
    sys.exit(1)
ids, xs, ys = (list(c) for c in zip(*rows))
with open(sys.argv[2], 'w') as f:
    f.write('landmarks:\n  id: %s\n  x: %s\n  y: %s\n' % (ids, xs, ys))
for i, x, y in rows:
    print('  tag %3d: x %7.3f  y %7.3f' % (i, x, y))
PY
    LANDMARKS=$YAML
fi

say "4/5 Start the EKF (fusion=$FUSION, start pose $X $Y $TH)"
EKF_LOG=$HOME/g203_bags/$RUN.ekf.log
# own process group, so that the stop below reaches ekf_node and not only the "ros2 run" wrapper
setsid ros2 run lab04_pkg ekf_node --ros-args \
    --params-file "$ROOT/install/lab04_pkg/share/lab04_pkg/config/real_robot.yaml" \
    -p fusion:=$FUSION -p "init_pose:=[$X, $Y, $TH]" \
    ${LANDMARKS:+-p landmarks_file:=$LANDMARKS} >"$EKF_LOG" 2>&1 &
EKF_PID=$!
stop_ekf() {
    kill -TERM -- -$EKF_PID 2>/dev/null || return 0
    for _ in 1 2 3 4 5; do
        pgrep -g $EKF_PID >/dev/null || return 0
        sleep 1
    done
    kill -KILL -- -$EKF_PID 2>/dev/null
}
trap stop_ekf EXIT
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
    tail -15 "$EKF_LOG"
    echo -e "\033[31mEKF did not start (log: $EKF_LOG).\033[0m"
    echo -e "\033[31mCtrl+C within 10 s to fix it and retry with a new RUN_NAME; otherwise only the raw topics are recorded.\033[0m"
    sleep 10
fi

say "5/5 Recording to $BAG  -- drive the robot now, Ctrl+C to stop"
ros2 bag record -o "$BAG" /odom /imu /scan /camera/landmarks /tf /tf_static /cmd_vel /ekf
stop_ekf
say "Bag info"
ros2 bag info "$BAG" | grep -E 'Duration|Messages|Topic:'
[ -f "$BAG/metadata.yaml" ] && echo "metadata.yaml ok" || echo "no metadata.yaml: run  ros2 bag reindex $BAG"

echo
echo "Copy the folder $BAG to your computer before leaving, e.g. from your laptop:"
echo "  scp -r students@<robot-ip>:g203_bags/$RUN ."
