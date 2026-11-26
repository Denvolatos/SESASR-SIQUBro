#!/usr/bin/env bash
# Run repeated Gazebo Task 1 trials (restarting simulation per trial from origin)
# to compute success rate and simulation-time metrics. Intended for Ubuntu VM.
#
# Uses setsid + process-group termination (kill -- -PGID) plus name-based cleanup
# so no dwa_node / parameter_bridge / robot_state_publisher processes leak across
# trials, and delegates goal publishing + /nav_result waiting to run_single_trial.py.
#
# Usage:
#   ./run_goal_trials.sh                     # 8 reachable goals, TRIALS=3 each
#   TRIALS=1 ./run_goal_trials.sh            # quick smoke run (1 trial per goal)
#   INCLUDE_HARD=1 ./run_goal_trials.sh      # also run 4 local-minimum goals
#   RECORD_BAGS=1 ./run_goal_trials.sh       # record rosbag2 per trial

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRIALS=${TRIALS:-3}
INCLUDE_HARD=${INCLUDE_HARD:-0}
RECORD_BAGS=${RECORD_BAGS:-0}
TRIAL_TIMEOUT_SEC=${TRIAL_TIMEOUT_SEC:-140}
OUT_DIR=${OUT_DIR:-"trial_results_$(date +%Y%m%d_%H%M%S)"}

# 8 reachable goals verified in Gazebo with planning_margin=0.10 (world frame)
REACHABLE_GOALS=(
    "1.5 -0.5"
    "2.0 1.0"
    "2.0 -1.0"
    "2.2 -3.4"
    "3.5 -3.0"
    "-3.45 0.8"
    "-4.0 0.0"
    "-3.5 -0.5"
)

# 4 local-minimum cases (blocked by central walls from origin)
HARD_GOALS=(
    "-2.45 -2.0"
    "2.0 3.0"
    "-2.0 2.0"
    "-4.0 2.0"
)

GOALS=("${REACHABLE_GOALS[@]}")
if [[ "${INCLUDE_HARD}" == "1" ]]; then
    GOALS+=("${HARD_GOALS[@]}")
fi

mkdir -p "${OUT_DIR}"
CSV_FILE="${OUT_DIR}/summary.csv"
echo "goal_x,goal_y,trial,result,sim_sec,path_m,min_scan_m" > "${CSV_FILE}"

SIM_PGID=""
NODE_PGID=""
BAG_PGID=""

kill_pgid() {
    local pgid="$1"
    if [[ -n "${pgid}" ]] && kill -0 -- "-${pgid}" 2>/dev/null; then
        kill -INT -- "-${pgid}" 2>/dev/null || true
        sleep 2
        kill -TERM -- "-${pgid}" 2>/dev/null || true
        sleep 1
        kill -KILL -- "-${pgid}" 2>/dev/null || true
    fi
}

cleanup_trial() {
    kill_pgid "${BAG_PGID}"
    kill_pgid "${NODE_PGID}"
    kill_pgid "${SIM_PGID}"

    # Fallback name-based cleanup matching VM reference script
    pkill -f "[l]ab07_pkg/dwa_node" 2>/dev/null || true
    pkill -INT -f "[p]roject.launch.py" 2>/dev/null || true
    sleep 2
    pkill -9 -f "[i]gn gazebo|[r]uby /usr/bin/ign|[p]arameter_bridge|[r]obot_state_publisher" 2>/dev/null || true

    SIM_PGID=""
    NODE_PGID=""
    BAG_PGID=""
    sleep 2
}

trap cleanup_trial EXIT INT TERM

wait_for_topic() {
    local topic="$1"
    local max_wait="${2:-40}"
    local elapsed=0
    while (( elapsed < max_wait )); do
        if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
            return 0
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    return 1
}

total_runs=0
total_goals=0
total_collisions=0
total_timeouts=0
total_other=0

for goal in "${GOALS[@]}"; do
    read -r gx gy <<< "${goal}"
    for (( t = 1; t <= TRIALS; t++ )); do
        total_runs=$((total_runs + 1))
        tag="g_${gx}_${gy}_t${t}"
        echo "=== Running goal (${gx}, ${gy}) trial ${t}/${TRIALS} ==="

        setsid ros2 launch turtlebot3_ignition project.launch.py \
            > "${OUT_DIR}/${tag}_sim.log" 2>&1 < /dev/null &
        SIM_PGID=$!
        wait_for_topic "/odom" 80 || echo "WARN: /odom not visible yet"
        sleep 6

        setsid ros2 launch lab07_pkg dwa.launch.py robot:=sim mode:=goal \
            > "${OUT_DIR}/${tag}_dwa.log" 2>&1 < /dev/null &
        NODE_PGID=$!
        sleep 3

        if [[ "${RECORD_BAGS}" == "1" ]]; then
            setsid ros2 bag record \
                /ground_truth /scan /cmd_vel /nav_result /distance_to_goal /goal_pose \
                -o "${OUT_DIR}/${tag}_bag" > "${OUT_DIR}/${tag}_bag.log" 2>&1 < /dev/null &
            BAG_PGID=$!
            sleep 1
        fi

        line=$(timeout "$((TRIAL_TIMEOUT_SEC + 35))" \
            python3 "${SCRIPT_DIR}/run_single_trial.py" "${gx}" "${gy}" \
            --trial-timeout "${TRIAL_TIMEOUT_SEC}" 2>"${OUT_DIR}/${tag}_runner.err" \
            | grep -E "^RESULT=" | tail -n 1 || true)

        result=$(echo "${line}" | sed -n 's/.*RESULT=\([^ ]*\).*/\1/p')
        sim_sec=$(echo "${line}" | sed -n 's/.*SIM_TIME=\([^ ]*\).*/\1/p')
        path_m=$(echo "${line}" | sed -n 's/.*PATH=\([^ ]*\).*/\1/p')
        min_scan_m=$(echo "${line}" | sed -n 's/.*MIN_SCAN=\([^ ]*\).*/\1/p')

        result=${result:-ScriptTimeout}
        sim_sec=${sim_sec:-0.00}
        path_m=${path_m:-0.00}
        min_scan_m=${min_scan_m:--1.00}

        case "${result}" in
            Goal) total_goals=$((total_goals + 1)) ;;
            Collision) total_collisions=$((total_collisions + 1)) ;;
            Timeout) total_timeouts=$((total_timeouts + 1)) ;;
            *) total_other=$((total_other + 1)) ;;
        esac

        echo "Result: ${result} | sim_time=${sim_sec}s | path=${path_m}m | min_scan=${min_scan_m}m"
        echo "${gx},${gy},${t},${result},${sim_sec},${path_m},${min_scan_m}" >> "${CSV_FILE}"
        cleanup_trial
    done
done

echo ""
echo "=== Summary (${CSV_FILE}) ==="
echo "Total: ${total_runs} | Goal: ${total_goals} | Collision: ${total_collisions} | Timeout: ${total_timeouts} | Other: ${total_other}"
