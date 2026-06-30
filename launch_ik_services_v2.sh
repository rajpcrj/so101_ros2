#!/bin/bash
# ============================================================================
# launch_ik_services_v2.sh
# ----------------------------------------------------------------------------
# Everything you need to SEE THE ARM MOVE, in one command:
#
#   1. MoveIt demo stack  (move_group + ros2_control fake hardware + RViz)
#        -> RViz opens with the MotionPlanning panel: DRAG-AND-DROP the goal
#           marker, click "Plan & Execute", and the arm moves.
#   2. /compute_ik_so & /compute_fk_so  (the named IK/FK proxy services)
#        -> send poses / joint states from the terminal as before.
#   3. /compute_ik_so_approx & /compute_fk_so_approx (tolerant IK, if built)
#   4. (optional) joint-slider GUI  -> drag sliders, the arm follows live.
#
# Then, from a SECOND terminal, you can MOVE the arm to an IK target:
#   source install/setup.bash
#   python3 move_to_ik.py --joints 0.3 0.2 0.1 0.0 0.0      # joint goal
#   python3 move_to_ik.py --pose   0.2 0.0 0.15             # pose -> IK -> move
#
# USAGE
#   ./launch_ik_services_v2.sh            # full stack, no sliders
#   ./launch_ik_services_v2.sh --sliders  # also open the joint-slider GUI
#
# Ctrl+C shuts everything down cleanly.
# ============================================================================
set -m  # job control: each background job in its own process group

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/install/setup.bash"

WITH_SLIDERS=0
for arg in "$@"; do
    case "$arg" in
        --sliders) WITH_SLIDERS=1 ;;
        -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    esac
done

ALL_PIDS=()

cleanup() {
    echo ''
    echo '[v2] Shutting down...'
    for pid in "${ALL_PIDS[@]}"; do
        kill -INT -- -"$pid" 2>/dev/null
    done
    sleep 3
    for pid in "${ALL_PIDS[@]}"; do
        kill -KILL -- -"$pid" 2>/dev/null
    done
    wait 2>/dev/null
    echo '[v2] All stopped.'
    exit 0
}
trap cleanup SIGINT SIGTERM

# ----------------------------------------------------------------------------
# 1. MoveIt demo stack: move_group + ros2_control (fake hw) + RViz MotionPlanning
#    This single launch gives drag-and-drop planning AND a trajectory controller
#    (soarmcontroller) that executes moves — which is what makes the arm move.
# ----------------------------------------------------------------------------
echo "[v2] Launching MoveIt demo stack (move_group + ros2_control + RViz)..."
ros2 launch so101_moveit_config demo.launch.py &
ALL_PIDS+=($!)

echo "[v2] Waiting for /compute_ik to come up..."
WAIT_SECS=0
until ros2 service list 2>/dev/null | grep -q "/compute_ik"; do
    sleep 1
    WAIT_SECS=$((WAIT_SECS + 1))
    if [ $WAIT_SECS -ge 45 ]; then
        echo "[v2] WARNING: /compute_ik not found after 45s — continuing anyway."
        break
    fi
done
echo "[v2] move_group ready (waited ${WAIT_SECS}s)."

# ----------------------------------------------------------------------------
# 2. Named IK/FK proxy services (/compute_ik_so, /compute_fk_so)
# ----------------------------------------------------------------------------
echo "[v2] Starting /compute_ik_so & /compute_fk_so ..."
ros2 run so101_moveit_config ik_fk_services.py &
ALL_PIDS+=($!)

# ----------------------------------------------------------------------------
# 3. Approximate (tolerant) IK/FK services, if that package is built.
# ----------------------------------------------------------------------------
if ros2 pkg prefix so101_moveit_config_ik_approx >/dev/null 2>&1; then
    echo "[v2] Starting /compute_ik_so_approx & /compute_fk_so_approx ..."
    ros2 run so101_moveit_config_ik_approx ik_fk_services_approx.py &
    ALL_PIDS+=($!)
else
    echo "[v2] (approx IK package not built — skipping approx services)"
fi

# ----------------------------------------------------------------------------
# 4. Optional joint-slider GUI -> bridge -> arm controller.
#    GUI publishes to a SIDE topic so it doesn't fight ros2_control's
#    joint_state_broadcaster; slider_to_arm.py forwards it to soarmcontroller.
# ----------------------------------------------------------------------------
if [ "$WITH_SLIDERS" -eq 1 ]; then
    echo "[v2] Starting joint-slider bridge..."
    python3 "$SCRIPT_DIR/slider_to_arm.py" &
    ALL_PIDS+=($!)

    echo "[v2] Opening joint_state_publisher_gui (sliders)..."
    ros2 run joint_state_publisher_gui joint_state_publisher_gui \
        --ros-args -r /joint_states:=/slider_joint_commands &
    ALL_PIDS+=($!)
fi

echo "---"
echo "[v2] All started (PIDs: ${ALL_PIDS[*]})."
echo ""
echo "  RViz: use the MotionPlanning panel — drag the orange goal marker,"
echo "        then 'Plan & Execute' to move the arm."
echo ""
echo "  Terminal (second shell):"
echo "    source install/setup.bash"
echo "    python3 move_to_ik.py --joints 0.3 0.2 0.1 0.0 0.0   # move to joints"
echo "    python3 move_to_ik.py --pose   0.2 0.0 0.15          # pose -> IK -> move"
echo ""
echo "    # raw services (no motion):"
echo "    ros2 service call /compute_ik_so moveit_msgs/srv/GetPositionIK ..."
echo "    ros2 service call /compute_fk_so moveit_msgs/srv/GetPositionFK ..."
echo ""
if [ "$WITH_SLIDERS" -eq 1 ]; then
    echo "  Sliders: drag the joint_state_publisher_gui sliders to move the arm."
    echo ""
fi
echo "  Ctrl+C to shut everything down."

wait
