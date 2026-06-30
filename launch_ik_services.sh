#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$SCRIPT_DIR/install/setup.bash"

echo "[move_group] Launching ..."
ros2 launch so101_moveit_config move_group.launch.py &
PID_MG=$!

echo "[move_group] Waiting for /compute_ik to become available ..."
WAIT_SECS=0
until ros2 service list 2>/dev/null | grep -q "/compute_ik"; do
    sleep 1
    WAIT_SECS=$((WAIT_SECS + 1))
    if [ $WAIT_SECS -ge 30 ]; then
        echo "[move_group] WARNING: /compute_ik not found after 30s — starting IK services anyway."
        break
    fi
done
echo "[move_group] Ready (waited ${WAIT_SECS}s)."

echo "[IK Services] Starting /compute_ik_so and /compute_fk_so ..."
ros2 run so101_moveit_config ik_fk_services.py &
PID1=$!

echo "[IK Approx Services] Starting /compute_ik_so_approx and /compute_fk_so_approx ..."
ros2 run so101_moveit_config_ik_approx ik_fk_services_approx.py &
PID2=$!

trap "echo 'Shutting down...'; kill $PID_MG $PID1 $PID2" SIGINT SIGTERM

wait $PID_MG $PID1 $PID2
