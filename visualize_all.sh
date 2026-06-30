#!/bin/bash
set -m  # job control: each background job gets its own process group

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/install/setup.bash"
source "$SCRIPT_DIR/camera_config.env"
export CAMERA_NAME CAMERA_STREAM_URL

# Collect all PIDs into one array as they start
ALL_PIDS=()

cleanup() {
    echo ''
    echo 'Shutting down...'
    # SIGINT first — lets ROS2 nodes shut down gracefully
    for pid in "${ALL_PIDS[@]}"; do
        kill -INT -- -"$pid" 2>/dev/null
    done
    sleep 3
    # SIGKILL any survivors
    for pid in "${ALL_PIDS[@]}"; do
        kill -KILL -- -"$pid" 2>/dev/null
    done
    wait 2>/dev/null
    echo 'All stopped.'
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "[RSP] Starting robot_state_publisher..."
ros2 launch so101_moveit_config rsp.launch.py &
ALL_PIDS+=($!)

echo "[JSP] Starting joint_state_publisher (zero pose)..."
ros2 run joint_state_publisher joint_state_publisher &
ALL_PIDS+=($!)

echo "[move_group] Starting..."
ros2 launch so101_moveit_config move_group.launch.py &
ALL_PIDS+=($!)

echo "Waiting for /tf..."
until ros2 topic list 2>/dev/null | grep -q "^/tf$"; do sleep 1; done
sleep 2

echo "[RViz2] Starting with link poses config..."
ros2 launch so101_moveit_config moveit_rviz.launch.py &
ALL_PIDS+=($!)

sleep 2

echo "[Poses] Starting link pose publisher..."
python3 "$SCRIPT_DIR/link_pose_publisher.py" &
ALL_PIDS+=($!)

echo "[Camera] Starting $CAMERA_NAME stream from $CAMERA_STREAM_URL..."
python3 "$SCRIPT_DIR/camera_stream_publisher.py" &
ALL_PIDS+=($!)

echo "---"
echo "All started (PIDs: ${ALL_PIDS[*]}). Ctrl+C to shut everything down."
echo ""
echo "Topics available:"
echo "  /link_poses/markers     — MarkerArray (arrows + labels in RViz2)"
echo "  /link_poses/<link_name> — PoseStamped per link"
echo "  /camera/image_raw       — $CAMERA_NAME stream → /camera/image_raw"
echo "  /camera/camera_info     — calibrated camera info"

wait
