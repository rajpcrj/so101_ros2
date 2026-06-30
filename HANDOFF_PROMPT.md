# Handoff Prompt — SO-101 ROS2 IK/FK + Visualization

You are picking up work on the **SO-101 follower arm** ROS2 workspace at
`<workspace-root>`. Read this fully before
touching anything — it captures non-obvious findings from the previous session
that will save you from re-discovering several bugs.

---

## What the user wants

The user is exploring IK/FK on the 5-DOF SO-101 arm and wants to **see the arm
move in RViz** in response to:
1. Terminal FK/IK commands (send a pose/joints → arm moves),
2. Drag-and-drop in the RViz MotionPlanning panel,
3. Joint sliders.

They also want to compare the **two IK algorithms** (exact KDL vs approximate
pick_ik). They prefer the **simplest** possible commands and concise answers.

---

## The robot (verified facts)

- **Planning group:** `soArm`  (5 joints, in this order):
  `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll`
  (gripper is a separate group `soGrippers`, joint `gripper`).
- **Base frame:** `base_link`
- **TIP / end-effector link of soArm:** **`gripper_link`** — NOT `wrist_link`.
  ⚠️ This was the #1 gotcha. The SRDF/comments mention `wrist_link`, but
  move_group reports `Available tip frames: [gripper_link, ]`. Use `gripper_link`
  for FK link names and let IK use the group default tip. Using `wrist_link`
  causes `Cannot compute IK ... pose reference frame 'wrist_link'` → instant `-31`.
- Arm is **5-DOF**, so arbitrary 6-DOF poses are often unreachable.

---

## Two MoveIt configs = two IK solvers (they CANNOT run together)

| Package | Solver | Service (raw) | Named proxy |
|---|---|---|---|
| `so101_moveit_config` | KDL (exact, 5 ms, collision-checked) | `/compute_ik` | `/compute_ik_so` |
| `so101_moveit_config_ik_approx` | pick_ik (approx, 50 ms, 3 attempts, 5 cm / ~6° tolerance, `avoid_collisions=False`) | `/compute_ik` | `/compute_ik_so_approx` |

⚠️ **Critical architecture fact:** BOTH packages' `move_group` expose the SAME
generic `/compute_ik` (and `/compute_fk`). So **only one move_group can run at a
time.** If you launch the exact stack, `/compute_ik_so_approx` secretly hits KDL,
not pick_ik. To actually test pick_ik you must shut down the exact stack and
launch the approx package's `demo.launch.py`.

- **Exact (KDL):** hits the pose exactly or returns `-31` (NO_IK_SOLUTION). No slack.
- **Approx (pick_ik):** returns nearest reachable within 5 cm / ~6°; great for a
  5-DOF arm where exact fails. BUT poses beyond that tolerance still return `-31`.

### pick_ik gotcha
pick_ik FAILS instantly with `Found empty JointState message` → `-31` if the IK
request has **no seed state**. You MUST populate
`ik_request.robot_state.joint_state` with the 5 joint names + positions (KDL
tolerates an empty seed; pick_ik does not).

---

## Files created last session (all at workspace root)

- **`launch_ik_services_v2.sh`** — one-command stack: MoveIt `demo.launch.py`
  (move_group + ros2_control fake hw + RViz drag-and-drop) + `/compute_ik_so` &
  `/compute_fk_so` + approx services (if built) + optional `--sliders`.
  Run: `./launch_ik_services_v2.sh` or `./launch_ik_services_v2.sh --sliders`.
- **`move_to_ik.py`** — pose/joints → IK → EXECUTES on `soarmcontroller` so the
  arm visibly moves. Already fixed to use `gripper_link` tip + seed pick_ik.
  Examples:
  - `python3 move_to_ik.py --joints 0.3 0.2 0.1 0.0 0.0`   (always works)
  - `python3 move_to_ik.py --pose 0.2888 -0.0509 0.1095 ...`  (pose → exact IK → move)
  - add `--approx` to use pick_ik (needs approx stack running)
- **`slider_to_arm.py`** — bridges joint_state_publisher_gui sliders (on side
  topic `/slider_joint_commands`) to `soarmcontroller`, so sliders don't fight
  ros2_control's joint_state_broadcaster.

These are NOT yet built into the colcon install — they run directly with
`python3` from the workspace root after `source install/setup.bash`. They are
also NOT committed to git.

---

## How to verify a pose IS reachable (the reliable workflow)

Exact IK rejects most hand-typed poses. To get a guaranteed-good pose:
pick joints → run FK at `gripper_link` → feed that exact pose back to IK.

```bash
ros2 service call /compute_fk_so moveit_msgs/srv/GetPositionFK \
  "{header: {frame_id: 'base_link'}, fk_link_names: ['gripper_link'], \
    robot_state: {joint_state: {name: ['shoulder_pan','shoulder_lift', \
    'elbow_flex','wrist_flex','wrist_roll'], position: [0.2,0.3,0.3,0,0]}}}"
```

Even FK-derived poses sometimes fail exact KDL (5 ms budget, seeded from current
state, collision-checked) — that's expected; use `--approx` or `--joints`.

---

## Simplest commands (what the user keeps asking for)

- **Just view + drag the arm:**
  `ros2 launch so101_moveit_config demo.launch.py`
- **Approx (pick_ik) stack:**
  `ros2 launch so101_moveit_config_ik_approx demo.launch.py`
- Always `source install/setup.bash` first. `Ctrl+C` to stop.

---

## RViz markers (the user asked about these)

Two unrelated marker systems:
1. **MoveIt markers** (default in demo stack): draggable orange goal handle
   (interactive marker, sets IK goal), TF axis triads, translucent planned-path
   ghost. Topic side: `/motion_plan_request`. This is what's normally on screen.
2. **Custom link-pose markers** from `link_pose_publisher.py` — colored per-link
   arrows + text labels + a yellow `so_101_camera` sphere, on `/link_poses/markers`.
   ⚠️ Only started by **`visualize_all.sh`**, NOT by the demo/v2 stacks. The RViz
   "Link Poses" display exists but is empty unless that publisher runs. It's
   TF-driven and safe to run alongside the MoveIt stack.

---

## Controllers (fake hardware, ros2_control)

`demo.launch.py` spawns: `soarmcontroller` (JointTrajectoryController, arm),
`sogrippercontroller` (GripperActionController), `joint_state_broadcaster`.
Execution action: `/soarmcontroller/follow_joint_trajectory`. All sim — no real
Feetech hardware involved in these flows.

---

## Open follow-ups (not done yet)

- Add an `--approx` / `--exact` flag to `launch_ik_services_v2.sh` so it can boot
  either solver cleanly (currently it only launches the exact stack).
- Consider committing the 3 new scripts + installing them via CMakeLists if the
  user wants them as proper `ros2 run` targets.
- A true side-by-side exact-vs-approx comparison requires restarting between
  stacks (shared `/compute_ik` name).

## Environment notes
- `source install/setup.bash` works; `ROS_DISTRO` may print empty in fresh shells
  but the workspace functions (ROS 2 Humble, pick_ik from `/opt/ros/humble`).
- `DISPLAY=:1`, GUIs open fine.
- Shutting down a stack: SIGINT the launch; `ros2 node list` can show stale cached
  nodes briefly after — verify with `ps`/`pgrep` if unsure.
