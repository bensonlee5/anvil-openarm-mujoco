# Testing Guide

This repo models the Anvil OpenARM 2.0 in MuJoCo and exposes a ROS 2 bridge
with joint-space and commanded end-effector interfaces. Use this checklist to
verify the generated MJCF, the Anvil-specific joint limits, the TCP frames, the
local commanded-EE IK approximation, ROS topic compatibility, and eventual
hardware parity.

## Setup

From the repo root:

```bash
git submodule update --init
uv sync
```

macOS does not provide a native ROS 2 environment. Run ROS tests through Docker
or use a Linux ROS 2 environment with this repo mounted.

## Full Local Gate

Run this before relying on the model:

```bash
uv run python scripts/check_model.py
uv run pytest -q
uv run python scripts/demo_wrist_sweep.py --headless
git diff --check
```

If Docker is available, also run:

```bash
scripts/run_viewer_docker.sh --smoke
scripts/run_ros2_tests.sh
```

Expected result:

- `check_model.py` compiles every generated MJCF file.
- Every arm joint range and actuator ctrlrange matches `anvil_openarm_spec.py`.
- Both TCP sites exist and match the upstream OpenArm v2 grasp-frame transform.
- `pytest` passes locally, with ROS tests skipped if `rclpy` is unavailable.
- The wrist sweep reaches approximately J6 -45..+70 degrees and J7 +/-90
  degrees.
- The viewer smoke test builds the Linux viewer image and keeps the official
  MuJoCo viewer alive under Xvfb until the expected timeout.
- The Docker suite passes all ROS 2 bridge tests.

## Generated Model Reproducibility

The generated files under `models/` should be treated as build artifacts. Edit
`anvil_openarm_spec.py`, `scripts/make_anvil_model.py`, or upstream files, then
regenerate:

```bash
uv run python scripts/make_anvil_model.py
uv run python scripts/check_model.py
uv run pytest tests/test_model_generation.py -q
```

`tests/test_model_generation.py` renders into a temporary directory and compares
against the tracked `models/anvil_*.xml` files. A failure means the generator
and checked-in models disagree.

## Sim-Core IK Tests

Run the ROS-free sim tests:

```bash
uv run pytest tests/test_sim_core.py -q
```

The commanded-EE IK tests cover:

- TCP pose reporting from `follower_l_hand_tcp` and `follower_r_hand_tcp`.
- Small Cartesian target convergence through `OpenArmSim.command_ee()`.
- Far Cartesian targets producing bounded joint targets instead of jumps.
- SDLS-style damping keeping near-singular directions finite and small.
- Joint-specific velocity-limit helpers.
- Nullspace posture and joint-limit centering bias.
- `CommandedEEPose.gripper` metres mapping to the mirrored MJCF gripper angle
  convention.

The IK behavior intentionally approximates Anvil's public description: SDLS
for singularity handling, joint-specific velocity caps, and nullspace posture
control. It is not Anvil's closed hardware IK implementation.

## ROS 2 Bridge Tests

Run the Docker suite:

```bash
scripts/run_ros2_tests.sh
```

This builds a ROS 2 Jazzy image, installs MuJoCo and pytest, then runs all
tests in a ROS environment. The bridge tests verify:

- `/joint_states` publishes names, positions, velocities, and effort.
- `/ee_pose_left` publishes a normalized TCP pose in `world`.
- Joint command topics move the arm and clamp to Anvil joint limits.
- `/commanded_ee_left` accepts the default `geometry_msgs/PoseStamped` shim and
  moves the TCP closer to a small target.
- Malformed joint commands are rejected without stopping publication.

## Viewer Tests

The official MuJoCo viewer currently crashes on this macOS setup with:

```text
RuntimeError: Caught an unknown exception!
```

Reproduce the local official-viewer failure with:

```bash
uv run python scripts/view.py --official models/anvil_pedestal.xml
```

For normal local macOS viewing, use the default fallback:

```bash
uv run python scripts/view.py models/anvil_pedestal.xml
uv run python scripts/demo_wrist_sweep.py
```

For the official MuJoCo viewer, use the Linux viewer container. Headless smoke
test:

```bash
scripts/run_viewer_docker.sh --smoke
```

Interactive macOS/XQuartz flow:

```bash
open -a XQuartz
xhost + 127.0.0.1
scripts/run_viewer_docker.sh models/anvil_pedestal.xml
xhost - 127.0.0.1
```

Interactive Linux/X11 flow:

```bash
xhost +local:docker
scripts/run_viewer_docker.sh models/anvil_pedestal.xml
xhost -local:docker
```

The smoke test runs the official viewer under Xvfb with a timeout. A timeout is
treated as success because it means the interactive viewer stayed alive instead
of failing during startup.

## Manual ROS 2 Smoke Test

In one ROS 2 shell:

```bash
python3 -m bridge.ros2_bridge --ros-args -p time_scale:=1.0
```

In another shell:

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /ee_pose_left --once
ros2 topic pub --once /follower_l_forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 1.0, 0, 0.5, 0, 0]}"
ros2 topic pub --once /commanded_ee_left geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.35, y: 0.2, z: 0.2}, orientation: {w: 1.0}}}"
```

Watch `/joint_states` and `/ee_pose_left` after each command. Use small
commanded-EE deltas near the current TCP pose; far targets are intentionally
rate-limited and may require repeated messages.

## Quest Teleop Compatibility

Anvil's public docs describe `CommandedEEPose` fields (`header`, `pose`,
`gripper`) but do not publish the package name. On the Anvil devbox:

```bash
source /opt/ros/jazzy/setup.bash
source <anvil_workspace>/install/setup.bash
ros2 topic type /commanded_ee_left
ros2 topic hz /commanded_ee_left
```

Start the bridge with auto-detection if the Quest teleop publisher is already
running:

```bash
python3 -m bridge.ros2_bridge --ros-args \
  -p commanded_ee_msg_type:=auto
```

Or pass the exact type returned by `ros2 topic type`:

```bash
python3 -m bridge.ros2_bridge --ros-args \
  -p commanded_ee_msg_type:=<anvil_pkg>/msg/CommandedEEPose
```

Validation steps:

1. Confirm the bridge subscribes to `/commanded_ee_left` and
   `/commanded_ee_right` with `ros2 topic info`.
2. Move the Quest controllers slowly and watch `/joint_states`.
3. Confirm `/ee_pose_left` and `/ee_pose_right` move continuously, without
   large discontinuities.
4. Verify gripper movement if the teleop publisher sends the custom `gripper`
   field. The default `PoseStamped` shim leaves gripper position unchanged.

## Hardware Comparison

Once the physical robot arrives, validate sim parity conservatively:

1. With the real robot powered and the Anvil stack running, record passive
   state:
   ```bash
   ros2 topic list
   ros2 topic echo /joint_states --once
   ros2 topic type /commanded_ee_left
   ```
2. Compare joint names and command ordering. Always index `/joint_states` by
   joint name; Anvil documents that state ordering differs from command
   ordering.
3. Move one joint by a few degrees from its current position and verify the
   same client command behaves similarly in sim and hardware.
4. Sweep J6 slowly toward the local spec limits: -45 degrees and +70 degrees.
   If your unit reaches a different limit, update `anvil_openarm_spec.py`,
   regenerate, and re-run this guide.
5. Bag comparable sim and hardware runs:
   ```bash
   ros2 bag record /joint_states /ee_pose_left /ee_pose_right
   ```
   Compare positions and limits first. Do not expect effort readings,
   contact-rich behavior, or exact IK nullspace posture to match until the
   sim gains and controller details are identified from hardware.

## Troubleshooting

- If `uv run pytest -q` skips ROS tests, that is expected outside a ROS 2
  Python environment. Use `scripts/run_ros2_tests.sh`.
- If `commanded_ee_msg_type:=auto` does not subscribe, start the Quest teleop
  publisher first or pass the exact message type from `ros2 topic type`.
- If commanded-EE motion appears slow for far targets, that is the velocity cap
  working. Send continuous small target updates as Quest teleop does.
- If generated models fail reproducibility, run `uv run python
  scripts/make_anvil_model.py` and inspect the XML diff.
- If hardware limits differ from the local pre-arrival spec, update
  `anvil_openarm_spec.py`; do not hand-edit generated XML.

## References

- Anvil IK and controls overview:
  <https://docs.anvil.bot/software/system-overview/inverse-kinematics-and-controls>
- Anvil command topics:
  <https://docs.anvil.bot/software/technical-reference/commanding-robot-movement>
- Anvil robot state topics:
  <https://docs.anvil.bot/software/technical-reference/querying-robot-state>
