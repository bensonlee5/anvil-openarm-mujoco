# Testing Guide

This repo models the Anvil OpenARM 2.0 in MuJoCo and exposes a ROS 2 bridge
with joint-space and commanded end-effector interfaces. Use this checklist to
verify the generated MJCF, the Anvil-specific joint limits, the TCP frames, the
local commanded-EE IK approximation, ROS topic compatibility, and eventual
hardware parity.

## Compatibility Scope

This repo is a simulation backend for OpenArm v2 / Anvil OpenARM 2.0 geometry
and ROS command interfaces, not a full teleop or inference product.

- OpenArm v2 model compatibility: generated MJCF starts from upstream OpenArm
  v2, then applies the local Anvil OpenARM 2.0 joint-limit and TCP deltas.
- Quest teleop compatibility: the bridge accepts `/commanded_ee_left` and
  `/commanded_ee_right` messages from an external teleop publisher, including
  an installed custom `CommandedEEPose` type with `header`, `pose`, and
  optional `gripper`.
- Quest app and data collection: not included here.
- OpenArm v2 inference: not included here. External policy code can publish
  joint commands or commanded-EE targets into the sim, but this repo does not
  provide policy checkpoints, observation builders, camera pipelines, dataset
  writers, or an inference server.
- OpenArm v2 loader configs: the hosted demo extracts selectable profile
  metadata from `upstream/anvil_loader/config/openarm_v2_*.yaml`; the YAML
  files remain owned by the `anvil-robotics/anvil-loader` submodule.

## Setup

From the repo root:

```bash
git submodule update --init --recursive
docker version
scripts/run_docker.sh --help
scripts/run_docker.sh build all
```

Docker is the default test runtime for this repo. It gives the checks a
reproducible Linux environment, includes the ROS 2 Jazzy image for bridge tests,
and avoids the missing-`rclpy` path on macOS.

Native `uv` and `npm` commands still work for fast local iteration where the
host has the right dependencies, but the instructions below use the Docker
harness unless a section is explicitly about host hardware or a ROS devbox.

The Docker wrapper reuses tagged images by default and mounts the current
checkout into the container. Normal source edits do not require rebuilding.
Rebuild explicitly after Dockerfile, dependency, ROS base image, or submodule
layout changes:

```bash
scripts/run_docker.sh rebuild all
# or force a rebuild for a single run:
ANVIL_DOCKER_REBUILD=1 scripts/run_docker.sh check
```

## Full Docker Gate

Run this before relying on the model:

```bash
scripts/run_docker.sh build all
scripts/run_docker.sh check
scripts/run_docker.sh test -q
scripts/run_docker.sh wrist-headless
scripts/run_docker.sh web-build
scripts/run_docker.sh viewer-smoke
scripts/run_docker.sh ros-test
```

Then run the host-side diff hygiene check:

```bash
git diff --check
```

Expected result:

- `check_model.py` compiles every generated MJCF file.
- Every arm joint range and actuator ctrlrange matches `anvil_openarm_spec.py`.
- Both TCP sites exist and match the upstream OpenArm v2 grasp-frame transform.
- `pytest` passes in the Python container.
- The wrist sweep reaches approximately J6 -45..+70 degrees and J7 +/-90
  degrees.
- The web build exports hosted assets and builds the Vite app.
- The viewer smoke test builds the Linux viewer image and keeps the official
  MuJoCo viewer alive under Xvfb until the expected timeout.
- The Docker suite passes all ROS 2 bridge tests.

## Generated Model Reproducibility

The generated files under `models/` should be treated as build artifacts. Edit
`anvil_openarm_spec.py`, `scripts/make_anvil_model.py`, or upstream files, then
regenerate in the Python container when you intentionally want to update
tracked XML:

```bash
scripts/run_docker.sh generate
```

Validate the regenerated output through Docker:

```bash
scripts/run_docker.sh check
scripts/run_docker.sh test tests/test_model_generation.py -q
```

`tests/test_model_generation.py` renders into a temporary directory and compares
against the tracked `models/anvil_*.xml` files. A failure means the generator
and checked-in models disagree.

## Sim-Core IK Tests

Run the ROS-free sim tests in the Python container:

```bash
scripts/run_docker.sh test tests/test_sim_core.py -q
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
scripts/run_docker.sh ros-test
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

Use the hosted browser demo for normal visual work:

```bash
scripts/run_docker.sh web-dev
```

Open <http://localhost:5173/> and select a demo/profile.

Use the Linux viewer container for the official MuJoCo desktop viewer. Headless
smoke test:

```bash
scripts/run_docker.sh viewer-smoke
```

For the official desktop viewer on macOS, use the Linux viewer container with
XQuartz:

```bash
open -a XQuartz
xhost + 127.0.0.1
scripts/run_docker.sh viewer models/anvil_pedestal.xml
xhost - 127.0.0.1
```

Interactive Linux/X11 flow:

```bash
xhost +local:docker
scripts/run_docker.sh viewer models/anvil_pedestal.xml
xhost -local:docker
```

The native Python viewer still exists for Linux hosts with MuJoCo viewer
dependencies installed:

```bash
uv run python scripts/view.py models/anvil_pedestal.xml
uv run python scripts/demo_wrist_sweep.py
```

The smoke test runs the official viewer under Xvfb with a timeout. A timeout is
treated as success because it means the interactive viewer stayed alive instead
of failing during startup.

## Docker ROS 2 Smoke Test

Start the bridge in the ROS 2 Jazzy container:

```bash
scripts/run_docker.sh ros-bridge --ros-args -p time_scale:=1.0
```

On Linux, the command uses host networking so other host ROS 2 processes can
discover the bridge. On Docker Desktop for macOS, DDS discovery across the
host/container boundary is limited; for interactive ROS topic testing, run the
ROS client in the same Linux/container environment or on the robot/devbox.

In another sourced ROS 2 shell in that same environment:

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /ee_pose_left --once
ros2 topic pub --once /follower_l_forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 1.0, 0, 0.5, 0, 0]}"
ros2 topic pub --once /commanded_ee_left geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.35, y: 0.2, z: 0.2}, orientation: {w: 1.0}}}"
```

Use small commanded-EE deltas near the current TCP pose; far targets are
intentionally rate-limited and may require repeated messages.

If you are on a real ROS 2 Linux or Anvil devbox shell, the equivalent native
bridge command is:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m bridge.ros2_bridge --ros-args -p time_scale:=1.0
```

If the native command fails with `No module named 'rclpy'`, that shell is not a
ROS 2 Python environment. Use the Docker bridge command above.

## Quest Teleop Compatibility

The goal is to let an external Quest teleop publisher drive the MuJoCo sim
through the same commanded-EE topic shape used by the robot stack. This repo
does not include the Quest app or VR transport.

For hosted browser demos, use
`?config=openarm_v2_quest_teleop_commanded_ee` when you want the selected
profile to match the commanded-EE bridge path, or
`?config=openarm_v2_quest_teleop` for the follower joint-command Quest profile.
Both selections are metadata only in the browser; an external Quest publisher
is still required for real ROS teleop.

Anvil's public docs describe `CommandedEEPose` fields (`header`, `pose`,
`gripper`) but do not publish the package name. On the Anvil/OpenArm devbox:

```bash
source /opt/ros/jazzy/setup.bash
source <anvil_workspace>/install/setup.bash
ros2 topic type /commanded_ee_left
ros2 topic hz /commanded_ee_left
```

Start the bridge with auto-detection if the Quest teleop publisher is already
running. In Docker:

```bash
scripts/run_docker.sh ros-bridge --ros-args \
  -p commanded_ee_msg_type:=auto
```

Or pass the exact type returned by `ros2 topic type`:

```bash
scripts/run_docker.sh ros-bridge --ros-args \
  -p commanded_ee_msg_type:=<anvil_pkg>/msg/CommandedEEPose
```

For Anvil's custom message type to load in Docker, the package that defines it
must also be available and sourced inside the container. If that package only
exists on the robot/devbox, run the native bridge there instead.

Validation steps:

1. Confirm the bridge subscribes to `/commanded_ee_left` and
   `/commanded_ee_right` with `ros2 topic info`.
2. Move the Quest controllers slowly and watch `/joint_states`.
3. Confirm `/ee_pose_left` and `/ee_pose_right` move continuously, without
   large discontinuities.
4. Verify gripper movement if the teleop publisher sends the custom `gripper`
   field. The default `PoseStamped` shim leaves gripper position unchanged.

## Inference Compatibility

This repo does not implement OpenArm v2 learned-policy inference. Treat it as a
sim target for an external inference runner.

For hosted browser demos, `?config=openarm_v2_inference` selects the upstream
loader profile metadata from `anvil-loader`. It does not load a checkpoint or
start an inference server.

Supported command outputs from an external policy:

- Joint-space actions published to
  `/follower_l_forward_position_controller/commands` and
  `/follower_r_forward_position_controller/commands`.
- Cartesian TCP targets published to `/commanded_ee_left` and
  `/commanded_ee_right`; these are converted with the local SDLS-style IK
  approximation and are not guaranteed to match a hardware controller exactly.

Not provided:

- camera/image observation topics,
- dataset recording/writing,
- action normalization or policy-specific wrappers,
- policy checkpoint loading,
- ACT, Diffusion Policy, GR00T, Pi, or other model runtimes.

Validation steps for an external inference runner:

1. Start `scripts/run_docker.sh ros-bridge --ros-args -p time_scale:=1.0`.
2. Confirm the policy publishes one of the supported command topic families.
3. Watch `/joint_states` and `/ee_pose_left` / `/ee_pose_right` for continuous,
   bounded motion.
4. Compare against hardware only after verifying joint names, command order,
   gripper convention, and camera/observation assumptions in the inference
   stack.

## Hardware Comparison

With a physical robot, validate sim parity conservatively:

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

- If native `uv run pytest -q` skips ROS tests, that is expected outside a
  ROS 2 Python environment. Use `scripts/run_docker.sh ros-test`.
- If `python3 -m bridge.ros2_bridge` reports missing `rclpy`, source ROS 2
  first or use `scripts/run_docker.sh ros-bridge`.
- If `commanded_ee_msg_type:=auto` does not subscribe, start the Quest teleop
  publisher first or pass the exact message type from `ros2 topic type`.
- If commanded-EE motion appears slow for far targets, that is the velocity cap
  working. Send continuous small target updates as Quest teleop does.
- If generated models fail reproducibility, run `scripts/run_docker.sh
  generate` and inspect the XML diff.
- If hardware limits differ from the local pre-arrival spec, update
  `anvil_openarm_spec.py`; do not hand-edit generated XML.

## References

- Anvil IK and controls overview:
  <https://docs.anvil.bot/software/system-overview/inverse-kinematics-and-controls>
- Anvil command topics:
  <https://docs.anvil.bot/software/technical-reference/commanding-robot-movement>
- Anvil robot state topics:
  <https://docs.anvil.bot/software/technical-reference/querying-robot-state>
