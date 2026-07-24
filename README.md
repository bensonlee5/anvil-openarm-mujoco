# anvil-openarm-mujoco

MuJoCo simulation workspace for the **Anvil OpenARM 2.0** — Anvil's variant of
the [OpenArm](https://github.com/enactic/openarm) bimanual arm with an
extended-range wrist.

**Unofficial project.** This repository is not affiliated with, endorsed by, or
supported by Anvil Robotics. It attempts to approximate Anvil's OpenARM 2.0
model and custom IK/control behavior from public docs plus the session-resolved
local spec in this repo. In particular, the `/commanded_ee_*` path is an
SDLS-style MuJoCo approximation of Anvil's custom IK solver and likely has
implementation differences from Anvil's hardware stack.

What's in this repo:

- **MJCF models** of the Anvil OpenARM 2.0 (bimanual arm and pedestal scenes),
  derived from the upstream
  [enactic/openarm_mujoco](https://github.com/enactic/openarm_mujoco) v2 files
  (vendored as a git submodule) by a generator script that applies this repo's
  local Anvil joint-limit spec and TCP frames
- **Spec validation** that asserts every joint/actuator range and TCP site
  against the local Anvil spec, plus stability checks
- **A ROS 2 bridge** that exposes the simulation with the real Anvil robot's
  joint-state, joint-command, and commanded end-effector topics
  (`/joint_states`, `/follower_{l,r}_forward_position_controller/commands`,
  `/commanded_ee_{left,right}`, …), so client code can move between sim and
  hardware with minimal changes
- **Tests** for the sim core (no ROS needed) and bridge integration tests
  (run anywhere via the Docker harness)
- **Viewer and demo scripts**, including a wrist range-of-motion sweep

## Compatibility target

This repo is meant to be a MuJoCo simulation backend for the **OpenArm v2 /
Anvil OpenARM 2.0 robot shape and command interfaces**, with particular
attention to Quest teleop compatibility. It is not a complete data-collection
or policy-inference stack.

| Concern | Status in this repo |
|---|---|
| OpenArm v2 physical model | **Yes, with Anvil deltas.** The generated MJCF starts from upstream OpenArm v2 assets and kinematics, then applies this repo's Anvil OpenARM 2.0 joint-limit/TCP changes. For stock OpenArm v2 limits, use the pristine `upstream/openarm_mujoco/v2` files or remove the Anvil patches. |
| Quest teleop | **Interface-compatible target.** The ROS bridge subscribes to `/commanded_ee_left` and `/commanded_ee_right`, can auto-detect an installed custom `CommandedEEPose` type, consumes `header`, `pose`, and optional `gripper`, and maps those commands into the MuJoCo sim with local IK. |
| Quest app / VR transport | **Not included.** This repo does not ship a Quest app, hand/controller tracking bridge, camera streaming, or data recording pipeline. It expects an external teleop publisher to provide the ROS topics. |
| OpenArm v2 inference | **Not implemented.** There is no ACT/Diffusion Policy/GR00T/Pi-style policy runner, observation builder, camera stack, dataset writer, action server, or checkpoint loader here. External inference code can drive this sim through the joint-command or commanded-EE ROS topics, but this repo is not itself the inference runtime. |
| OpenArm v2 loader configs | **Selectable metadata.** The hosted demo extracts the relevant profile fields from the `anvil-robotics/anvil-loader` submodule's `openarm_v2_*.yaml` files so each loader profile can be selected by URL/UI. These selections document the intended external control surface; they do not add the missing external runtimes. |
| Hosted browser demo | **Visualization and joint-space teleop only.** The web viewer is useful for static hosted demos and keyboard/sliders, not Quest teleop or learned-policy inference. |

## Quick start

```bash
git submodule update --init --recursive      # first checkout only
uv sync                          # installs Python deps

uv run python scripts/make_anvil_model.py   # regenerate models/ from upstream
uv run python scripts/make_anvil_urdf.py    # regenerate the URDF counterpart
uv run python scripts/check_model.py        # validate against the Anvil spec

uv run python scripts/view.py                          # native desktop viewer on Linux
uv run python scripts/view.py models/anvil_pedestal.xml
uv run python scripts/demo_wrist_sweep.py              # wrist range-of-motion demo
uv run python scripts/demo_wrist_sweep.py --headless   # same, no GUI, prints ranges

npm --prefix web install                              # first web run only
npm --prefix web run dev                              # hosted browser demos
```

### Hosted browser demos

The `web/` app is a static Vite/TypeScript viewer built on the official
`@mujoco/mujoco` WASM package and Three.js. It exports the generated Anvil
models into a Menagerie-style browser asset tree before dev/build:

```bash
npm --prefix web run prepare:assets
npm --prefix web run dev
npm --prefix web run build
```

The splash page exposes the bimanual, pedestal, wrist-sweep, and full
range-of-motion demos. The full-ROM demo sweeps every
joint (J1–J7 and the gripper) through its complete Anvil range on both arms,
one joint at a time; sweep feasibility on the showroom stage is pinned by
`tests/test_web_scene.py`. The viewer renders a mujoco_anywhere-style
environment: dark slate horizon and a mirror-blended checker floor that
reflects the robot. It also exposes the OpenArm v2 loader profiles from
`upstream/anvil_loader/config/openarm_v2_*.yaml`:

- `?config=openarm_v2_inference`
- `?config=openarm_v2_leader_follower_teleop`
- `?config=openarm_v2_leader_only`
- `?config=openarm_v2_quest_teleop`
- `?config=openarm_v2_quest_teleop_commanded_ee`

Combine a profile with a scene, for example
`?demo=pedestal&config=openarm_v2_quest_teleop_commanded_ee`. Profile selection
is metadata for the hosted viewer and docs: it highlights the intended external
control surface but does not run the Quest app, leader hardware, or inference
policy inside the browser.

In a loaded demo, press `1`/`2` to select the left/right arm, use `Q/A` through
`U/J` to jog J1..J7, and use `[`/`]` for the gripper. Targets are clamped to
the MJCF actuator ranges and mirrored by the sliders.

### Docker runtime

Docker is the reproducible path for repo checks, browser builds, ROS tests, and
the official MuJoCo desktop viewer from macOS:

```bash
scripts/run_docker.sh build all              # one-time; reuses existing images
scripts/run_docker.sh check
scripts/run_docker.sh test
scripts/run_docker.sh wrist-headless
scripts/run_docker.sh web-build
scripts/run_docker.sh viewer-smoke
scripts/run_docker.sh ros-bridge --ros-args -p time_scale:=1.0
scripts/run_docker.sh ros-test
```

Run commands reuse the tagged images by default and mount the current checkout,
so code changes do not require rebuilding the images. Use
`scripts/run_docker.sh rebuild all` or `ANVIL_DOCKER_REBUILD=1` when the
Dockerfile, Python deps, Node deps, ROS base, or submodule layout changes.

For browser development in Docker:

```bash
scripts/run_docker.sh web-dev
```

The native Python viewer uses the official MuJoCo viewer and is intended for
Linux desktops. On macOS, use the hosted browser demo or run the official viewer
inside the Linux container with XQuartz:

```bash
# one-time: install XQuartz, enable "Allow connections from network clients",
# then restart XQuartz
open -a XQuartz
xhost + 127.0.0.1
scripts/run_docker.sh viewer models/anvil_pedestal.xml
xhost - 127.0.0.1
```

For Linux desktops:

```bash
xhost +local:docker
scripts/run_docker.sh viewer models/anvil_pedestal.xml
xhost -local:docker
```

To validate the container without a visible display:

```bash
scripts/run_docker.sh viewer-smoke
```

## Models

| File | Contents |
|---|---|
| `models/anvil_openarm_bimanual.xml` | the bimanual arm + grippers, no scene |
| `models/anvil_pedestal.xml` | arm on a pedestal with floor and lighting |
| `models/anvil_openarm_bimanual.urdf` | the same bimanual arm as URDF (for Isaac/URDF consumers), derived from `enactic/openarm_description` with the same Anvil deltas |

All are **generated files** — edit `scripts/make_anvil_model.py` /
`scripts/make_anvil_urdf.py` (or upstream) and regenerate rather than editing
them directly. `tests/test_urdf_generation.py` holds the MJCF and URDF to
per-joint and full-kinematic-chain parity.

## What makes it the *Anvil* OpenARM 2.0

The live Anvil docs describe the OpenARM 2.0 wrist swap and a wider J6
radial/ulnar deviation range
([docs](https://docs.anvil.bot/introduction/openarm-2.0)), but do not publish a
machine-readable numeric controller contract. The side-specific signs below
are resolved from follower state in all 33 sessions of
`bohlt/openarm2-shirt-fold-phase-aligned-v1@8411e3e85eaf3e482b4ccb1cac9d4fc02891305e`:
right J6 reaches
-70.30° and left J6 reaches +63.46°. The nominal 70° envelope treats the
small encoder overrun as calibration/tolerance rather than expanding the
mechanical limit.

| Joint | Left controller range | Right controller range | Source |
|---|---:|---:|---|
| J1 | -200° to +80° | -80° to +200° | upstream OpenARM 2.0 |
| J2 | -190° to +10° | -10° to +190° | upstream OpenARM 2.0 |
| J3 | -90° to +90° | -90° to +90° | upstream OpenARM 2.0 |
| J4 | 0° to 140° | 0° to 140° | upstream OpenARM 2.0 |
| J5 | -90° to +90° | -90° to +90° | upstream OpenARM 2.0 |
| J6 (radial/ulnar deviation) | **-45° to +70°** | **-70° to +45°** | Anvil sessions |
| J7 (flexion/extension) | -90° to +90° | -90° to +90° | upstream OpenARM 2.0 |

The upstream v2 model already contains the OpenARM 2.0 wrist swap (J6 =
deviation, J7 = flexion/extension). The generator changes J6 only, applying
the side-specific range to both joint `range` and actuator `ctrlrange`:

- **left J6: -45° to +70°; right J6: -70° to +45°** — the wider direction is
  sign-mirrored in the controller coordinates recorded by the real robot
- **J1 and every other non-J6 joint remain upstream**
- **TCP sites** — `follower_l_hand_tcp` and `follower_r_hand_tcp`, using the
  upstream OpenArm v2 pinch-gripper grasp-frame transform so `/ee_pose_*`
  represents the end-effector TCP rather than the gripper base.
- **The red wrist bracket** — the C-bracket shown on the Anvil variant in the
  docs photo (it clamps the J6 rotor hub and lands on a plate bolted to the
  J7 motor end cap; it is the part that enables the extra +25 deg of
  deviation). Stock v2 meshes don't include it, so the generator emits a
  stylised **visual-only** approximation as inline MJCF meshes (one mirrored
  mesh per side, `anvil_wrist_bracket_{left,right}`) attached to the `link6`
  gimbal bodies. Dimensions live in `anvil_openarm_spec.WRIST_BRACKET_BOXES`
  and are validated by `scripts/check_model.py`.

### Sign conventions, verified

The numeric spec uses controller coordinates and matches upstream MJCF and
URDF numerics directly. J1 and J2 are already side-specific upstream. The
session evidence shows that J6 must also be side-specific: left
`range="-0.7854 1.2217"`, right `range="-1.2217 0.7854"`. Geometry remains
mirrored through the upstream joint axes; action and state values are never
silently sign-flipped.

## Layout

```
upstream/openarm_mujoco/   git submodule: enactic/openarm_mujoco (pristine)
upstream/openarm_description/  git submodule: enactic/openarm_description
                           (pristine); source for the generated URDF
upstream/anvil_loader/     git submodule: anvil-robotics/anvil-loader;
                           OpenArm v2 runtime profile YAML source
anvil_openarm_spec.py      shared local spec constants used by generation,
                           validation, tests, and the sim bridge
models/                    generated Anvil-variant MJCF (meshes referenced
                           from the submodule; nothing duplicated)
bridge/
  sim_core.py              ROS-free sim wrapper: real-robot-shaped position
                           commands (J1..J7 + gripper per arm), clamped to
                           the Anvil limits; SDLS-style local TCP IK for
                           commanded EE
  ros2_bridge.py           rclpy node mirroring the real robot's topics
docs/
  TESTING.md               full model, IK, ROS bridge, Quest teleop, and
                           hardware validation checklist
scripts/
  make_anvil_model.py      derives models/ from upstream; the Anvil delta
                           lives here, replacements fail loudly if upstream
                           changes shape
  make_anvil_urdf.py       same idea for the URDF: derives it from the
                           openarm_description submodule with the same spec
  check_model.py           validates ranges/ctrlranges/keyframes/stability
  view.py                  open a model in an interactive viewer
  demo_wrist_sweep.py      sweep both wrists through the Anvil ranges
                           (elbows raised; J6 sweep, J7 sweep, wrist circles)
  run_docker.sh            split Docker workflow for checks, web, viewer, ROS
  common.py                official MuJoCo viewer helper
tests/
  test_sim_core.py         unit tests, no ROS required
  test_ros2_bridge.py      bridge integration tests (auto-skip without rclpy)
docker/
  Dockerfile               multi-target Python, web, viewer, and ROS images
```

### Updating upstream

```bash
git -C upstream/openarm_mujoco pull origin master
git -C upstream/openarm_description pull origin main
uv run python scripts/make_anvil_model.py   # fails loudly on layout drift
uv run python scripts/make_anvil_urdf.py    # likewise for the URDF
uv run python scripts/check_model.py
```

## ROS 2 bridge

`bridge/ros2_bridge.py` steps the MuJoCo sim and exposes the real Anvil
robot's **joint-state, joint-command, and commanded-EE topic surface**
([technical reference](https://docs.anvil.bot/software/technical-reference/querying-robot-state)),
so scripts written against the simulation can carry over to hardware:

| Direction | Topic | Type | Notes |
|---|---|---|---|
| pub | `/joint_states` | `sensor_msgs/JointState` | position, velocity, effort for all 18 joints |
| pub | `/ee_pose_left`, `/ee_pose_right` | `geometry_msgs/PoseStamped` | TCP pose (`follower_{l,r}_hand_tcp`) in `world`; the **real robot uses Anvil's custom `CommandedEEPose`** here — adjust your subscriber type when moving to hardware |
| pub | `/clock` | `rosgraph_msgs/Clock` | simulation time |
| sub | `/follower_l_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 8 values, **ordered J1..J7 then `finger_joint1`** (7 accepted: gripper unchanged) |
| sub | `/follower_r_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | same, right arm |
| sub | `/commanded_ee_left`, `/commanded_ee_right` | `geometry_msgs/PoseStamped` by default; configurable to Anvil's custom `CommandedEEPose` | target TCP pose in `world`; custom `gripper` field is metres and maps to the mirrored MJCF gripper angle convention |

Parameters: `model_xml` (default `models/anvil_openarm_bimanual.xml`),
`publish_rate_hz` (200), `time_scale` (1.0 = realtime),
`commanded_ee_msg_type` (default `geometry_msgs/msg/PoseStamped`).

For Quest teleop, source the Anvil ROS workspace so the custom message package
is visible, then either set the exact type or ask the bridge to detect it once
the teleop publisher appears:

```bash
ros2 topic type /commanded_ee_left
python3 -m bridge.ros2_bridge --ros-args \
  -p commanded_ee_msg_type:=auto
# or, once you know the package:
python3 -m bridge.ros2_bridge --ros-args \
  -p commanded_ee_msg_type:=<anvil_pkg>/msg/CommandedEEPose
```

Anvil's public docs document the `CommandedEEPose` fields (`header`, `pose`,
`gripper`) but not the package name, so the exact `<anvil_pkg>` must come from
your devbox environment. With the default `PoseStamped` shim, the gripper is
left unchanged.

Two deliberate differences from raw hardware:

- **Commands are clamped to the Anvil 2.0 joint limits** (the real stack
  applies minimal safety constraints — Anvil's docs warn about this; the sim
  bridge refuses to exceed the mechanical ranges).
- Cartesian `/commanded_ee_*` commands use this repo's local SDLS-style IK,
  with target delta clamping, joint-specific velocity caps, and a nullspace
  posture/joint-limit bias. This follows Anvil's public IK description at the
  behavior level, but it is not Anvil's hardware IK engine and does not run TF
  transforms or reproduce the full controller stack.

### Inference integration status

This repo intentionally stops at the simulation and ROS command surface. It can
serve as a target for an external OpenArm v2 inference process if that process
publishes either:

- joint commands to `/follower_{l,r}_forward_position_controller/commands`, or
- Cartesian TCP commands to `/commanded_ee_{left,right}`.

It does **not** provide the inference process itself: no learned-policy model,
camera observation pipeline, dataset schema, action normalization layer, or
policy-specific controller is included. Those should live in a separate
training/inference repo or be added here as an explicit new subsystem once the
desired OpenArm v2 inference interface is pinned down.

Run it on a machine with ROS 2 (Jazzy or newer) and this repo checked out:

```bash
source /opt/ros/jazzy/setup.bash
pip install mujoco numpy          # alongside your ROS 2 python env
python3 -m bridge.ros2_bridge --ros-args -p time_scale:=1.0
# then, from another shell:
ros2 topic echo /joint_states --once
ros2 topic pub --once /follower_l_forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 1.0, 0, 0.5, 0, 0]}"
ros2 topic pub --once /commanded_ee_left geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.35, y: 0.2, z: 0.2}, orientation: {w: 1.0}}}"
```

If `python3 -m bridge.ros2_bridge` fails with `No module named 'rclpy'`, that
shell is not a sourced ROS 2 environment. `rclpy` is provided by ROS 2; it is
not installed by `uv sync` or the browser-demo workflow.

macOS has no native ROS 2. Use the hosted browser demo for local visual work,
run the bridge on the robot/devbox or another Linux ROS 2 machine, or use the
Docker harness:

```bash
scripts/run_docker.sh ros-bridge --ros-args -p time_scale:=1.0
```

On Linux, the Docker bridge uses host networking so other ROS 2 processes on
the host can discover it. On Docker Desktop for macOS, DDS discovery across the
host/container boundary is limited; use this mainly for local bridge execution
or run all ROS clients in the same Linux/container environment.

## Testing

The short version is below. The full checklist for model generation, IK
behavior, ROS topics, Quest teleop, and eventual hardware comparison is in
[docs/TESTING.md](/Users/bensonlee/dev/anvil-openarm-mujoco/docs/TESTING.md).

### In simulation

| What | Command | Needs |
|---|---|---|
| Model spec validation (joint/actuator ranges, TCP sites, keyframes, 2000-step stability) | `uv run python scripts/check_model.py` | nothing extra |
| Sim-core unit tests (command order, clamping, TCP pose, SDLS boundedness, velocity caps, nullspace bias, generation reproducibility) | `uv run pytest` | nothing extra (ROS tests auto-skip) |
| ROS 2 bridge runtime in Docker | `scripts/run_docker.sh ros-bridge --ros-args -p time_scale:=1.0` | Docker |
| ROS 2 bridge integration tests (topics publish, joint and commanded-EE commands move the arm, limits clamp, malformed commands rejected) | `scripts/run_docker.sh ros-test` | Docker |
| Official MuJoCo viewer container smoke test | `scripts/run_docker.sh viewer-smoke` | Docker |
| Wrist range-of-motion evidence (left J6 −45°..+70°, right J6 −70°..+45°, J7 ±90°) | `uv run python scripts/demo_wrist_sweep.py --headless` | nothing extra |
| Eyeball QA | `uv run python scripts/view.py`, `uv run python scripts/demo_wrist_sweep.py` | display |

The Docker suite builds focused targets for Python checks, the hosted web demo,
the official MuJoCo viewer, and ROS 2 Jazzy integration tests.

### On the real robot

The bridge mirrors the real topics, so the recommended flow is: prove your
client code against the sim bridge, then point it at the robot. Anvil's own
warning applies — the real stack imposes **minimal safety constraints** on
programmatic control
([docs](https://docs.anvil.bot/software/technical-reference/commanding-robot-movement)).

1. **Passive checks first** (robot on, Anvil stack running on the devbox,
   E-stop in reach, workspace clear):
   ```bash
   ros2 topic list                        # expect the same topics as the sim bridge
   ros2 topic echo /joint_states --once   # names, units, and ordering
   ```
   Move the arms by hand / teleop and watch `/joint_states` update. Note
   that **the real `/joint_states` ordering differs from the command
   ordering** (Anvil documents this footgun); always index by joint name.
2. **Interface parity**: run any client script against the sim bridge and
   confirm it behaves. The topic names match; `ee_pose_*` publishes
   `PoseStamped` in sim versus Anvil's custom state type on hardware, and
   `/clock` is sim-only. For Quest teleop, set
   `commanded_ee_msg_type:=auto` or the exact Anvil `CommandedEEPose` type so
   the subscription matches the teleop publisher.
3. **Small-delta commands**: read the current position from
   `/joint_states`, command *current + a few degrees* on one joint of one
   arm, and verify tracking before anything larger. Never command a pose far
   from the current one — the real controller does not interpolate for you.
4. **Joint-limit conformance** (the point of this repo): slowly step J6
   toward each limit on one arm. The real Anvil 2.0 wrist should reach
   approximately +70° on the left and -70° on the right; the sim clamps at
   those nominal limits. If your unit reaches a different limit, update
   `scripts/make_anvil_model.py` and regenerate so sim matches your
   hardware.
5. **Record and compare**: `ros2 bag record /joint_states` during a scripted
   motion on both sim and robot, and compare trajectories offline. Expect
   effort readings and contact-rich behavior to differ (sim actuator gains
   are upstream's, not identified from your unit); positions and limits
   should agree.
6. **Gripper**: joint commands are `finger_joint1` angle (left 0..+45°, right
   −45°..0 in the MJCF convention); Anvil's Cartesian interface instead uses
   meters of opening. The sim maps `CommandedEEPose.gripper` metres onto the
   mirrored MJCF finger-angle convention.

## Known approximations

- **Meshes are stock OpenArm v2 plus a stylised bracket.** Anvil's 2.0
  hardware adds a wrist bracket (and hard-case cabling); the bracket is
  represented by a generated, visual-only cuboid mesh sized off the upstream
  wrist meshes and eyeballed against the docs photo — it is not Anvil's CAD
  and takes part in no collision. Joint frame positions are assumed unchanged
  from standard v2 — Anvil describes the change as range-of-motion only.
- **J6 bounds are session-resolved nominal limits.** The 33-session dataset
  is treated as representative, but it is not a hard-stop calibration test.
  Re-run the range audit if controller offsets or robot firmware change.
- **Cartesian EE commands are approximate.** `/commanded_ee_*` on real
  hardware invokes Anvil's IK stack. The sim accepts the topics for Quest
  teleop compatibility, but resolves them with local SDLS-style IK against the
  MuJoCo model and only interprets targets in `world`. It is intended to mimic
  Anvil's documented behavior at a high level, not to reproduce Anvil's custom
  implementation exactly.

## Sources

- Anvil OpenARM 2.0 announcement: <https://docs.anvil.bot/introduction/openarm-2.0>
- Anvil OpenARM hardware docs: <https://docs.anvil.bot/hardware/openarm>
- Anvil IK and controls overview: <https://docs.anvil.bot/software/system-overview/inverse-kinematics-and-controls>
- Upstream MJCF: <https://github.com/enactic/openarm_mujoco> (Apache-2.0)
- Standard v2 description (URDF configs): <https://github.com/enactic/openarm_description>
  (Apache-2.0; vendored as the `upstream/openarm_description` submodule)

Upstream content is Copyright Enactic, Inc., Apache License 2.0 (see the
submodule's `LICENSE`); the generated models in `models/` are derivative works
and keep that license.

## License

This repo is licensed under the Apache License 2.0; see
[LICENSE](/Users/bensonlee/dev/anvil-openarm-mujoco/LICENSE). Generated MJCF
files under `models/` are derivative works of upstream
`enactic/openarm_mujoco`, which is also Apache-2.0 licensed. Anvil names,
product names, and trademarks belong to their respective owners.
