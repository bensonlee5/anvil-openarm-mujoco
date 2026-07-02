# anvil-openarm-mujoco

MuJoCo simulation workspace for the **Anvil OpenARM 2.0** — Anvil's variant of
the [OpenArm](https://github.com/enactic/openarm) bimanual arm with an
extended-range wrist.

What's in this repo:

- **MJCF models** of the Anvil OpenARM 2.0 (bimanual arm, pedestal, workcell,
  and manipulation-demo scenes), derived from the upstream
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

## Quick start

```bash
git submodule update --init      # first checkout only
uv sync                          # installs mujoco (incl. mjpython on macOS)

uv run python scripts/make_anvil_model.py   # regenerate models/ from upstream
uv run python scripts/check_model.py        # validate against the Anvil spec

uv run python scripts/view.py                          # demo scene in the viewer
uv run python scripts/view.py models/anvil_pedestal.xml
uv run python scripts/demo_wrist_sweep.py              # wrist range-of-motion demo
uv run python scripts/demo_wrist_sweep.py --headless   # same, no GUI, prints ranges
```

### Viewer on macOS

The stock MuJoCo viewer (`mjpython` / `_Simulate`) currently crashes with
`RuntimeError: Caught an unknown exception!` on this machine's macOS
(Darwin 25) — reproduced with mujoco 3.7–3.10 on uv-managed, python.org
framework, and Terminal-launched Python alike. The scripts therefore default
to [mujoco-python-viewer](https://github.com/rohanpsingh/mujoco-python-viewer)
on macOS, which works. Pass `--official` to `view.py` /
`demo_wrist_sweep.py` to try the stock viewer (worth re-testing after MuJoCo
updates; `scripts/common.py` also self-heals the known
[mjpython-under-uv dlopen issue](https://github.com/google-deepmind/mujoco/issues/1923)
by symlinking `libpython` into `.venv/lib`).

## Models

| File | Contents |
|---|---|
| `models/anvil_openarm_bimanual.xml` | the bimanual arm + grippers, no scene |
| `models/anvil_pedestal.xml` | arm on a pedestal with floor and lighting |
| `models/anvil_cell.xml` | arm in the OpenArm workcell (lifter, sheet, walls) |
| `models/anvil_demo.xml` | workcell + manipulable props (cube, tray) |

All are **generated files** — edit `scripts/make_anvil_model.py` (or upstream)
and regenerate rather than editing them directly.

## What makes it the *Anvil* OpenARM 2.0

The live Anvil docs describe the OpenARM 2.0 wrist swap and a wider J6
radial/ulnar deviation range qualitatively
([docs](https://docs.anvil.bot/introduction/openarm-2.0)), but they no longer
expose an inline numeric comparison table. This repo keeps the following local
pre-arrival numeric spec until the hardware can be measured or Anvil publishes
updated machine-readable limits:

| Joint | Anvil OpenARM 1.0 | Standard OpenARM 2.0 | **Anvil OpenARM 2.0** |
|---|---|---|---|
| J1 | -135° to +135° | -200° to +80° | **-135° to +135°** |
| J2 | -190° to +10° | -190° to +10° | -190° to +10° |
| J3 | -90° to +90° | -90° to +90° | -90° to +90° |
| J4 | 0° to 140° | 0° to 140° | 0° to 140° |
| J5 | -90° to +90° | -90° to +90° | -90° to +90° |
| J6 (radial/ulnar deviation) | -45° to +45° (flex/ext in 1.0) | -45° to +45° | **-45° to +70°** |
| J7 (flexion/extension) | -90° to +90° (deviation in 1.0) | -90° to +90° | -90° to +90° |
| Gripper | -45° to 0° | -45° to 0° | -45° to 0° |

The upstream v2 model already contains the OpenARM 2.0 wrist swap (J6 =
deviation, J7 = flexion/extension). The generator applies the two Anvil
deltas on both arms, to joint `range` and actuator `ctrlrange`:

- **J6: -45° to +70°** — the extra +25° of deviation enabled by Anvil's
  wrist bracket
- **J1: ±135°** — per the local spec (standard v2 uses -80°..+200° in MJCF
  sign convention)
- **TCP sites** — `follower_l_hand_tcp` and `follower_r_hand_tcp`, using the
  upstream OpenArm v2 pinch-gripper grasp-frame transform so `/ee_pose_*`
  represents the end-effector TCP rather than the gripper base.

### Sign conventions, verified

The numeric spec is written in the convention that matches the upstream MJCF
*left*-arm numerics exactly — verified on both asymmetric joints
(J1: spec "-200..+80" = left `range="-3.4907 1.3963"`; J2: spec "-190..+10"
= left `range="-3.3161 0.17453"`). J6's "-45..+70" therefore maps to
`range="-0.7854 1.2217"` on the left arm. Upstream encodes left/right
mirroring for J6 as *flipped axis + identical numeric range*, so the right arm
gets the same numbers; this was confirmed numerically (fingertip displacement
at left +q exactly equals right -q).

## Layout

```
upstream/openarm_mujoco/   git submodule: enactic/openarm_mujoco (pristine)
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
  check_model.py           validates ranges/ctrlranges/keyframes/stability
  view.py                  open a model in an interactive viewer
  demo_wrist_sweep.py      sweep both wrists through the Anvil ranges
                           (elbows raised; J6 sweep, J7 sweep, wrist circles)
  run_ros2_tests.sh        dockerized ROS 2 integration test suite
  common.py                viewer selection + mjpython workarounds
tests/
  test_sim_core.py         unit tests, no ROS required
  test_ros2_bridge.py      bridge integration tests (auto-skip without rclpy)
docker/
  Dockerfile.ros2-test     ROS 2 Jazzy image that runs the full test suite
```

### Updating upstream

```bash
git -C upstream/openarm_mujoco pull origin master
uv run python scripts/make_anvil_model.py   # fails loudly on layout drift
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

Run it on a machine with ROS 2 (Jazzy or newer) and this repo checked out:

```bash
pip install mujoco numpy          # alongside your ROS 2 python env
python3 -m bridge.ros2_bridge --ros-args -p time_scale:=1.0
# then, from another shell:
ros2 topic echo /joint_states --once
ros2 topic pub --once /follower_l_forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0, 0, 0, 1.0, 0, 0.5, 0, 0]}"
ros2 topic pub --once /commanded_ee_left geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {position: {x: 0.35, y: 0.2, z: 0.2}, orientation: {w: 1.0}}}"
```

macOS has no native ROS 2; use the Docker harness below (tests) or run the
bridge inside your own ROS 2 container/VM with the repo mounted.

## Testing

The short version is below. The full checklist for model generation, IK
behavior, ROS topics, Quest teleop, and eventual hardware comparison is in
[docs/TESTING.md](/Users/bensonlee/dev/anvil-openarm-mujoco/docs/TESTING.md).

### In simulation

| What | Command | Needs |
|---|---|---|
| Model spec validation (joint/actuator ranges, TCP sites, keyframes, 2000-step stability) | `uv run python scripts/check_model.py` | nothing extra |
| Sim-core unit tests (command order, clamping, TCP pose, SDLS boundedness, velocity caps, nullspace bias, generation reproducibility) | `uv run pytest` | nothing extra (ROS tests auto-skip) |
| ROS 2 bridge integration tests (topics publish, joint and commanded-EE commands move the arm, limits clamp, malformed commands rejected) | `scripts/run_ros2_tests.sh` | Docker |
| Wrist range-of-motion evidence (both arms achieve J6 −45°..+70°, J7 ±90° under position control) | `uv run python scripts/demo_wrist_sweep.py --headless` | nothing extra |
| Eyeball QA | `uv run python scripts/view.py`, `uv run python scripts/demo_wrist_sweep.py` | display |

The Docker suite builds a ROS 2 Jazzy image, installs MuJoCo alongside the
apt rclpy stack, and runs **all** tests (the sim-core tests run twice: once
locally via uv, once inside the container — cheap and catches
python-version drift).

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
   ≈ +70° of deviation and stop; the sim clamps at exactly +70°. If your
   unit reaches a different limit (e.g. a standard v2 wrist at ±45°, or a
   J1 range of −80°..+200° instead of ±135°), update
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

- **Meshes are stock OpenArm v2.** Anvil's 2.0 hardware adds a wrist bracket
  (and hard-case cabling) that the visual/collision meshes don't show. Joint
  frame positions are assumed unchanged from standard v2 — Anvil describes
  the change as range-of-motion only.
- **J1 = ±135° follows this repo's local pre-arrival spec.** Anvil's public
  `openarm_description` repository and upstream standard v2 allow
  -80°..+200° for J1, so ±135° may be an operational rather than mechanical
  limit. If hardware measurement shows the wider range, update
  `anvil_openarm_spec.py` and regenerate.
- **Cartesian EE commands are approximate.** `/commanded_ee_*` on real
  hardware invokes Anvil's IK stack. The sim accepts the topics for Quest
  teleop compatibility, but resolves them with local SDLS-style IK against the
  MuJoCo model and only interprets targets in `world`.
- `anvil_demo.xml` inherits an upstream quirk: the demo attaches the cell
  model without pinning `timestep`, so MuJoCo warns and runs at 2 ms instead
  of the cell's 1 ms.

## Sources

- Anvil OpenARM 2.0 announcement: <https://docs.anvil.bot/introduction/openarm-2.0>
- Anvil OpenARM hardware docs: <https://docs.anvil.bot/hardware/openarm>
- Anvil IK and controls overview: <https://docs.anvil.bot/software/system-overview/inverse-kinematics-and-controls>
- Upstream MJCF: <https://github.com/enactic/openarm_mujoco> (Apache-2.0)
- Standard v2 description (URDF configs): <https://github.com/enactic/openarm_description>

Upstream content is Copyright Enactic, Inc., Apache License 2.0 (see the
submodule's `LICENSE`); the generated models in `models/` are derivative works
and keep that license.
