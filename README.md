# anvil-openarm-mujoco

MuJoCo simulation workspace for the **Anvil OpenARM 2.0** — Anvil's variant of
the [OpenArm](https://github.com/enactic/openarm) bimanual arm with an
extended-range wrist.

The models are derived from the upstream
[enactic/openarm_mujoco](https://github.com/enactic/openarm_mujoco) v2 MJCF
files (vendored as a git submodule) with the Anvil joint changes applied by a
generator script.

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

Joint-limit comparison published by Anvil
([docs](https://docs.anvil.bot/introduction/openarm-2.0)):

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
- **J1: ±135°** — per Anvil's table (standard v2 uses -80°..+200° in MJCF
  sign convention)

### Sign conventions, verified

Anvil's table is written in the convention that matches the upstream MJCF
*left*-arm numerics exactly — verified on both asymmetric joints
(J1: table "-200..+80" = left `range="-3.4907 1.3963"`; J2: table "-190..+10"
= left `range="-3.3161 0.17453"`). J6's "-45..+70" therefore maps to
`range="-0.7854 1.2217"` on the left arm. Upstream encodes left/right
mirroring for J6 as *flipped axis + identical numeric range*, so the right arm
gets the same numbers; this was confirmed numerically (fingertip displacement
at left +q exactly equals right -q).

## Layout

```
upstream/openarm_mujoco/   git submodule: enactic/openarm_mujoco (pristine)
models/                    generated Anvil-variant MJCF (meshes referenced
                           from the submodule; nothing duplicated)
scripts/
  make_anvil_model.py      derives models/ from upstream; the Anvil delta
                           lives here, replacements fail loudly if upstream
                           changes shape
  check_model.py           validates ranges/ctrlranges/keyframes/stability
  view.py                  open a model in an interactive viewer
  demo_wrist_sweep.py      sweep both wrists through the Anvil ranges
                           (elbows raised; J6 sweep, J7 sweep, wrist circles)
  common.py                viewer selection + mjpython workarounds
```

### Updating upstream

```bash
git -C upstream/openarm_mujoco pull origin master
uv run python scripts/make_anvil_model.py   # fails loudly on layout drift
uv run python scripts/check_model.py
```

## Known approximations

- **Meshes are stock OpenArm v2.** Anvil's 2.0 hardware adds a wrist bracket
  (and hard-case cabling) that the visual/collision meshes don't show. Joint
  frame positions are assumed unchanged from standard v2 — Anvil describes
  the change as range-of-motion only.
- **J1 = ±135° follows Anvil's published table.** Anvil's v1.0 URDF
  (`anvil-robotics/openarm_description`) allows -80°..+200° for J1, so the
  table value may be an operational rather than mechanical limit. If you want
  the wider range, change `J1_RANGE`/`J1_CTRLRANGE` in
  `scripts/make_anvil_model.py` and regenerate.
- `anvil_demo.xml` inherits an upstream quirk: the demo attaches the cell
  model without pinning `timestep`, so MuJoCo warns and runs at 2 ms instead
  of the cell's 1 ms.

## Sources

- Anvil OpenARM 2.0 announcement + joint table: <https://docs.anvil.bot/introduction/openarm-2.0>
- Anvil OpenARM hardware docs: <https://docs.anvil.bot/hardware/openarm>
- Upstream MJCF: <https://github.com/enactic/openarm_mujoco> (Apache-2.0)
- Standard v2 description (URDF configs): <https://github.com/enactic/openarm_description>

Upstream content is Copyright Enactic, Inc., Apache License 2.0 (see the
submodule's `LICENSE`); the generated models in `models/` are derivative works
and keep that license.
