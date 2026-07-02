"""Physics regression tests for the exported browser demo scenes.

The hosted "Bimanual Arm" demo uses the exporter-authored scene.xml, and the
"Full Range of Motion" demo sweeps every joint through its full ctrlrange on
that stage. These tests pin down:

  1. the arm spawns clear of the floor and settles (no contact explosion)
  2. every joint can sweep its full range without instability

Run:  uv run pytest tests/test_web_scene.py
"""

import math
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
SCENE = ROOT / "web" / "public" / "sim-assets" / "anvil_openarm" / "scene.xml"

# posture targets held during each joint's sweep so full-range motion stays
# clear of the floor and the other arm (mirrors the web fullRom script)
SWEEP_POSTURE_J4 = {
    "joint5": math.pi / 2,
    "joint6": math.pi / 2,
    "joint7": math.pi / 2,
    "finger1": math.pi / 2,
}
SWEEP_KEYS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
    "finger1",
]


@pytest.fixture(scope="module")
def scene_model():
    subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "export_web_assets.py")],
        check=True,
        capture_output=True,
    )
    return mujoco.MjModel.from_xml_path(str(SCENE))


def reset(model):
    data = mujoco.MjData(model)
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if kid >= 0:
        mujoco.mj_resetDataKeyframe(model, data, kid)
        data.ctrl[:] = model.key_ctrl[kid]
    mujoco.mj_forward(model, data)
    return data


def actuator(model, name):
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    assert aid >= 0, f"actuator '{name}' missing"
    return aid


def test_scene_spawns_clear_of_floor(scene_model):
    data = reset(scene_model)
    penetrations = [
        data.contact[i].dist for i in range(data.ncon) if data.contact[i].dist < -1e-4
    ]
    assert not penetrations, (
        f"{len(penetrations)} penetrating contacts at spawn "
        f"(worst {min(penetrations):.4f} m) — arm intersects the scene"
    )


def test_scene_settles_calm(scene_model):
    data = reset(scene_model)
    for _ in range(round(3.0 / scene_model.opt.timestep)):
        mujoco.mj_step(scene_model, data)
    assert np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))
    peak = float(np.max(np.abs(data.qvel)))
    assert peak < 0.5, f"scene still moving after 3 s (|qvel|max={peak:.3f} rad/s)"


def test_scene_uses_one_ms_timestep(scene_model):
    assert scene_model.opt.timestep == pytest.approx(0.001)


def test_full_rom_sweep_every_joint(scene_model):
    """Sweep each actuated joint (both arms together) through its full
    ctrlrange; the sim must stay finite and reach >=95% of the range."""
    model = scene_model
    data = reset(model)
    sweep_seconds = 6.0
    dt = model.opt.timestep

    for key in SWEEP_KEYS:
        aids = [actuator(model, f"{side}_{key}_ctrl") for side in ("left", "right")]
        posture = [
            actuator(model, f"{side}_joint4_ctrl") for side in ("left", "right")
        ]
        posture_target = SWEEP_POSTURE_J4.get(key)

        achieved = {aid: [math.inf, -math.inf] for aid in aids}
        nsteps = round(sweep_seconds / dt)
        for step in range(nsteps):
            s = (step + 1) / nsteps
            blend = min(1.0, 10 * s * (1 - s) * 2)  # ease in/out
            for aid in posture:
                lo, hi = model.actuator_ctrlrange[aid]
                base = float(np.clip(posture_target or 0.0, lo, hi))
                data.ctrl[aid] = base if posture_target is not None else data.ctrl[aid]
            for aid in aids:
                lo, hi = model.actuator_ctrlrange[aid]
                mid, amp = (lo + hi) / 2, (hi - lo) / 2
                data.ctrl[aid] = mid + amp * math.sin(2 * math.pi * s) * blend
            mujoco.mj_step(model, data)
            for aid in aids:
                jid = model.actuator_trnid[aid, 0]
                q = data.qpos[model.jnt_qposadr[jid]]
                achieved[aid][0] = min(achieved[aid][0], q)
                achieved[aid][1] = max(achieved[aid][1], q)

        assert np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)), (
            f"non-finite state while sweeping {key}"
        )
        peak = float(np.max(np.abs(data.qvel)))
        assert peak < 25.0, f"instability while sweeping {key}: |qvel|max={peak:.1f}"

        for aid in aids:
            lo, hi = model.actuator_ctrlrange[aid]
            got_lo, got_hi = achieved[aid]
            span = hi - lo
            coverage = (min(got_hi, hi) - max(got_lo, lo)) / span
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aid)
            assert coverage >= 0.95, (
                f"{name}: swept [{got_lo:.3f}, {got_hi:.3f}] covers only "
                f"{coverage:.0%} of [{lo:.3f}, {hi:.3f}]"
            )

        # settle back to neutral between joints
        for aid in aids + posture:
            lo, hi = model.actuator_ctrlrange[aid]
            data.ctrl[aid] = float(np.clip(0.0, lo, hi))
        for _ in range(round(1.5 / dt)):
            mujoco.mj_step(model, data)
