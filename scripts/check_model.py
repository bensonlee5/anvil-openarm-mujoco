#!/usr/bin/env python
"""Validate the generated Anvil OpenARM 2.0 models against the published spec.

Spec source: https://docs.anvil.bot/introduction/openarm-2.0 (joint-limit
comparison table). Anvil OpenARM 2.0 differs from the standard/enactic
OpenArm v2 in exactly two joints per arm:

  - J1: -135 deg .. +135 deg   (standard v2: -80 .. +200 in MJCF convention)
  - J6: -45 deg .. +70 deg     (standard v2: -45 .. +45); J6 is radial/ulnar
                                deviation after the v2 wrist swap, and the
                                extra 25 deg is on the radial (+) side

Everything else must remain identical to upstream. Exits non-zero on any
violation.
"""

import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = ROOT / "models"

DEG = math.pi / 180.0
TOL = 2e-3  # rad; upstream XML rounds to ~5 significant figures

# Symmetric-per-arm joint ranges (same numbers for left/right because the
# upstream MJCF mirrors the joint axes between arms).
ARM_JOINT_RANGES = {
    "joint1": (-135 * DEG, 135 * DEG),  # Anvil keeps 1.0-style symmetric J1
    "joint3": (-90 * DEG, 90 * DEG),
    "joint4": (0.0, 140 * DEG),
    "joint5": (-90 * DEG, 90 * DEG),
    "joint6": (-45 * DEG, 70 * DEG),  # Anvil extended deviation range
    "joint7": (-90 * DEG, 90 * DEG),  # flexion/extension after v2 swap
}

# J2 is asymmetric and sign-mirrored between arms in the upstream convention.
SIDE_JOINT_RANGES = {
    "openarm_left_joint2": (-190 * DEG, 10 * DEG),
    "openarm_right_joint2": (-10 * DEG, 190 * DEG),
}

# Actuator ctrlranges that the generator must patch alongside the joints.
PATCHED_CTRLRANGES = {
    "left_joint1_ctrl": (-135 * DEG, 135 * DEG),
    "right_joint1_ctrl": (-135 * DEG, 135 * DEG),
    "left_joint6_ctrl": (-45 * DEG, 70 * DEG),
    "right_joint6_ctrl": (-45 * DEG, 70 * DEG),
}

MODEL_FILES = [
    "anvil_openarm_bimanual.xml",
    "anvil_cell.xml",
    "anvil_demo.xml",
    "anvil_pedestal.xml",
]

failures: list[str] = []


def fail(msg: str) -> None:
    failures.append(msg)
    print(f"  FAIL  {msg}")


def ok(msg: str) -> None:
    print(f"  ok    {msg}")


def approx(a: float, b: float) -> bool:
    return abs(a - b) <= TOL


def check_joint_ranges(model: mujoco.MjModel, tag: str) -> None:
    expected: dict[str, tuple[float, float]] = {}
    for side in ("left", "right"):
        for jname, rng in ARM_JOINT_RANGES.items():
            expected[f"openarm_{side}_{jname}"] = rng
    expected.update(SIDE_JOINT_RANGES)

    before = len(failures)
    for name, (lo, hi) in expected.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            fail(f"{tag}: joint '{name}' not found")
            continue
        got_lo, got_hi = model.jnt_range[jid]
        if not (approx(got_lo, lo) and approx(got_hi, hi)):
            fail(
                f"{tag}: {name} range is [{got_lo:.4f}, {got_hi:.4f}] rad, "
                f"expected [{lo:.4f}, {hi:.4f}] "
                f"([{lo / DEG:.0f}, {hi / DEG:.0f}] deg)"
            )
    if len(failures) == before:
        ok(f"{tag}: all 14 arm joint ranges match the Anvil 2.0 table")

    # Bimanual mirror consistency: identical numeric ranges on mirrored axes.
    for jname in ARM_JOINT_RANGES:
        lid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_left_{jname}"
        )
        rid = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"openarm_right_{jname}"
        )
        if lid >= 0 and rid >= 0 and not np.allclose(
            model.jnt_range[lid], model.jnt_range[rid], atol=1e-9
        ):
            fail(f"{tag}: left/right '{jname}' ranges differ numerically")


def check_actuators(model: mujoco.MjModel, tag: str) -> None:
    before = len(failures)
    for name, (lo, hi) in PATCHED_CTRLRANGES.items():
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid < 0:
            fail(f"{tag}: actuator '{name}' not found")
            continue
        got_lo, got_hi = model.actuator_ctrlrange[aid]
        if not (approx(got_lo, lo) and approx(got_hi, hi)):
            fail(
                f"{tag}: {name} ctrlrange is [{got_lo:.4f}, {got_hi:.4f}], "
                f"expected [{lo:.4f}, {hi:.4f}]"
            )

    # Every joint-position actuator's ctrlrange must lie within its joint range.
    for aid in range(model.nu):
        if model.actuator_trntype[aid] != mujoco.mjtTrn.mjTRN_JOINT:
            continue
        if not model.actuator_ctrllimited[aid]:
            continue
        jid = model.actuator_trnid[aid, 0]
        if not model.jnt_limited[jid]:
            continue
        clo, chi = model.actuator_ctrlrange[aid]
        jlo, jhi = model.jnt_range[jid]
        if clo < jlo - TOL or chi > jhi + TOL:
            aname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aid)
            fail(
                f"{tag}: actuator '{aname}' ctrlrange [{clo:.4f}, {chi:.4f}] "
                f"exceeds joint range [{jlo:.4f}, {jhi:.4f}]"
            )
    if len(failures) == before:
        ok(f"{tag}: actuator ctrlranges patched and within joint limits")


def check_keyframes(model: mujoco.MjModel, tag: str) -> None:
    for kid in range(model.nkey):
        kname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, kid) or str(kid)
        qpos = model.key_qpos[kid]
        for jid in range(model.njnt):
            if not model.jnt_limited[jid]:
                continue
            if model.jnt_type[jid] not in (
                mujoco.mjtJoint.mjJNT_HINGE,
                mujoco.mjtJoint.mjJNT_SLIDE,
            ):
                continue
            adr = model.jnt_qposadr[jid]
            lo, hi = model.jnt_range[jid]
            if qpos[adr] < lo - 1e-6 or qpos[adr] > hi + 1e-6:
                jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
                fail(
                    f"{tag}: keyframe '{kname}' puts joint '{jname}' at "
                    f"{qpos[adr]:.4f}, outside [{lo:.4f}, {hi:.4f}]"
                )
    if model.nkey:
        ok(f"{tag}: {model.nkey} keyframe(s) within joint limits")


def check_stability(model: mujoco.MjModel, tag: str, nsteps: int = 2000) -> None:
    data = mujoco.MjData(model)
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if kid >= 0:
        mujoco.mj_resetDataKeyframe(model, data, kid)
        data.ctrl[:] = model.key_ctrl[kid]
    else:
        mujoco.mj_resetData(model, data)
    for _ in range(nsteps):
        mujoco.mj_step(model, data)
    if not (np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))):
        fail(f"{tag}: non-finite state after {nsteps} steps")
        return
    peak = float(np.max(np.abs(data.qvel))) if model.nv else 0.0
    if peak > 5.0:
        fail(f"{tag}: still moving fast after {nsteps} steps (|qvel|max={peak:.2f})")
        return
    ok(f"{tag}: stable after {nsteps} steps (|qvel|max={peak:.3f})")


def main() -> int:
    if not MODELS_DIR.is_dir():
        print(f"FAIL: models directory not found: {MODELS_DIR}")
        print("Run: uv run python scripts/make_anvil_model.py")
        return 1

    for fname in MODEL_FILES:
        path = MODELS_DIR / fname
        print(f"\n== {fname} ==")
        if not path.is_file():
            fail(f"missing file {path}")
            continue
        try:
            model = mujoco.MjModel.from_xml_path(str(path))
        except Exception as exc:  # noqa: BLE001 - report and continue
            fail(f"{fname}: failed to compile: {exc}")
            continue
        ok(f"compiles ({model.njnt} joints, {model.nu} actuators)")
        check_joint_ranges(model, fname)
        check_actuators(model, fname)
        check_keyframes(model, fname)
        check_stability(model, fname)

    print()
    if failures:
        print(f"RESULT: {len(failures)} failure(s)")
        return 1
    print("RESULT: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
