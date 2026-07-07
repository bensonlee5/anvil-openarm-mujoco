#!/usr/bin/env python
"""Validate the generated Anvil OpenARM 2.0 models against the local spec.

The live Anvil OpenARM 2.0 docs describe the wrist swap and wider J6
radial/ulnar deviation qualitatively. This repo keeps a local pre-arrival
numeric spec for the expected Anvil variant. It differs from standard/enactic
OpenArm v2 in exactly two joints per arm:

  - J1: -135 deg .. +135 deg   (standard v2: -80 .. +200 in MJCF convention)
  - J6: -45 deg .. +70 deg     (standard v2: -45 .. +45); J6 is radial/ulnar
                                deviation after the v2 wrist swap, and the
                                extra 25 deg is on the radial (+) side

The generated bimanual model also exposes follower_{l,r}_hand_tcp sites using
the upstream OpenArm v2 pinch-gripper grasp frame. Everything else must remain
identical to upstream. Exits non-zero on any violation.
"""

import contextlib
import io
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from anvil_openarm_spec import (  # noqa: E402
    ARM_JOINT_RANGES,
    DEG,
    MODEL_FILES,
    PATCHED_CTRLRANGES,
    SIDE_JOINT_RANGES,
    TCP_SITE_BODY_NAMES,
    TCP_SITE_NAMES,
    TCP_SITE_POS,
    TCP_SITE_QUAT,
    J6_AXIS_XZ_IN_LINK5,
    WRIST_BRACKET_BODY_NAMES,
    WRIST_BRACKET_LINK5_CYLINDERS,
    WRIST_BRACKET_MESH_AABB,
    WRIST_BRACKET_MESH_NAMES,
    WRIST_BRACKET_RGBA,
    WRIST_BRACKET_SCREW_CYLINDERS,
    WRIST_BRACKET_SCREW_RGBA,
    wrist_bracket_link5_geom_names,
    wrist_bracket_screw_geom_names,
)

MODELS_DIR = ROOT / "models"

TOL = 2e-3  # rad; upstream XML rounds to ~5 significant figures
TCP_TOL = 1e-6

failures: list[str] = []


def fail(msg: str) -> None:
    failures.append(msg)
    print(f"  FAIL  {msg}")


def ok(msg: str) -> None:
    print(f"  ok    {msg}")


def compile_model(path: Path, tag: str) -> mujoco.MjModel | None:
    compile_stderr = io.StringIO()
    try:
        with contextlib.redirect_stderr(compile_stderr):
            model = mujoco.MjModel.from_xml_path(str(path))
    except Exception as exc:  # noqa: BLE001 - report and continue
        fail(f"{tag}: failed to compile: {exc}")
        return None

    warning_text = " ".join(
        line.strip() for line in compile_stderr.getvalue().splitlines() if line.strip()
    )
    if warning_text:
        fail(f"{tag}: compile emitted warning(s): {warning_text}")
        return None

    return model


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
        ok(f"{tag}: all 14 arm joint ranges match the local Anvil 2.0 spec")

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


def check_tcp_sites(model: mujoco.MjModel, tag: str) -> None:
    before = len(failures)
    for side, site_name in TCP_SITE_NAMES.items():
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if sid < 0:
            fail(f"{tag}: TCP site '{site_name}' not found")
            continue
        body_name = TCP_SITE_BODY_NAMES[side]
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if bid < 0:
            fail(f"{tag}: TCP parent body '{body_name}' not found")
        elif model.site_bodyid[sid] != bid:
            got = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, int(model.site_bodyid[sid])
            )
            fail(
                f"{tag}: TCP site '{site_name}' is attached to '{got}', "
                f"expected '{body_name}'"
            )
        if not np.allclose(model.site_pos[sid], TCP_SITE_POS, atol=TCP_TOL):
            fail(
                f"{tag}: TCP site '{site_name}' pos is {model.site_pos[sid]}, "
                f"expected {TCP_SITE_POS}"
            )
        if not np.allclose(model.site_quat[sid], TCP_SITE_QUAT, atol=TCP_TOL):
            fail(
                f"{tag}: TCP site '{site_name}' quat is {model.site_quat[sid]}, "
                f"expected {TCP_SITE_QUAT}"
            )
    if len(failures) == before:
        ok(f"{tag}: follower hand TCP sites match the upstream v2 grasp frame")


def check_wrist_bracket(model: mujoco.MjModel, tag: str) -> None:
    """The Anvil wrist bracket: CAD STL mesh plus cylinder details per side.

    Checks presence, parenting, colour, non-collidability, that each side's
    mesh AABB matches the CAD-pinned bracket AABB (the right side mirrored in
    y via the negative mesh scale), that the screw/standoff cylinders match
    their source spec, and that the strap-side lug screw sits on the J6
    axis — so a generator regression or a stale STL fails loudly.
    """
    before = len(failures)
    lo, hi = (np.array(v) for v in WRIST_BRACKET_MESH_AABB)
    expected_aabb = {
        "l": (lo, hi),
        "r": (lo * np.array([1, -1, 1]), hi * np.array([1, -1, 1])),
    }
    for side, mesh_name in WRIST_BRACKET_MESH_NAMES.items():
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, mesh_name)
        if gid < 0:
            fail(f"{tag}: wrist bracket geom '{mesh_name}' not found")
            continue
        body_name = WRIST_BRACKET_BODY_NAMES[side]
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if model.geom_bodyid[gid] != bid:
            got = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[gid])
            )
            fail(f"{tag}: bracket '{mesh_name}' attached to '{got}', expected '{body_name}'")
        if model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH:
            fail(f"{tag}: bracket '{mesh_name}' is not a mesh geom")
            continue
        if model.geom_contype[gid] != 0 or model.geom_conaffinity[gid] != 0:
            fail(f"{tag}: bracket '{mesh_name}' must be visual-only (contype/conaffinity 0)")
        mat_id = model.geom_matid[gid]
        rgba = model.mat_rgba[mat_id] if mat_id >= 0 else model.geom_rgba[gid]
        if not np.allclose(rgba, WRIST_BRACKET_RGBA, atol=1e-6):
            fail(f"{tag}: bracket '{mesh_name}' rgba is {rgba}, expected {WRIST_BRACKET_RGBA}")
        mesh_id = model.geom_dataid[gid]
        va, vn = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
        verts = model.mesh_vert[va : va + vn].reshape(-1, 3)
        # MuJoCo re-centers user meshes at their CoM and rotates them into
        # the principal inertia frame, compensating via the geom pos/quat —
        # map the vertices back into the body (link5) frame before comparing.
        rot = np.zeros(9)
        mujoco.mju_quat2Mat(rot, model.geom_quat[gid])
        verts = verts @ rot.reshape(3, 3).T + model.geom_pos[gid]
        exp_lo, exp_hi = expected_aabb[side]
        got_lo = verts.min(axis=0)
        got_hi = verts.max(axis=0)
        # atol covers the pinned spec AABB being rounded to 0.5 mm vs the
        # tessellated STL extents.
        if not (
            np.allclose(np.minimum(got_lo, got_hi), np.minimum(exp_lo, exp_hi), atol=5e-4)
            and np.allclose(np.maximum(got_lo, got_hi), np.maximum(exp_lo, exp_hi), atol=5e-4)
        ):
            fail(
                f"{tag}: bracket '{mesh_name}' AABB [{got_lo}, {got_hi}] does not "
                f"match the CAD-pinned bracket AABB [{exp_lo}, {exp_hi}]"
            )

    def check_cylinder_specs(
        side: str,
        body_name: str,
        cylinders: dict,
        geom_names: dict,
        rgba: tuple,
        label: str,
    ) -> None:
        sy = 1.0 if side == "l" else -1.0
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        for key, gname in geom_names.items():
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, gname)
            if gid < 0:
                fail(f"{tag}: wrist bracket {label} geom '{gname}' not found")
                continue
            if model.geom_bodyid[gid] != bid:
                got = mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[gid])
                )
                fail(f"{tag}: {label} '{gname}' attached to '{got}', expected '{body_name}'")
            if model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_CYLINDER:
                fail(f"{tag}: {label} '{gname}' is not a cylinder geom")
                continue
            if model.geom_contype[gid] != 0 or model.geom_conaffinity[gid] != 0:
                fail(f"{tag}: {label} '{gname}' must be visual-only (contype/conaffinity 0)")
            mat_id = model.geom_matid[gid]
            got_rgba = model.mat_rgba[mat_id] if mat_id >= 0 else model.geom_rgba[gid]
            if not np.allclose(got_rgba, rgba, atol=1e-6):
                fail(f"{tag}: {label} '{gname}' rgba is {got_rgba}, expected {rgba}")
            start, end, radius = cylinders[key]
            start = np.array(start) * [1.0, sy, 1.0]
            end = np.array(end) * [1.0, sy, 1.0]
            direction = (end - start) / np.linalg.norm(end - start)
            rot = np.zeros(9)
            mujoco.mju_quat2Mat(rot, model.geom_quat[gid])
            zaxis = rot.reshape(3, 3)[:, 2]
            half_len = np.linalg.norm(end - start) / 2
            if not (
                np.allclose(model.geom_pos[gid], (start + end) / 2, atol=1e-6)
                and np.allclose(model.geom_size[gid][:2], [radius, half_len], atol=1e-6)
                and abs(float(zaxis @ direction)) > 1 - 1e-6
            ):
                fail(f"{tag}: {label} '{gname}' does not match the spec cylinder")

    for side in WRIST_BRACKET_BODY_NAMES:
        check_cylinder_specs(
            side,
            WRIST_BRACKET_BODY_NAMES[side],
            WRIST_BRACKET_SCREW_CYLINDERS,
            wrist_bracket_screw_geom_names(side),
            WRIST_BRACKET_SCREW_RGBA,
            "fastener",
        )
        check_cylinder_specs(
            side,
            WRIST_BRACKET_BODY_NAMES[side],
            WRIST_BRACKET_LINK5_CYLINDERS,
            wrist_bracket_link5_geom_names(side),
            WRIST_BRACKET_RGBA,
            "forearm standoff",
        )
        # The strap-side lug is the outboard bearing for the gimbal: its
        # screw's centreline must lie exactly on the J6 axis, which runs
        # along y through link5 (x, z) = J6_AXIS_XZ_IN_LINK5.
        gname = wrist_bracket_screw_geom_names(side)["lug_j6_screw"]
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, gname)
        if gid >= 0 and not np.allclose(
            np.delete(model.geom_pos[gid], 1), J6_AXIS_XZ_IN_LINK5, atol=1e-9
        ):
            fail(
                f"{tag}: '{gname}' centreline is off the J6 axis "
                f"(pos {model.geom_pos[gid]})"
            )
    if len(failures) == before:
        ok(
            f"{tag}: Anvil wrist bracket present on both link5 forearm bodies "
            "(visual-only CAD STL, fasteners, and forearm standoffs; bearing "
            "lug on the J6 axis)"
        )


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
        model = compile_model(path, fname)
        if model is None:
            continue
        ok(f"compiles ({model.njnt} joints, {model.nu} actuators)")
        check_joint_ranges(model, fname)
        check_actuators(model, fname)
        check_tcp_sites(model, fname)
        check_wrist_bracket(model, fname)
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
