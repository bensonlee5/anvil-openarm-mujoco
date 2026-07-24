#!/usr/bin/env python
"""Sweep the Anvil OpenARM 2.0 wrists through their full range of motion.

Cycles through three phases on both arms, reading the limits from the model's
actuator ctrlranges (so it always demonstrates exactly the Anvil ranges):

  1. J6 radial/ulnar deviation sweep  (side-specific mirrored extended range)
  2. J7 flexion/extension sweep       (-90 deg .. +90 deg)
  3. combined circular wrist motion

Usage:
    uv run python scripts/demo_wrist_sweep.py [models/anvil_pedestal.xml]
"""

import argparse
import math

import mujoco

from common import ROOT, interactive_loop

PHASE_SECONDS = 6.0
BLEND_SECONDS = 1.0
SETTLE_SECONDS = 2.0  # raise the elbows clear of the pedestal before sweeping


def smoothstep(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "xml", nargs="?", default=str(ROOT / "models" / "anvil_pedestal.xml")
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run one full cycle without the viewer and report achieved ranges",
    )
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if kid >= 0:
        mujoco.mj_resetDataKeyframe(model, data, kid)
        data.ctrl[:] = model.key_ctrl[kid]
    home_ctrl = data.ctrl.copy()

    def act(name: str) -> int:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid < 0:
            raise SystemExit(f"actuator '{name}' not found in {args.xml}")
        return aid

    # bend the elbows 90 deg so the wrists sweep in free space instead of
    # brushing the pedestal
    for side in ("left", "right"):
        home_ctrl[act(f"{side}_joint4_ctrl")] = math.radians(90)

    j6 = [act("left_joint6_ctrl"), act("right_joint6_ctrl")]
    j7 = [act("left_joint7_ctrl"), act("right_joint7_ctrl")]
    for label, aids in (("J6 (deviation)", j6), ("J7 (flexion/extension)", j7)):
        for aid in aids:
            lo, hi = model.actuator_ctrlrange[aid]
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aid)
            print(
                f"{label} {name}: sweeping {math.degrees(lo):+.1f} .. "
                f"{math.degrees(hi):+.1f} deg"
            )

    def full_sweep(aids: list[int], s: float) -> dict[int, float]:
        # lo -> hi -> lo over one phase, starting and ending at the midpoint
        out = {}
        for aid in aids:
            lo, hi = model.actuator_ctrlrange[aid]
            mid, amp = (lo + hi) / 2, (hi - lo) / 2
            out[aid] = mid + amp * math.sin(2 * math.pi * s)
        return out

    def circle(s: float) -> dict[int, float]:
        out = {}
        for aid in j6:
            lo, hi = model.actuator_ctrlrange[aid]
            out[aid] = (lo + hi) / 2 + (hi - lo) / 2 * math.sin(2 * math.pi * s)
        for aid in j7:
            lo, hi = model.actuator_ctrlrange[aid]
            out[aid] = (lo + hi) / 2 + (hi - lo) / 2 * math.cos(2 * math.pi * s)
        return out

    phases = [
        ("J6 deviation sweep", lambda s: full_sweep(j6, s)),
        ("J7 flexion/extension sweep", lambda s: full_sweep(j7, s)),
        ("combined wrist circles", circle),
    ]

    state = {"phase": -1}

    def apply_ctrl() -> None:
        t = data.time - SETTLE_SECONDS
        if t < 0:
            data.ctrl[:] = home_ctrl
            return
        idx = int(t // PHASE_SECONDS) % len(phases)
        s = (t % PHASE_SECONDS) / PHASE_SECONDS
        if idx != state["phase"]:
            state["phase"] = idx
            print(f"[{t:7.1f}s] {phases[idx][0]}")
        data.ctrl[:] = home_ctrl
        targets = phases[idx][1](s)
        # ease in at the start of each phase and back out at the end, so
        # targets never jump at phase boundaries
        t_in = t % PHASE_SECONDS
        blend = smoothstep(t_in / BLEND_SECONDS) * smoothstep(
            (PHASE_SECONDS - t_in) / BLEND_SECONDS
        )
        for aid, v in targets.items():
            data.ctrl[aid] = home_ctrl[aid] + blend * (v - home_ctrl[aid])

    if args.headless:
        watch = {}
        for aid in j6 + j7:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aid)
            jid = model.actuator_trnid[aid, 0]
            watch[name] = (model.jnt_qposadr[jid], [math.inf, -math.inf])
        while data.time < SETTLE_SECONDS + PHASE_SECONDS * len(phases):
            apply_ctrl()
            mujoco.mj_step(model, data)
            for adr, minmax in watch.values():
                q = data.qpos[adr]
                minmax[0] = min(minmax[0], q)
                minmax[1] = max(minmax[1], q)
        print("\nachieved joint ranges over one cycle:")
        for name, (_, (lo, hi)) in watch.items():
            print(f"  {name:>18}: {math.degrees(lo):+6.1f} .. {math.degrees(hi):+6.1f} deg")
        return

    interactive_loop(model, data, step_fn=apply_ctrl)


if __name__ == "__main__":
    main()
