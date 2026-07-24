#!/usr/bin/env python
"""Generate Anvil OpenARM 2.0 MJCF models from the upstream OpenArm v2 files.

Reads the standard OpenArm v2 models from the upstream/openarm_mujoco
submodule and writes Anvil-variant copies into models/, applying the measured
Anvil controller-coordinate wrist limits:

  - left J6:  -45 deg .. +70 deg
  - right J6: -70 deg .. +45 deg

All other ranges, including the asymmetric J1 and J2 limits, remain upstream.

The generated bimanual model also exposes follower_{l,r}_hand_tcp sites using
the upstream OpenArm v2 pinch-gripper grasp frame.

All other content is preserved byte-for-byte, except that meshdir is pointed
back at the submodule's assets and inter-file references are renamed. Every
replacement must match exactly once; if upstream changes shape, this script
fails loudly rather than silently producing a stale model.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from anvil_openarm_spec import (  # noqa: E402
    PATCHED_XML_RANGES,
    TCP_SITE_NAMES,
    TCP_SITE_XML,
    WRIST_BRACKET_LINK5_CYLINDERS,
    WRIST_BRACKET_MATERIAL,
    WRIST_BRACKET_MESH_ASSET,
    WRIST_BRACKET_MESH_NAMES,
    WRIST_BRACKET_MESH_REF,
    WRIST_BRACKET_MESH_SCALES,
    WRIST_BRACKET_MESH_SOURCE,
    WRIST_BRACKET_RGBA,
    WRIST_BRACKET_SCREW_CYLINDERS,
    WRIST_BRACKET_SCREW_MATERIAL,
    WRIST_BRACKET_SCREW_RGBA,
    wrist_bracket_link5_geom_names,
    wrist_bracket_screw_geom_names,
)

UPSTREAM = ROOT / "upstream" / "openarm_mujoco" / "v2"
OUT = ROOT / "models"

MESHDIR_OLD = 'meshdir="assets"'
MESHDIR_NEW = 'meshdir="../upstream/openarm_mujoco/v2/assets"'


def attr_sub(anchor: str, attr: str, new_value: str) -> tuple[str, str]:
    """Build a (pattern, replacement) that rewrites one attribute on the
    element whose name= attribute equals `anchor`."""
    pattern = rf'(name="{re.escape(anchor)}"[^>]*\b{attr}=")[^"]+(")'
    return pattern, rf"\g<1>{new_value}\g<2>"


def literal(old: str, new: str) -> tuple[str, str]:
    return re.escape(old), new.replace("\\", "\\\\")


def tcp_site(side: str) -> str:
    return f'<site name="{TCP_SITE_NAMES[side]}" {TCP_SITE_XML} />'


# ── Wrist bracket (CAD STL mesh + fastener/standoff cylinders) ───────────────
# The bare-aluminum Anvil 2.0 wrist support bracket. The solid comes from the
# user-authored hardware CAD (cad/anvil_openarm2_wrist_bracket_source.step),
# placed into the LEFT link6 frame and exported to an STL by
# cad/anvil_wrist_bracket.py; the generator copies that STL into
# models/assets/ and references it as a mesh asset per side, mirroring y for
# the right arm via a negative mesh scale (the upstream v2 mesh convention).
# Dark screw heads (link6) and the forearm-side pivot standoff (link5) stay
# parametric cylinder geoms.

BRACKET_SRC_MESH = ROOT / WRIST_BRACKET_MESH_SOURCE


def copy_bracket_mesh(out_dir: Path, verbose: bool = True) -> bool:
    if not BRACKET_SRC_MESH.is_file():
        print(
            f"ERROR: bracket mesh not found: {BRACKET_SRC_MESH}\n"
            "Regenerate it from cad/anvil_wrist_bracket.py with the CAD "
            "skill tooling (STEP + --stl sidecar)."
        )
        return False
    out_path = out_dir / "assets" / WRIST_BRACKET_MESH_ASSET
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(BRACKET_SRC_MESH.read_bytes())
    if verbose:
        try:
            shown = out_path.relative_to(ROOT)
        except ValueError:
            shown = out_path
        print(f"wrote {shown} (copied from {WRIST_BRACKET_MESH_SOURCE})")
    return True


def _fmt(values: list[float]) -> str:
    return " ".join(f"{v + 0.0:.6g}" for v in values)  # + 0.0 normalises -0.0


def bracket_assets() -> str:
    rgba = _fmt(list(WRIST_BRACKET_RGBA))
    screw_rgba = _fmt(list(WRIST_BRACKET_SCREW_RGBA))
    lines = [
        f'<material name="{WRIST_BRACKET_MATERIAL}" rgba="{rgba}" '
        'specular="1.0" shininess="0.9" reflectance="0.8" />',
        f'<material name="{WRIST_BRACKET_SCREW_MATERIAL}" rgba="{screw_rgba}" '
        'specular="0.35" shininess="0.45" />',
    ]
    for side in ("l", "r"):
        lines.append(
            f'<mesh name="{WRIST_BRACKET_MESH_NAMES[side]}" '
            f'file="{WRIST_BRACKET_MESH_REF}" '
            f'scale="{WRIST_BRACKET_MESH_SCALES[side]}" />'
        )
    return "\n    ".join(lines)


def _cylinder_geom(name: str, start, end, radius: float, side: str, material: str) -> str:
    sy = 1.0 if side == "l" else -1.0
    fromto = _fmt([start[0], sy * start[1], start[2], end[0], sy * end[1], end[2]])
    return (
        f'<geom name="{name}" type="cylinder" fromto="{fromto}" '
        f'size="{radius:.6g}" class="visual" material="{material}" />'
    )


def bracket_geom(side: str) -> str:
    """All bracket geoms — mesh, screw heads, forearm standoff — for the
    link5 (forearm) body of one side."""
    mesh = WRIST_BRACKET_MESH_NAMES[side]
    lines = [
        f'<geom name="{mesh}" type="mesh" mesh="{mesh}" class="visual" '
        f'material="{WRIST_BRACKET_MATERIAL}" />'
    ]
    screw_names = wrist_bracket_screw_geom_names(side)
    for key, (start, end, radius) in WRIST_BRACKET_SCREW_CYLINDERS.items():
        lines.append(
            _cylinder_geom(
                screw_names[key], start, end, radius, side, WRIST_BRACKET_SCREW_MATERIAL
            )
        )
    standoff_names = wrist_bracket_link5_geom_names(side)
    for key, (start, end, radius) in WRIST_BRACKET_LINK5_CYLINDERS.items():
        lines.append(
            _cylinder_geom(
                standoff_names[key], start, end, radius, side, WRIST_BRACKET_MATERIAL
            )
        )
    return "\n                ".join(lines)


# fname -> list of (pattern, replacement); each must match exactly once.
FILES: dict[str, dict] = {
    "openarm_bimanual.xml": {
        "out": "anvil_openarm_bimanual.xml",
        "subs": [
            literal('<mujoco model="openarm_v20">', '<mujoco model="anvil_openarm_v20">'),
            literal(MESHDIR_OLD, MESHDIR_NEW),
            # J6: the wider radial/ulnar direction is sign-mirrored in the
            # controller coordinates used by the real session data.
            attr_sub(
                "openarm_left_joint6",
                "range",
                PATCHED_XML_RANGES[("openarm_left_joint6", "range")],
            ),
            attr_sub(
                "openarm_right_joint6",
                "range",
                PATCHED_XML_RANGES[("openarm_right_joint6", "range")],
            ),
            attr_sub(
                "left_joint6_ctrl",
                "ctrlrange",
                PATCHED_XML_RANGES[("left_joint6_ctrl", "ctrlrange")],
            ),
            attr_sub(
                "right_joint6_ctrl",
                "ctrlrange",
                PATCHED_XML_RANGES[("right_joint6_ctrl", "ctrlrange")],
            ),
            literal(
                '<site name="left_ee_control_point" pos="0 0 0.0" size="0.01" rgba="0 1 0 1" />',
                '<site name="left_ee_control_point" pos="0 0 0.0" size="0.01" rgba="0 1 0 1" />\n'
                f"                    {tcp_site('l')}",
            ),
            literal(
                '<site name="right_ee_control_point" pos="0.0 0.0 0.0" size="0.01" rgba="1 0 0 1" />',
                '<site name="right_ee_control_point" pos="0.0 0.0 0.0" size="0.01" rgba="1 0 0 1" />\n'
                f"                    {tcp_site('r')}",
            ),
            # Anvil wrist bracket: aluminum/fastener materials, plus all
            # bracket geoms (CAD STL mesh, dark screw heads, forearm pivot
            # standoff) on the link5 forearm bodies, visual-only. The bracket
            # is rigid to the forearm; its bearing lug sits on the J6 axis.
            literal(
                "</asset>",
                f"  {bracket_assets()}\n  </asset>",
            ),
            literal(
                '<geom name="link5_left_collision_02" type="mesh" '
                'mesh="link5_left_collision_02" class="collision" />',
                '<geom name="link5_left_collision_02" type="mesh" '
                'mesh="link5_left_collision_02" class="collision" />\n'
                f"                {bracket_geom('l')}",
            ),
            literal(
                '<geom name="link5_right_collision_02" type="mesh" '
                'mesh="link5_right_collision_02" class="collision" />',
                '<geom name="link5_right_collision_02" type="mesh" '
                'mesh="link5_right_collision_02" class="collision" />\n'
                f"                {bracket_geom('r')}",
            ),
        ],
    },
    "pedestal.xml": {
        "out": "anvil_pedestal.xml",
        "subs": [
            literal(
                '<mujoco model="openarm_v20_pedestal">',
                '<mujoco model="anvil_openarm_v20_pedestal">',
            ),
            literal(MESHDIR_OLD, MESHDIR_NEW),
            literal('file="openarm_bimanual.xml"', 'file="anvil_openarm_bimanual.xml"'),
        ],
    },
}

HEADER = """\
<!--
GENERATED FILE - do not edit by hand. Regenerate with:
    uv run python scripts/make_anvil_model.py

Derived from upstream/openarm_mujoco/v2/{src} (enactic/openarm_mujoco,
Apache-2.0) with Anvil OpenARM 2.0 controller-coordinate wrist limits:
left J6 -45..+70 deg, right J6 -70..+45 deg, follower hand TCP sites, plus
the Anvil wrist support bracket from user hardware CAD (visual-only STL
meshes on the link5 forearm bodies,
with fastener and forearm-standoff cylinder details).
-->
"""


def inject_keyframe_ctrl(path: Path) -> bool:
    """Add ctrl to every keyframe so position actuators hold the keyframe pose.

    Upstream keyframes carry qpos only; with ctrl defaulting to zero, any
    consumer that seeds actuator targets from key_ctrl (e.g. the web viewer)
    watches the arms sag out of the home pose. Derive each actuator's ctrl
    from its joint's keyframe qpos, clamped to ctrlrange.
    """
    import mujoco

    # Load without depending on the output directory's location: the
    # generated meshdir is relative to models/, which breaks when generating
    # elsewhere (e.g. the reproducibility test's tmp dir). Point meshdir at
    # the submodule absolutely and feed <include> files as in-memory assets.
    abs_meshdir = f'meshdir="{(UPSTREAM / "assets").resolve()}"'

    def absolutized(p: Path) -> str:
        return p.read_text().replace(MESHDIR_NEW, abs_meshdir)

    xml = absolutized(path)
    assets = {
        m.group(1): absolutized(path.parent / m.group(1)).encode()
        for m in re.finditer(r'<(?:include|model)\b[^>]*file="([^"]+\.xml)"', xml)
    }
    model = mujoco.MjModel.from_xml_string(xml, assets)
    if model.nkey == 0:
        return True
    text = path.read_text()
    for k in range(model.nkey):
        kname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, k) or ""
        ctrl = []
        for aid in range(model.nu):
            value = 0.0
            if model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT:
                jid = int(model.actuator_trnid[aid, 0])
                value = float(model.key_qpos[k][model.jnt_qposadr[jid]])
                lo, hi = model.actuator_ctrlrange[aid]
                if model.actuator_ctrllimited[aid]:
                    value = min(max(value, float(lo)), float(hi))
            ctrl.append(value)
        ctrl_attr = " ".join(f"{v:.6g}" for v in ctrl)
        pattern = rf'(<key name="{re.escape(kname)}"[^>]*?)\s*/>'
        text, n = re.subn(pattern, rf'\g<1>\n      ctrl="{ctrl_attr}" />', text, flags=re.DOTALL)
        if n != 1:
            print(
                f"ERROR: keyframe '{kname}' matched {n} times (expected 1) in {path.name}; "
                "update inject_keyframe_ctrl."
            )
            return False
    path.write_text(text)
    return True


def generate(out_dir: Path = OUT, verbose: bool = True) -> int:
    if not UPSTREAM.is_dir():
        print(f"upstream checkout not found: {UPSTREAM}")
        print("Run: git submodule update --init")
        return 1
    out_dir.mkdir(exist_ok=True)

    if not copy_bracket_mesh(out_dir, verbose=verbose):
        return 1

    for src_name, spec in FILES.items():
        src = UPSTREAM / src_name
        text = src.read_text()
        for pattern, repl in spec["subs"]:
            text, n = re.subn(pattern, repl, text)
            if n != 1:
                print(
                    f"ERROR: pattern matched {n} times (expected 1) in "
                    f"{src_name}:\n  {pattern}\n"
                    "Upstream layout changed; update this script."
                )
                return 1
        header = HEADER.format(src=src_name)
        decl_match = re.match(r"<\?xml[^>]*\?>\s*\n", text)
        if decl_match:
            insert_at = decl_match.end()
            text = text[:insert_at] + header + text[insert_at:]
        else:
            text = header + text
        out_path = out_dir / spec["out"]
        out_path.write_text(text)
        if not inject_keyframe_ctrl(out_path):
            return 1
        if verbose:
            try:
                shown = out_path.relative_to(ROOT)
            except ValueError:
                shown = out_path
            print(f"wrote {shown}")
    return 0


def main() -> int:
    return generate()


if __name__ == "__main__":
    sys.exit(main())
