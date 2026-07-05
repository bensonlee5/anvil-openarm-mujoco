#!/usr/bin/env python
"""Generate Anvil OpenARM 2.0 MJCF models from the upstream OpenArm v2 files.

Reads the standard OpenArm v2 models from the upstream/openarm_mujoco
submodule and writes Anvil-variant copies into models/, applying this repo's
local Anvil pre-arrival joint-limit spec:

  - J1: -135 deg .. +135 deg   (upstream: -80 .. +200 in MJCF sign convention)
  - J6: -45 deg .. +70 deg     (upstream: -45 .. +45); the +25 deg is on the
                                radial-deviation (+) side

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
    WRIST_BRACKET_BOXES,
    WRIST_BRACKET_MATERIAL,
    WRIST_BRACKET_MESH_NAMES,
    WRIST_BRACKET_RGBA,
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


# ── Wrist bracket (inline meshes) ─────────────────────────────────────────────
# The red Anvil 2.0 C-bracket from the docs photo, as one visual-only inline
# mesh per side: disjoint cuboids from WRIST_BRACKET_BOXES, in the link6 frame
# (the gimbal body spanning J6 -> J7). The right side mirrors y with reversed
# face winding so the normals stay outward.

_BOX_FACES = [
    (0, 2, 1), (0, 3, 2),  # -z
    (4, 5, 6), (4, 6, 7),  # +z
    (0, 1, 5), (0, 5, 4),  # -y
    (2, 3, 7), (2, 7, 6),  # +y
    (0, 4, 7), (0, 7, 3),  # -x
    (1, 2, 6), (1, 6, 5),  # +x
]


def _bracket_mesh_data(mirror_y: bool) -> tuple[list[float], list[int]]:
    verts: list[float] = []
    faces: list[int] = []
    sy = -1.0 if mirror_y else 1.0
    for (cx, cy, cz), (hx, hy, hz) in WRIST_BRACKET_BOXES:
        base = len(verts) // 3
        for dz in (-hz, hz):
            for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
                verts += [cx + dx, sy * (cy + dy), cz + dz]
        for a, b, c in _BOX_FACES:
            if mirror_y:
                faces += [base + a, base + c, base + b]  # flip winding
            else:
                faces += [base + a, base + b, base + c]
    return verts, faces


def _fmt(values: list[float]) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def bracket_assets() -> str:
    rgba = _fmt(list(WRIST_BRACKET_RGBA))
    lines = [
        f'<material name="{WRIST_BRACKET_MATERIAL}" rgba="{rgba}" '
        'specular="0.4" shininess="0.3" />'
    ]
    for side, mirror in (("l", False), ("r", True)):
        verts, faces = _bracket_mesh_data(mirror)
        lines.append(
            f'<mesh name="{WRIST_BRACKET_MESH_NAMES[side]}" '
            f'vertex="{_fmt(verts)}" face="{" ".join(str(i) for i in faces)}" />'
        )
    return "\n    ".join(lines)


def bracket_geom(side: str) -> str:
    mesh = WRIST_BRACKET_MESH_NAMES[side]
    return (
        f'<geom name="{mesh}" type="mesh" mesh="{mesh}" class="visual" '
        f'material="{WRIST_BRACKET_MATERIAL}" />'
    )


# fname -> list of (pattern, replacement); each must match exactly once.
FILES: dict[str, dict] = {
    "openarm_bimanual.xml": {
        "out": "anvil_openarm_bimanual.xml",
        "subs": [
            literal('<mujoco model="openarm_v20">', '<mujoco model="anvil_openarm_v20">'),
            literal(MESHDIR_OLD, MESHDIR_NEW),
            # J1: symmetric +/-135 deg on both arms
            attr_sub(
                "openarm_left_joint1",
                "range",
                PATCHED_XML_RANGES[("openarm_left_joint1", "range")],
            ),
            attr_sub(
                "openarm_right_joint1",
                "range",
                PATCHED_XML_RANGES[("openarm_right_joint1", "range")],
            ),
            attr_sub(
                "left_joint1_ctrl",
                "ctrlrange",
                PATCHED_XML_RANGES[("left_joint1_ctrl", "ctrlrange")],
            ),
            attr_sub(
                "right_joint1_ctrl",
                "ctrlrange",
                PATCHED_XML_RANGES[("right_joint1_ctrl", "ctrlrange")],
            ),
            # J6: -45..+70 deg; identical numbers on both arms because the
            # upstream axes are mirrored (left "0 -1 0", right "0 1 0")
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
            # Anvil wrist bracket: red material + one inline mesh per side,
            # attached to the link6 gimbal bodies (visual-only).
            literal(
                "</asset>",
                f"  {bracket_assets()}\n  </asset>",
            ),
            literal(
                '<geom name="link6_left_collision_00" type="mesh" '
                'mesh="link6_left_collision_00" class="collision" />',
                '<geom name="link6_left_collision_00" type="mesh" '
                'mesh="link6_left_collision_00" class="collision" />\n'
                f"                  {bracket_geom('l')}",
            ),
            literal(
                '<geom name="link6_right_collision_00" type="mesh" '
                'mesh="link6_right_collision_00" class="collision" />',
                '<geom name="link6_right_collision_00" type="mesh" '
                'mesh="link6_right_collision_00" class="collision" />\n'
                f"                  {bracket_geom('r')}",
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
Apache-2.0) with Anvil OpenARM 2.0 local spec changes: J1 +/-135 deg,
J6 -45..+70 deg, follower hand TCP sites, plus the stylised red Anvil
wrist bracket (visual-only inline meshes on the link6 bodies).
-->
"""


def generate(out_dir: Path = OUT, verbose: bool = True) -> int:
    if not UPSTREAM.is_dir():
        print(f"upstream checkout not found: {UPSTREAM}")
        print("Run: git submodule update --init")
        return 1
    out_dir.mkdir(exist_ok=True)

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
