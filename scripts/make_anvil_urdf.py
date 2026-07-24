#!/usr/bin/env python
"""Generate the Anvil OpenARM 2.0 URDF from the upstream OpenArm v2 file.

Reads the xacro-expanded bimanual example URDF from the
upstream/openarm_description submodule and writes an Anvil-variant copy into
models/, applying the same measured controller-coordinate wrist limits as
make_anvil_model.py:

  - left J6:  -45 deg .. +70 deg
  - right J6: -70 deg .. +45 deg

All other ranges, including the asymmetric J1 and J2 limits, remain upstream.

The patched numbers are identical to the MJCF ones because the upstream URDF
and MJCF share joint axes and sign conventions on every joint (verified by
tests/test_urdf_generation.py). The generated URDF also exposes
follower_{l,r}_hand_tcp as massless links on fixed joints at the upstream
pinch-gripper grasp frame — the URDF counterpart of the MJCF TCP sites — and
carries the Anvil wrist support bracket (the CAD STL mesh plus screw and
standoff cylinders) as visual-only blocks on the link5 forearm links,
mirroring the MJCF exactly.

All other content is preserved byte-for-byte, except that
package://openarm_description/ mesh references are rewritten to paths
relative to models/ so the file loads without a ROS package index. Every
replacement must match exactly once; if upstream changes shape, this script
fails loudly rather than silently producing a stale model.
"""

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from anvil_openarm_spec import (  # noqa: E402
    PATCHED_URDF_LIMITS,
    TCP_SITE_BODY_NAMES,
    TCP_SITE_NAMES,
    TCP_SITE_POS,
    TCP_SITE_RPY,
    WRIST_BRACKET_BODY_NAMES,
    WRIST_BRACKET_LINK5_CYLINDERS,
    WRIST_BRACKET_MATERIAL,
    WRIST_BRACKET_MESH_ASSET,
    WRIST_BRACKET_MESH_NAMES,
    WRIST_BRACKET_MESH_SCALES,
    WRIST_BRACKET_RGBA,
    WRIST_BRACKET_SCREW_CYLINDERS,
    WRIST_BRACKET_SCREW_MATERIAL,
    WRIST_BRACKET_SCREW_RGBA,
    wrist_bracket_link5_geom_names,
    wrist_bracket_screw_geom_names,
)

UPSTREAM = ROOT / "upstream" / "openarm_description"
SRC = UPSTREAM / "assets" / "robot" / "openarm_v2.0" / "urdf" / "example" / "v2.urdf"
OUT_DIR = ROOT / "models"
OUT_NAME = "anvil_openarm_bimanual.urdf"

# Mesh references: ROS package URI -> path relative to models/.
PKG_PREFIX = "package://openarm_description/"
MESH_PREFIX = "../upstream/openarm_description/"
# The CAD bracket mesh copied into models/assets/ by make_anvil_model.py,
# referenced relative to models/ (where the URDF lives).
BRACKET_MESH_PATH = f"assets/{WRIST_BRACKET_MESH_ASSET}"
# URDF cylinders run along local z; these rpy values rotate z onto the spec
# cylinder axes (y for the lug screws and standoff, x for the foot screws).
_RPY_Z_TO_Y = "-1.5707963267948966 0 0"
_RPY_Z_TO_X = "0 1.5707963267948966 0"


def _fmt(values) -> str:
    # repr() is the shortest exact round-trip form; +0.0 folds -0.0 away.
    return " ".join(repr(float(v) + 0.0) for v in values)


def literal(old: str, new: str) -> tuple[str, str]:
    return re.escape(old), new.replace("\\", "\\\\")


def limit_sub(joint: str) -> tuple[str, str]:
    """Rewrite the lower/upper attributes of `joint`'s <limit> element."""
    lower, upper = PATCHED_URDF_LIMITS[joint]
    pattern = (
        rf'(?s)(<joint name="{re.escape(joint)}" type="revolute">.*?'
        rf'<limit [^>]*\blower=")[^"]+(" upper=")[^"]+(")'
    )
    return pattern, rf"\g<1>{lower}\g<2>{upper}\g<3>"


def tcp_block(side: str) -> str:
    """A massless TCP link on a fixed joint — the URDF form of the MJCF site."""
    name = TCP_SITE_NAMES[side]
    parent = TCP_SITE_BODY_NAMES[side]
    # No indent on the first line: the anchor's own leading whitespace is
    # preserved in front of the replacement.
    return (
        f'<link name="{name}"/>\n'
        f'  <joint name="{name}_joint" type="fixed">\n'
        f'    <origin rpy="{_fmt(TCP_SITE_RPY)}" xyz="{_fmt(TCP_SITE_POS)}"/>\n'
        f'    <parent link="{parent}"/>\n'
        f'    <child link="{name}"/>\n'
        f'  </joint>\n'
    )


def _cylinder_visual(name: str, cylinder: tuple, sy: float, material: str) -> str:
    """A spec ((from), (to), radius) cylinder as a URDF <visual> block."""
    start, end, radius = cylinder
    start = (start[0], sy * start[1], start[2])
    end = (end[0], sy * end[1], end[2])
    mid = tuple((a + b) / 2 for a, b in zip(start, end))
    length = sum((b - a) ** 2 for a, b in zip(start, end)) ** 0.5
    axis = tuple((b - a) / length for a, b in zip(start, end))
    rpy = _RPY_Z_TO_X if abs(axis[0]) > 0.5 else _RPY_Z_TO_Y
    return (
        f'    <visual name="{name}">\n'
        f'      <origin rpy="{rpy}" xyz="{_fmt(mid)}"/>\n'
        f'      <geometry>\n'
        f'        <cylinder length="{repr(length)}" radius="{repr(float(radius))}"/>\n'
        f'      </geometry>\n'
        f'      <material name="{material}"/>\n'
        f'    </visual>'
    )


def bracket_visuals(side: str) -> str:
    """Visual-only bracket blocks on link5: the CAD STL mesh (right side
    mirrored in y via a negative mesh scale, like the MJCF), the dark screw
    heads, and the forearm-standoff cylinder — same geoms as the MJCF."""
    sy = -1.0 if side == "r" else 1.0
    scale = WRIST_BRACKET_MESH_SCALES[side]
    blocks = [
        f'    <visual name="{WRIST_BRACKET_MESH_NAMES[side]}">\n'
        f'      <origin rpy="0 0 0" xyz="0 0 0"/>\n'
        f'      <geometry>\n'
        f'        <mesh filename="{BRACKET_MESH_PATH}" scale="{scale}"/>\n'
        f'      </geometry>\n'
        f'      <material name="{WRIST_BRACKET_MATERIAL}"/>\n'
        f'    </visual>'
    ]
    screw_names = wrist_bracket_screw_geom_names(side)
    for key, cylinder in WRIST_BRACKET_SCREW_CYLINDERS.items():
        blocks.append(
            _cylinder_visual(screw_names[key], cylinder, sy, WRIST_BRACKET_SCREW_MATERIAL)
        )
    standoff_names = wrist_bracket_link5_geom_names(side)
    for key, cylinder in WRIST_BRACKET_LINK5_CYLINDERS.items():
        blocks.append(
            _cylinder_visual(standoff_names[key], cylinder, sy, WRIST_BRACKET_MATERIAL)
        )
    return "\n".join(blocks)


def bracket_insertion(side: str) -> tuple[str, str]:
    """Insert the bracket visuals after link5's collision block, keeping the
    link's visual/collision/inertial order intact."""
    link = WRIST_BRACKET_BODY_NAMES[side]
    pattern = rf'(?s)(<collision name="{re.escape(link)}_collision">.*?</collision>)'
    return pattern, rf"\g<1>\n{bracket_visuals(side)}"


def _material_block(name: str, rgba: tuple) -> str:
    return (
        f'  <material name="{name}">\n'
        f'    <color rgba="{" ".join(f"{v:.6g}" for v in rgba)}"/>\n'
        '  </material>'
    )


ROBOT_TAG_OLD = '<robot name="openarm_v20">'
ROBOT_TAG_NEW = (
    '<robot name="anvil_openarm_v20">\n'
    f"{_material_block(WRIST_BRACKET_MATERIAL, WRIST_BRACKET_RGBA)}\n"
    f"{_material_block(WRIST_BRACKET_SCREW_MATERIAL, WRIST_BRACKET_SCREW_RGBA)}"
)

# (pattern, replacement); each must match exactly once.
SUBS: list[tuple[str, str]] = [
    literal(ROBOT_TAG_OLD, ROBOT_TAG_NEW),
    limit_sub("openarm_left_joint6"),
    limit_sub("openarm_right_joint6"),
    # TCP link+joint go just before each side's finger_joint1; the
    # `type="revolute"` suffix keeps the anchor out of the ros2_control block.
    literal(
        '<joint name="openarm_left_finger_joint1" type="revolute">',
        f'{tcp_block("l")}  <joint name="openarm_left_finger_joint1" type="revolute">',
    ),
    literal(
        '<joint name="openarm_right_finger_joint1" type="revolute">',
        f'{tcp_block("r")}  <joint name="openarm_right_finger_joint1" type="revolute">',
    ),
    bracket_insertion("l"),
    bracket_insertion("r"),
]

HEADER = """\
<!--
GENERATED FILE - do not edit by hand. Regenerate with:
    uv run python scripts/make_anvil_urdf.py

Derived from upstream/openarm_description
assets/robot/openarm_v2.0/urdf/example/v2.urdf (enactic/openarm_description,
Apache-2.0) with Anvil OpenARM 2.0 controller-coordinate wrist limits:
left J6 -45..+70 deg, right J6 -70..+45 deg, follower hand TCP frames, plus
the Anvil wrist support bracket from user hardware CAD (visual-only mesh and
cylinder detail on the
link5 forearm links). Mesh references are rewritten from package:// URIs to
paths relative to this directory.
-->
"""


def generate(out_dir: Path = OUT_DIR, verbose: bool = True) -> int:
    if not SRC.is_file():
        print(f"upstream URDF not found: {SRC}")
        print("Run: git submodule update --init")
        return 1
    out_dir.mkdir(exist_ok=True)

    text = SRC.read_text()
    for pattern, repl in SUBS:
        text, n = re.subn(pattern, repl, text)
        if n != 1:
            print(
                f"ERROR: pattern matched {n} times (expected 1) in "
                f"{SRC.name}:\n  {pattern}\n"
                "Upstream layout changed; update this script."
            )
            return 1

    n_meshes = text.count(PKG_PREFIX)
    text = text.replace(PKG_PREFIX, MESH_PREFIX)
    if "package://" in text:
        print(
            "ERROR: package:// references from an unexpected package remain "
            "after rewriting; update this script."
        )
        return 1

    decl_match = re.match(r"<\?xml[^>]*\?>\s*\n", text)
    if decl_match:
        insert_at = decl_match.end()
        text = text[:insert_at] + HEADER + text[insert_at:]
    else:
        text = HEADER + text

    try:
        ET.fromstring(text)
    except ET.ParseError as exc:
        print(f"ERROR: generated URDF is not well-formed XML: {exc}")
        return 1

    out_path = out_dir / OUT_NAME
    out_path.write_text(text)
    if verbose:
        try:
            shown = out_path.relative_to(ROOT)
        except ValueError:
            shown = out_path
        print(f"wrote {shown} ({n_meshes} mesh references rewritten)")
    return 0


def main() -> int:
    return generate()


if __name__ == "__main__":
    sys.exit(main())
