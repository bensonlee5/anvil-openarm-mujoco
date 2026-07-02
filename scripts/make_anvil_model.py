#!/usr/bin/env python
"""Generate Anvil OpenARM 2.0 MJCF models from the upstream OpenArm v2 files.

Reads the standard OpenArm v2 models from the upstream/openarm_mujoco
submodule and writes Anvil-variant copies into models/, applying the joint
changes published in Anvil's comparison table
(https://docs.anvil.bot/introduction/openarm-2.0):

  - J1: -135 deg .. +135 deg   (upstream: -80 .. +200 in MJCF sign convention)
  - J6: -45 deg .. +70 deg     (upstream: -45 .. +45); the +25 deg is on the
                                radial-deviation (+) side per the table legend

All other content is preserved byte-for-byte, except that meshdir is pointed
back at the submodule's assets and inter-file references are renamed. Every
replacement must match exactly once; if upstream changes shape, this script
fails loudly rather than silently producing a stale model.
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
UPSTREAM = ROOT / "upstream" / "openarm_mujoco" / "v2"
OUT = ROOT / "models"

# Anvil OpenARM 2.0 values, radians. Number formatting mirrors the precision
# style upstream uses in each attribute (5 sig figs in range=, 6 in ctrlrange=).
J1_RANGE = "-2.3562 2.3562"          # +/-135 deg
J1_CTRLRANGE = "-2.35619 2.35619"
J6_RANGE = "-0.7854 1.2217"          # -45 deg (ulnar) .. +70 deg (radial)
J6_CTRLRANGE = "-0.785398 1.22173"

MESHDIR_OLD = 'meshdir="assets"'
MESHDIR_NEW = 'meshdir="../upstream/openarm_mujoco/v2/assets"'


def attr_sub(anchor: str, attr: str, new_value: str) -> tuple[str, str]:
    """Build a (pattern, replacement) that rewrites one attribute on the
    element whose name= attribute equals `anchor`."""
    pattern = rf'(name="{re.escape(anchor)}"[^>]*\b{attr}=")[^"]+(")'
    return pattern, rf"\g<1>{new_value}\g<2>"


def literal(old: str, new: str) -> tuple[str, str]:
    return re.escape(old), new.replace("\\", "\\\\")


# fname -> list of (pattern, replacement); each must match exactly once.
FILES: dict[str, dict] = {
    "openarm_bimanual.xml": {
        "out": "anvil_openarm_bimanual.xml",
        "subs": [
            literal('<mujoco model="openarm_v20">', '<mujoco model="anvil_openarm_v20">'),
            literal(MESHDIR_OLD, MESHDIR_NEW),
            # J1: symmetric +/-135 deg on both arms
            attr_sub("openarm_left_joint1", "range", J1_RANGE),
            attr_sub("openarm_right_joint1", "range", J1_RANGE),
            attr_sub("left_joint1_ctrl", "ctrlrange", J1_CTRLRANGE),
            attr_sub("right_joint1_ctrl", "ctrlrange", J1_CTRLRANGE),
            # J6: -45..+70 deg; identical numbers on both arms because the
            # upstream axes are mirrored (left "0 -1 0", right "0 1 0")
            attr_sub("openarm_left_joint6", "range", J6_RANGE),
            attr_sub("openarm_right_joint6", "range", J6_RANGE),
            attr_sub("left_joint6_ctrl", "ctrlrange", J6_CTRLRANGE),
            attr_sub("right_joint6_ctrl", "ctrlrange", J6_CTRLRANGE),
        ],
    },
    "cell.xml": {
        "out": "anvil_cell.xml",
        "subs": [
            literal(MESHDIR_OLD, MESHDIR_NEW),
            literal('file="openarm_bimanual.xml"', 'file="anvil_openarm_bimanual.xml"'),
        ],
    },
    "demo.xml": {
        "out": "anvil_demo.xml",
        "subs": [
            literal(
                '<mujoco model="openarm_bimanual cell demo">',
                '<mujoco model="anvil_openarm_bimanual cell demo">',
            ),
            literal('file="cell.xml"', 'file="anvil_cell.xml"'),
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
Apache-2.0) with Anvil OpenARM 2.0 joint changes: J1 +/-135 deg,
J6 -45..+70 deg. Spec: https://docs.anvil.bot/introduction/openarm-2.0
-->
"""


def main() -> int:
    if not UPSTREAM.is_dir():
        print(f"upstream checkout not found: {UPSTREAM}")
        print("Run: git submodule update --init")
        return 1
    OUT.mkdir(exist_ok=True)

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
        out_path = OUT / spec["out"]
        out_path.write_text(text)
        print(f"wrote {out_path.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
