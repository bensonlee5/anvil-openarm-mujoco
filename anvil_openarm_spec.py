"""Shared Anvil OpenARM 2.0 model constants.

The live Anvil OpenARM 2.0 page documents the wrist swap and wider J6
radial/ulnar deviation qualitatively. The numeric limits here are the local
pre-arrival spec this repo validates against until hardware measurements can
confirm or correct them.
"""

import math

DEG = math.pi / 180.0

MODEL_FILES = [
    "anvil_openarm_bimanual.xml",
    "anvil_pedestal.xml",
]

# Command slot order used by the real robot's
# /follower_{l,r}_forward_position_controller/commands topics.
ARM_COMMAND_ORDER = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint7",
    "finger_joint1",
]

JOINT_COMMAND_TOPICS = {
    "l": "/follower_l_forward_position_controller/commands",
    "r": "/follower_r_forward_position_controller/commands",
}
EE_POSE_TOPICS = {"l": "/ee_pose_left", "r": "/ee_pose_right"}
COMMANDED_EE_TOPICS = {"l": "/commanded_ee_left", "r": "/commanded_ee_right"}
WORLD_FRAME = "world"

SIDE_PREFIX = {"l": "openarm_left_", "r": "openarm_right_"}
SIDE_ACT_PREFIX = {"l": "left_", "r": "right_"}

# Symmetric-per-arm joint ranges, in radians. These have identical numeric
# ranges on both arms because the upstream MJCF mirrors joint axes where needed.
ARM_JOINT_RANGES = {
    "joint1": (-135 * DEG, 135 * DEG),
    "joint3": (-90 * DEG, 90 * DEG),
    "joint4": (0.0, 140 * DEG),
    "joint5": (-90 * DEG, 90 * DEG),
    "joint6": (-45 * DEG, 70 * DEG),
    "joint7": (-90 * DEG, 90 * DEG),
}

# J2 is asymmetric and sign-mirrored between arms in the upstream convention.
SIDE_JOINT_RANGES = {
    "openarm_left_joint2": (-190 * DEG, 10 * DEG),
    "openarm_right_joint2": (-10 * DEG, 190 * DEG),
}

PATCHED_CTRLRANGES = {
    "left_joint1_ctrl": ARM_JOINT_RANGES["joint1"],
    "right_joint1_ctrl": ARM_JOINT_RANGES["joint1"],
    "left_joint6_ctrl": ARM_JOINT_RANGES["joint6"],
    "right_joint6_ctrl": ARM_JOINT_RANGES["joint6"],
}

# Number formatting mirrors upstream XML precision: 5 sig figs in range= and
# 6 in ctrlrange= for the attributes patched by the generator.
PATCHED_XML_RANGES = {
    ("openarm_left_joint1", "range"): "-2.3562 2.3562",
    ("openarm_right_joint1", "range"): "-2.3562 2.3562",
    ("left_joint1_ctrl", "ctrlrange"): "-2.35619 2.35619",
    ("right_joint1_ctrl", "ctrlrange"): "-2.35619 2.35619",
    ("openarm_left_joint6", "range"): "-0.7854 1.2217",
    ("openarm_right_joint6", "range"): "-0.7854 1.2217",
    ("left_joint6_ctrl", "ctrlrange"): "-0.785398 1.22173",
    ("right_joint6_ctrl", "ctrlrange"): "-0.785398 1.22173",
}

# Upstream OpenArm v2 pinch-gripper grasp frame, exposed here under Anvil's
# documented follower_{l,r}_hand_tcp naming convention.
TCP_SITE_NAMES = {"l": "follower_l_hand_tcp", "r": "follower_r_hand_tcp"}
TCP_SITE_BODY_NAMES = {
    "l": "openarm_left_ee_base_link",
    "r": "openarm_right_ee_base_link",
}
TCP_SITE_POS = (-0.02193, 0.0, -0.138)
TCP_SITE_QUAT = (0.70710678, 0.0, 0.70710678, 0.0)

TCP_SITE_XML = (
    'pos="-0.02193 0 -0.138" '
    'quat="0.70710678 0 0.70710678 0" '
    'size="0.01" rgba="0 0.6 1 1"'
)

# Anvil CommandedEEPose gripper opening, in metres. The MJCF gripper itself is
# controlled by mirrored finger angles, so sim_core maps this scalar per side.
GRIPPER_METERS_RANGE = (-0.003, 0.05)

# ── Anvil 2.0 wrist bracket ──────────────────────────────────────────────────
# The Anvil wrist support bracket, from the user-authored hardware CAD
# (cad/anvil_openarm2_wrist_bracket_source.step; see also video IMG_0085.MOV):
# two lap-jointed arm plates with round pivot lugs 80 mm apart, an integral
# Ø10 spacer under the strap-side lug, and a strap ending in a two-bolt foot.
# It is the part that enables the extra +25 deg of radial deviation.
#
# The bracket is rigid to the FOREARM (link5), spanning the J5 and J6
# actuators: the foot bolts into the J6 motor case (link5 structure), the
# far lug pivots at a forearm standoff, and the strap-side lug is an
# outboard bearing seat ON the J6 axis — the gimbal shaft rotates within it,
# so the part stays visually connected at both ends for any joint angle.
#
# cad/anvil_wrist_bracket.py places that STEP into the LEFT link5 frame
# (strap-side lug bore on the J6 axis at link5 z -0.1205, spacer flush on
# the gimbal hub face, far lug at link5 z -0.0405 where the forearm standoff
# meets it, foot bolts at link5 z -0.1105/-0.1305 pointing inboard) and
# exports the STL the generator ships. Regenerate the STL with the CAD skill
# tooling when the source STEP changes. The right side mirrors y via a
# negative mesh scale (the upstream v2 mesh convention).
WRIST_BRACKET_MESH_SOURCE = "cad/anvil_wrist_bracket.stl"
WRIST_BRACKET_MESH_ASSET = "anvil_wrist_bracket.stl"
# file= reference in the generated XML, relative to the generated meshdir
# (the upstream submodule assets dir), climbing back to models/assets/.
WRIST_BRACKET_MESH_REF = f"../../../../models/assets/{WRIST_BRACKET_MESH_ASSET}"
# STL is in millimetres of the LEFT link6 frame; the right side mirrors y.
WRIST_BRACKET_MESH_SCALES = {"l": "0.001 0.001 0.001", "r": "0.001 -0.001 0.001"}
# Expected AABB of the LEFT bracket mesh in metres (from the placed CAD
# bounds: x -14..11, y 7.5..51.5, z -135.5..-35.5 mm), pinned so a stale or
# misplaced STL fails scripts/check_model.py loudly.
WRIST_BRACKET_MESH_AABB = ((-0.014, 0.0075, -0.1355), (0.011, 0.0515, -0.0355))
WRIST_BRACKET_RGBA = (0.77, 0.78, 0.80, 1.0)
WRIST_BRACKET_MATERIAL = "anvil_aluminum"
WRIST_BRACKET_SCREW_RGBA = (0.015, 0.014, 0.013, 1.0)
WRIST_BRACKET_SCREW_MATERIAL = "anvil_dark_fastener"
WRIST_BRACKET_MESH_NAMES = {"l": "anvil_wrist_bracket_left", "r": "anvil_wrist_bracket_right"}
WRIST_BRACKET_BODY_NAMES = {"l": "openarm_left_link5", "r": "openarm_right_link5"}
# The J6 axis runs along y through link5 (x, z) = (0, -0.1205): the link6
# body frame's position in link5. Used to pin the bearing-lug screw.
J6_AXIS_XZ_IN_LINK5 = (0.0, -0.1205)
# Dark fastener heads on the part's Ø3 holes, ((from), (to), radius) in the
# LEFT link5 frame; the right side mirrors y. The strap-side lug screw sits
# on the J6 axis (checked); the foot screws sit on the foot's outboard face,
# pointing inboard along x toward the J6 motor case.
WRIST_BRACKET_SCREW_CYLINDERS = {
    "lug_j6_screw": ((0.000, 0.0515, -0.1205), (0.000, 0.0555, -0.1205), 0.0030),
    "lug_forearm_screw": ((0.000, 0.0475, -0.0405), (0.000, 0.0515, -0.0405), 0.0030),
    "foot_upper_screw": ((0.011, 0.0125, -0.1105), (0.015, 0.0125, -0.1105), 0.0030),
    "foot_lower_screw": ((0.011, 0.0125, -0.1305), (0.015, 0.0125, -0.1305), 0.0030),
}
# Aluminum standoff on the forearm that the far lug pivots on, ((from),
# (to), radius) in the LEFT link5 frame; the right side mirrors y. The inner
# end starts just inside the forearm plate face (link5 y ~ 0.0323 at this z,
# ray-measured) so the standoff seats on it; the outer end meets the bracket
# far-lug inner face at y 0.0435.
WRIST_BRACKET_LINK5_CYLINDERS = {
    "forearm_standoff": ((0.000, 0.031, -0.0405), (0.000, 0.0435, -0.0405), 0.0050),
}


def wrist_bracket_screw_geom_names(side: str) -> dict:
    """Geom names for the dark screw-head cylinders on one side ('l' or 'r')."""
    return {
        key: f"{WRIST_BRACKET_MESH_NAMES[side]}_{key}"
        for key in WRIST_BRACKET_SCREW_CYLINDERS
    }


def wrist_bracket_link5_geom_names(side: str) -> dict:
    """Geom names for the forearm-side standoff cylinders ('l' or 'r')."""
    return {
        key: f"{WRIST_BRACKET_MESH_NAMES[side]}_{key}"
        for key in WRIST_BRACKET_LINK5_CYLINDERS
    }
