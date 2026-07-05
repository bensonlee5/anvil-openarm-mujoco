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
# The red C-bracket shown on the Anvil variant in the OpenARM 2.0 docs photo:
# it clamps onto the J6 rotor hub, runs out past the J7 motor, and lands on a
# plate bolted to the J7 end cap — the part that enables the extra +25 deg of
# radial deviation. Stock v2 meshes don't include it, so the generator emits a
# stylised visual-only approximation as inline MJCF meshes (one mirrored mesh
# per side) attached to the link6 gimbal body — the body that spans J6 → J7.
# Cuboids are (center, half-size) in the LEFT link6 frame (J6 axis = y with
# the rotor hub face at y ~ +0.045; J7 axis = x with the J7 motor housing —
# the ee_base cylinder — spanning z -0.023..-0.113; the hand continues in -z);
# the right side mirrors y. Sized against the model's own geom AABBs so the
# lug lands on the J6 hub face and the foot on the J7 housing barrel — a
# stylised take on the docs photo, not measured hardware.
WRIST_BRACKET_BOXES = [
    ((0.014, 0.0570, 0.000), (0.012, 0.0155, 0.013)),  # lug out from the J6 hub face
    ((0.014, 0.0670, -0.034), (0.012, 0.0055, 0.040)),  # arm dropping past the housing
    ((0.014, 0.0605, -0.068), (0.012, 0.0125, 0.012)),  # foot onto the J7 motor housing
]
WRIST_BRACKET_RGBA = (0.72, 0.06, 0.06, 1.0)
WRIST_BRACKET_MATERIAL = "anvil_red"
WRIST_BRACKET_MESH_NAMES = {"l": "anvil_wrist_bracket_left", "r": "anvil_wrist_bracket_right"}
WRIST_BRACKET_BODY_NAMES = {"l": "openarm_left_link6", "r": "openarm_right_link6"}
