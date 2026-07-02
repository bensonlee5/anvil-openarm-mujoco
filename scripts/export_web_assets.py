#!/usr/bin/env python
"""Export generated Anvil MJCF and meshes for the hosted browser demo."""

from __future__ import annotations

import json
import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MODELS = ROOT / "models"
UPSTREAM_V2 = ROOT / "upstream" / "openarm_mujoco" / "v2"
ANVIL_LOADER_CONFIGS = ROOT / "upstream" / "anvil_loader" / "config"
OUT = ROOT / "web" / "public" / "sim-assets" / "anvil_openarm"

OPENARM_V2_CONFIG_FILES = (
    "openarm_v2_inference.yaml",
    "openarm_v2_leader_follower_teleop.yaml",
    "openarm_v2_leader_only.yaml",
    "openarm_v2_quest_teleop.yaml",
    "openarm_v2_quest_teleop_commanded_ee.yaml",
)

CONFIG_TITLES = {
    "openarm_v2_inference": "OpenArm v2 Inference",
    "openarm_v2_leader_follower_teleop": "Leader-Follower Teleop",
    "openarm_v2_leader_only": "Leader Only",
    "openarm_v2_quest_teleop": "Quest Teleop",
    "openarm_v2_quest_teleop_commanded_ee": "Quest Commanded-EE Teleop",
}

REPO_SUPPORT_NOTES = {
    "openarm_v2_inference": (
        "External policy code can publish follower joint commands into the sim; "
        "policy runtime, observations, and checkpoints are not included."
    ),
    "openarm_v2_leader_follower_teleop": (
        "Metadata only in this sim; physical leader-arm CAN interfaces are not modeled."
    ),
    "openarm_v2_leader_only": (
        "Metadata only; this repo does not simulate leader-only hardware."
    ),
    "openarm_v2_quest_teleop": (
        "External Quest code can publish follower joint commands into the sim; "
        "the Quest app and VR transport are not included."
    ),
    "openarm_v2_quest_teleop_commanded_ee": (
        "Best match for this repo's commanded-EE bridge path; an external Quest "
        "publisher must provide /commanded_ee_* messages."
    ),
}


def rewrite_model_xml(src: Path, dst: Path, replacements: dict[str, str]) -> None:
    text = src.read_text()
    text = text.replace(
        'meshdir="../upstream/openarm_mujoco/v2/assets"',
        'meshdir="assets"',
    )
    for old, new in replacements.items():
        text = text.replace(old, new)
    dst.write_text(text)


# Height of the floating mount for the bimanual "showroom" scene. The arm
# hangs ~0.62 m below its mount frame at full downward reach, so 1.05 m keeps
# every full-range sweep clear of the floor (pinned by tests/test_web_scene.py).
SCENE_MOUNT_HEIGHT = 1.05


def write_scene_xml(dst: Path) -> None:
    dst.write_text(
        f"""<mujoco model="anvil_openarm_bimanual scene">
  <option timestep="0.001"/>

  <statistic center="0 0 0.85" extent="1.4"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0.2 0.2 0.2"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="150" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.78 0.84 0.88" rgb2="1 1 1" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.62 0.67 0.64" rgb2="0.48 0.54 0.52" markrgb="0.9 0.9 0.86" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="6 6" reflectance="0.12"/>
    <material name="mount_column" rgba="0.28 0.30 0.31 1" reflectance="0.05"/>
    <model name="bimanual" file="anvil_openarm_bimanual.xml"/>
  </asset>

  <worldbody>
    <!-- visual-only stand (contype/conaffinity 0) so full-range joint sweeps
         never collide with it -->
    <geom name="mount_base" type="cylinder" size="0.11 0.006" pos="0 0 0.006"
          material="mount_column" contype="0" conaffinity="0"/>
    <geom name="mount_column" type="cylinder" size="0.028 {SCENE_MOUNT_HEIGHT / 2}"
          pos="0 0 {SCENE_MOUNT_HEIGHT / 2}" material="mount_column"
          contype="0" conaffinity="0"/>
    <body name="mount" pos="0 0 {SCENE_MOUNT_HEIGHT}">
      <attach model="bimanual" prefix=""/>
    </body>
    <light pos="0 0 3" dir="0 0 -1" directional="false"/>
    <geom name="floor" size="0 0 .125" type="plane" material="groundplane" conaffinity="15" condim="3"/>
  </worldbody>
</mujoco>
"""
    )


def parse_scalar(value: str):
    value = value.strip()
    if value == "true":
        return True
    if value == "false":
        return False
    try:
        return float(value)
    except ValueError:
        return value.strip("\"'")


def infer_command_surface(
    config_id: str,
    control_mode: str,
    commanded_ee: bool,
    arms: list[dict[str, str]],
) -> str:
    arm_names = {arm["name"] for arm in arms}
    if commanded_ee:
        return "commanded_ee"
    if any(name.startswith("leader_") for name in arm_names) and any(
        name.startswith("follower_") for name in arm_names
    ):
        return "leader_follower"
    if arm_names and all(name.startswith("leader_") for name in arm_names):
        return "leader_only"
    if any("vrController" in arm for arm in arms):
        return "quest_joint_position"
    if "inference" in config_id:
        return "joint_position_policy"
    return control_mode


def extract_loader_profile(path: Path) -> dict:
    """Extract only the loader fields the browser selector needs.

    The anvil-loader files are the source of truth. We avoid copying complete
    YAML payloads into the static app because the browser only needs identity,
    command surface, and arm/interface hints.
    """
    text = path.read_text()
    config_id = path.stem
    summary = ""
    top_level: dict[str, object] = {}
    arms: list[dict[str, str]] = []
    current_arm: dict[str, str] | None = None
    in_arms = False

    for line in text.splitlines():
        if not summary and line.startswith("# OpenArm:"):
            summary = line.removeprefix("# OpenArm:").strip()
            continue

        top_match = re.match(r"^([a-zA-Z_][\w]*):\s*(.+)$", line)
        if top_match and top_match.group(1) in {
            "arm_type",
            "control_mode",
            "commanded_ee",
            "homing_velocity",
        }:
            top_level[top_match.group(1)] = parse_scalar(top_match.group(2))
            continue

        if line.strip() == "arms:":
            in_arms = True
            continue
        if in_arms and line and not line.startswith(" "):
            in_arms = False
            current_arm = None
        if not in_arms:
            continue

        arm_match = re.match(r"^  ([a-zA-Z_][\w]*):\s*$", line)
        if arm_match:
            current_arm = {"name": arm_match.group(1)}
            arms.append(current_arm)
            continue

        field_match = re.match(r"^    ([a-zA-Z_][\w]*):\s*(.+)$", line)
        if not field_match or current_arm is None:
            continue
        key, value = field_match.group(1), field_match.group(2)
        if key == "can_interface_name":
            current_arm["canInterfaceName"] = str(parse_scalar(value))
        elif key == "vr_controller":
            current_arm["vrController"] = str(parse_scalar(value))

    arm_type = str(top_level.get("arm_type", ""))
    control_mode = str(top_level.get("control_mode", ""))
    commanded_ee = bool(top_level.get("commanded_ee", False))
    return {
        "id": config_id,
        "title": CONFIG_TITLES.get(config_id, config_id.replace("_", " ").title()),
        "summary": summary,
        "filename": path.name,
        "sourcePath": f"upstream/anvil_loader/config/{path.name}",
        "sourceUrl": (
            "https://github.com/anvil-robotics/anvil-loader/blob/main/"
            f"config/{path.name}"
        ),
        "armType": arm_type,
        "controlMode": control_mode,
        "commandedEe": commanded_ee,
        "homingVelocity": top_level.get("homing_velocity"),
        "commandSurface": infer_command_surface(
            config_id,
            control_mode,
            commanded_ee,
            arms,
        ),
        "arms": arms,
        "repoSupport": REPO_SUPPORT_NOTES.get(config_id, ""),
    }


def write_loader_profiles(dst: Path) -> None:
    if not ANVIL_LOADER_CONFIGS.is_dir():
        raise SystemExit(
            "anvil-loader configs are missing; run "
            "git submodule update --init upstream/anvil_loader"
        )

    profiles = []
    for filename in OPENARM_V2_CONFIG_FILES:
        path = ANVIL_LOADER_CONFIGS / filename
        if not path.is_file():
            raise SystemExit(f"missing expected anvil-loader config: {path}")
        profiles.append(extract_loader_profile(path))

    payload = {
        "sourceRepo": "https://github.com/anvil-robotics/anvil-loader",
        "sourcePath": "upstream/anvil_loader/config",
        "profiles": profiles,
    }
    dst.write_text(json.dumps(payload, indent=2) + "\n")


def main() -> int:
    if not (MODELS / "anvil_openarm_bimanual.xml").is_file():
        raise SystemExit("generated models are missing; run scripts/make_anvil_model.py")
    if not (UPSTREAM_V2 / "assets").is_dir():
        raise SystemExit("upstream v2 assets are missing; run git submodule update --init")

    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)

    shutil.copytree(UPSTREAM_V2 / "assets", OUT / "assets")
    preview = ROOT / "upstream" / "openarm_mujoco" / "media" / "v2.png"
    if preview.is_file():
        shutil.copy2(preview, OUT / "preview.png")

    rewrite_model_xml(
        MODELS / "anvil_openarm_bimanual.xml",
        OUT / "anvil_openarm_bimanual.xml",
        {},
    )
    rewrite_model_xml(
        MODELS / "anvil_pedestal.xml",
        OUT / "pedestal.xml",
        {},
    )
    rewrite_model_xml(
        MODELS / "anvil_cell.xml",
        OUT / "cell.xml",
        {},
    )
    rewrite_model_xml(
        MODELS / "anvil_demo.xml",
        OUT / "demo.xml",
        {'file="anvil_cell.xml"': 'file="cell.xml"'},
    )
    write_scene_xml(OUT / "scene.xml")
    write_loader_profiles(OUT / "openarm_v2_configs.json")

    files = sorted(
        path.relative_to(OUT).as_posix()
        for path in OUT.rglob("*")
        if path.is_file() and path.name != "manifest.json"
    )
    (OUT / "manifest.json").write_text(json.dumps({"files": files}, indent=2) + "\n")
    print(f"exported {len(files)} web asset files to {OUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
