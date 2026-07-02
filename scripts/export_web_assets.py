#!/usr/bin/env python
"""Export generated Anvil MJCF and meshes for the hosted browser demo."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MODELS = ROOT / "models"
UPSTREAM_V2 = ROOT / "upstream" / "openarm_mujoco" / "v2"
OUT = ROOT / "web" / "public" / "sim-assets" / "anvil_openarm"


def rewrite_model_xml(src: Path, dst: Path, replacements: dict[str, str]) -> None:
    text = src.read_text()
    text = text.replace(
        'meshdir="../upstream/openarm_mujoco/v2/assets"',
        'meshdir="assets"',
    )
    for old, new in replacements.items():
        text = text.replace(old, new)
    dst.write_text(text)


def write_scene_xml(dst: Path) -> None:
    dst.write_text(
        """<mujoco model="anvil_openarm_bimanual scene">
  <statistic center="0 0 0.55" extent="1.2"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0.2 0.2 0.2"/>
    <rgba haze="0.75 0.82 0.86 1"/>
    <global azimuth="150" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.78 0.84 0.88" rgb2="1 1 1" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.62 0.67 0.64" rgb2="0.48 0.54 0.52" markrgb="0.9 0.9 0.86" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="6 6" reflectance="0.12"/>
    <model name="bimanual" file="anvil_openarm_bimanual.xml"/>
  </asset>

  <worldbody>
    <attach model="bimanual" prefix=""/>
    <light pos="0 0 3" dir="0 0 -1" directional="false"/>
    <geom name="floor" size="0 0 .125" type="plane" material="groundplane" conaffinity="15" condim="3"/>
  </worldbody>
</mujoco>
"""
    )


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
