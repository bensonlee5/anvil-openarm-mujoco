"""Tests for deterministic generation of the tracked Anvil MJCF files."""

from pathlib import Path

from scripts import make_anvil_model


def test_generated_models_are_reproducible(tmp_path):
    import anvil_openarm_spec as spec_mod

    assert make_anvil_model.generate(tmp_path, verbose=False) == 0
    for spec in make_anvil_model.FILES.values():
        name = spec["out"]
        assert (tmp_path / name).read_text() == (
            make_anvil_model.OUT / name
        ).read_text()
    mesh = Path("assets") / spec_mod.WRIST_BRACKET_MESH_ASSET
    assert (tmp_path / mesh).read_bytes() == (make_anvil_model.OUT / mesh).read_bytes()


def test_wrist_bracket_meshes_mirror_via_scale():
    """Both sides reference the same CAD STL; the right side mirrors y via a
    negative mesh scale (the upstream v2 convention), so the two arms stay
    exact mirror images of one source mesh."""
    import anvil_openarm_spec as spec

    text = (make_anvil_model.OUT / "anvil_openarm_bimanual.xml").read_text()
    for side, mesh_name in spec.WRIST_BRACKET_MESH_NAMES.items():
        assert (
            f'<mesh name="{mesh_name}" file="{spec.WRIST_BRACKET_MESH_REF}" '
            f'scale="{spec.WRIST_BRACKET_MESH_SCALES[side]}" />'
        ) in text
    left = [float(v) for v in spec.WRIST_BRACKET_MESH_SCALES["l"].split()]
    right = [float(v) for v in spec.WRIST_BRACKET_MESH_SCALES["r"].split()]
    assert right == [left[0], -left[1], left[2]]


def test_wrist_bracket_stl_matches_cad_export():
    """models/assets carries a byte-exact copy of the CAD-exported STL."""
    import anvil_openarm_spec as spec

    src = make_anvil_model.ROOT / spec.WRIST_BRACKET_MESH_SOURCE
    copy = make_anvil_model.OUT / "assets" / spec.WRIST_BRACKET_MESH_ASSET
    assert src.is_file(), "re-export cad/anvil_wrist_bracket.stl from the CAD source"
    assert copy.read_bytes() == src.read_bytes()


def test_wrist_bracket_geoms_in_generated_model():
    """Bracket mesh, fasteners, and forearm standoffs are emitted per side."""
    import anvil_openarm_spec as spec

    text = (make_anvil_model.OUT / "anvil_openarm_bimanual.xml").read_text()
    assert f'<material name="{spec.WRIST_BRACKET_MATERIAL}"' in text
    assert f'<material name="{spec.WRIST_BRACKET_SCREW_MATERIAL}"' in text
    for side, mesh_name in spec.WRIST_BRACKET_MESH_NAMES.items():
        assert f'<mesh name="{mesh_name}"' in text
        assert f'<geom name="{mesh_name}" type="mesh" mesh="{mesh_name}"' in text
        for screw_name in spec.wrist_bracket_screw_geom_names(side).values():
            assert f'<geom name="{screw_name}" type="cylinder"' in text
        for standoff_name in spec.wrist_bracket_link5_geom_names(side).values():
            assert f'<geom name="{standoff_name}" type="cylinder"' in text
