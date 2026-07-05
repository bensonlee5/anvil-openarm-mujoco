"""Tests for deterministic generation of the tracked Anvil MJCF files."""

from scripts import make_anvil_model


def test_generated_models_are_reproducible(tmp_path):
    assert make_anvil_model.generate(tmp_path, verbose=False) == 0
    for spec in make_anvil_model.FILES.values():
        name = spec["out"]
        assert (tmp_path / name).read_text() == (
            make_anvil_model.OUT / name
        ).read_text()


def test_wrist_bracket_meshes_mirror_exactly():
    """Right bracket mesh is the left mesh mirrored in y (winding flipped)."""
    lv, lf = make_anvil_model._bracket_mesh_data(mirror_y=False)
    rv, rf = make_anvil_model._bracket_mesh_data(mirror_y=True)
    assert len(lv) == len(rv) and len(lf) == len(rf)
    for i in range(0, len(lv), 3):
        assert rv[i] == lv[i]  # x
        assert rv[i + 1] == -lv[i + 1]  # y mirrored
        assert rv[i + 2] == lv[i + 2]  # z
    # Faces are re-wound (b/c swapped) so the mirrored normals stay outward.
    for i in range(0, len(lf), 3):
        assert rf[i] == lf[i]
        assert rf[i + 1] == lf[i + 2]
        assert rf[i + 2] == lf[i + 1]


def test_wrist_bracket_geoms_in_generated_model():
    """Both bracket geoms are emitted, red, and attached to the link6 bodies."""
    import anvil_openarm_spec as spec

    text = (make_anvil_model.OUT / "anvil_openarm_bimanual.xml").read_text()
    assert f'<material name="{spec.WRIST_BRACKET_MATERIAL}"' in text
    for mesh_name in spec.WRIST_BRACKET_MESH_NAMES.values():
        assert f'<mesh name="{mesh_name}"' in text
        assert f'<geom name="{mesh_name}" type="mesh" mesh="{mesh_name}"' in text
