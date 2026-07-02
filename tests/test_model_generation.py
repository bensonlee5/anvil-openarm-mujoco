"""Tests for deterministic generation of the tracked Anvil MJCF files."""

from scripts import make_anvil_model


def test_generated_models_are_reproducible(tmp_path):
    assert make_anvil_model.generate(tmp_path, verbose=False) == 0
    for spec in make_anvil_model.FILES.values():
        name = spec["out"]
        assert (tmp_path / name).read_text() == (
            make_anvil_model.OUT / name
        ).read_text()
