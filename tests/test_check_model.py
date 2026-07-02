"""Regression tests for the standalone model validator."""

import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_check_model_is_warning_free():
    result = subprocess.run(
        [sys.executable, "scripts/check_model.py"],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    combined = result.stdout + result.stderr
    assert result.returncode == 0, combined
    assert result.stderr == ""
    assert "WARNING:" not in result.stdout
    assert "compile emitted warning" not in result.stdout
