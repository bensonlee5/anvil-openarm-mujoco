"""Shared helpers for the viewer scripts."""

import os
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = ROOT / "models" / "anvil_demo.xml"


def interactive_loop(model, data, step_fn=None, official=False) -> None:
    """Run an interactive viewer until its window is closed.

    step_fn, if given, is called before every physics step (for control).

    On macOS the default is mujoco-python-viewer: the stock MuJoCo viewer
    (mjpython + _Simulate) currently crashes with "Caught an unknown
    exception" on recent macOS regardless of Python build. Pass
    official=True to try the stock viewer anyway.
    """
    import mujoco

    # render at ~60 fps regardless of the physics timestep
    steps_per_frame = max(1, round((1 / 60) / model.opt.timestep))
    frame_wall = steps_per_frame * model.opt.timestep

    if official or sys.platform != "darwin":
        ensure_mjpython()
        import mujoco.viewer as mjv

        if step_fn is None:
            mjv.launch(model, data)
            return
        with mjv.launch_passive(model, data) as viewer:
            while viewer.is_running():
                t0 = time.time()
                for _ in range(steps_per_frame):
                    step_fn()
                    mujoco.mj_step(model, data)
                viewer.sync()
                leftover = frame_wall - (time.time() - t0)
                if leftover > 0:
                    time.sleep(leftover)
        return

    import mujoco_viewer

    viewer = mujoco_viewer.MujocoViewer(model, data)
    try:
        while viewer.is_alive:
            t0 = time.time()
            for _ in range(steps_per_frame):
                if step_fn is not None:
                    step_fn()
                mujoco.mj_step(model, data)
            viewer.render()
            leftover = frame_wall - (time.time() - t0)
            if leftover > 0:
                time.sleep(leftover)
    finally:
        if viewer.is_alive:
            viewer.close()

_REEXEC_SENTINEL = "_ANVIL_OPENARM_MJPYTHON"


def _ensure_libpython() -> None:
    """mjpython's app bundle dlopens the venv python and resolves
    @executable_path/../lib/libpythonX.Y.dylib into .venv/lib/, which uv
    venvs leave empty — symlink the dylib from the base interpreter."""
    ver = f"{sys.version_info.major}.{sys.version_info.minor}"
    name = f"libpython{ver}.dylib"
    target = Path(sys.prefix) / "lib" / name
    if target.exists():
        return
    source = Path(sys.base_prefix) / "lib" / name
    if source.is_file():
        target.parent.mkdir(parents=True, exist_ok=True)
        target.symlink_to(source)


def ensure_mjpython() -> None:
    """Re-exec under mjpython on macOS, where the MuJoCo viewer requires it."""
    if sys.platform != "darwin" or os.environ.get(_REEXEC_SENTINEL):
        return
    _ensure_libpython()
    candidates = [
        Path(sys.executable).parent / "mjpython",
        ROOT / ".venv" / "bin" / "mjpython",
    ]
    mjpython = next((str(c) for c in candidates if c.is_file()), None) or shutil.which(
        "mjpython"
    )
    if mjpython is None:
        sys.exit(
            "The MuJoCo viewer needs mjpython on macOS and none was found.\n"
            "Run `uv sync`, then retry."
        )
    os.environ[_REEXEC_SENTINEL] = "1"
    os.execv(mjpython, [mjpython, *sys.argv])
