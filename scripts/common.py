"""Shared helpers for the viewer scripts."""

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = ROOT / "models" / "anvil_demo.xml"


def interactive_loop(model, data, step_fn=None) -> None:
    """Run the official MuJoCo viewer until its window is closed."""
    import mujoco

    if sys.platform == "darwin":
        raise SystemExit(
            "The native MuJoCo desktop viewer is not supported on macOS in this repo.\n"
            "Use `scripts/run_docker.sh viewer` for the official viewer, or "
            "`npm --prefix web run dev` for the browser demos."
        )

    import mujoco.viewer as mjv

    # Render at about 60 fps regardless of the physics timestep.
    steps_per_frame = max(1, round((1 / 60) / model.opt.timestep))
    frame_wall = steps_per_frame * model.opt.timestep

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
