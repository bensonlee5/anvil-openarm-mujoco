#!/usr/bin/env python
"""Open an Anvil OpenARM 2.0 model in an interactive MuJoCo viewer.

Usage:
    uv run python scripts/view.py [models/anvil_demo.xml] [--keyframe home]

On macOS this uses mujoco-python-viewer by default (the stock mjpython
viewer currently crashes on recent macOS); pass --official to try the stock
MuJoCo viewer instead.
"""

import argparse

from common import DEFAULT_MODEL, interactive_loop


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xml", nargs="?", default=str(DEFAULT_MODEL))
    parser.add_argument(
        "--keyframe",
        "-k",
        default="home",
        help="keyframe to load as the initial state (if the model defines it)",
    )
    parser.add_argument(
        "--official",
        action="store_true",
        help="use the stock MuJoCo viewer (mjpython) instead of mujoco-python-viewer",
    )
    args = parser.parse_args()

    import mujoco

    model = mujoco.MjModel.from_xml_path(args.xml)
    data = mujoco.MjData(model)
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, args.keyframe)
    if kid >= 0:
        mujoco.mj_resetDataKeyframe(model, data, kid)
        data.ctrl[:] = model.key_ctrl[kid]

    interactive_loop(model, data, official=args.official)


if __name__ == "__main__":
    main()
