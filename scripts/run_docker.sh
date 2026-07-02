#!/usr/bin/env bash
# Build and run this repo's split Docker targets.
set -euo pipefail

cd "$(dirname "$0")/.."

IMAGE_PREFIX="${ANVIL_DOCKER_IMAGE_PREFIX:-anvil-openarm}"
DOCKERFILE="${ANVIL_DOCKERFILE:-docker/Dockerfile}"
WEB_PORT="${ANVIL_WEB_PORT:-5173}"
WEB_NODE_MODULES_VOLUME="${ANVIL_WEB_NODE_MODULES_VOLUME:-anvil-openarm-web-node-modules}"
SMOKE_TIMEOUT="${ANVIL_VIEWER_SMOKE_TIMEOUT:-8s}"

usage() {
  cat <<'EOF'
Usage: scripts/run_docker.sh <command> [args...]

Commands:
  check              Run scripts/check_model.py in the Python container
  test [args...]     Run pytest in the Python container
  wrist-headless     Run the wrist sweep in headless mode
  web-build          Build the hosted browser demo
  web-dev            Serve the hosted browser demo on localhost:5173
  viewer-smoke       Smoke-test the official MuJoCo viewer under Xvfb
  viewer [xml]       Run the official MuJoCo viewer through X11 forwarding
  ros-test [args...] Run the ROS 2 bridge integration test suite
EOF
}

build_target() {
  local target="$1"
  local image="${IMAGE_PREFIX}-${target}"
  docker build -f "$DOCKERFILE" --target "$target" -t "$image" . >&2
  printf '%s\n' "$image"
}

run_python() {
  local image
  image="$(build_target python)"
  docker run --rm -v "$PWD":/ws -w /ws "$image" "$@"
}

run_web_image() {
  local image
  image="$(build_target web)"
  docker run --rm "$image" "$@"
}

command="${1:-}"
if [[ -z "$command" || "$command" == "-h" || "$command" == "--help" ]]; then
  usage
  exit 0
fi
shift

case "$command" in
  check)
    run_python uv run python scripts/check_model.py "$@"
    ;;
  test)
    run_python uv run pytest "$@"
    ;;
  wrist-headless)
    run_python uv run python scripts/demo_wrist_sweep.py --headless "$@"
    ;;
  web-build)
    run_web_image npm --prefix web run build -- "$@"
    ;;
  web-dev)
    image="$(build_target web)"
    docker run --rm -it \
      -p "${WEB_PORT}:${WEB_PORT}" \
      -e "ANVIL_WEB_PORT=${WEB_PORT}" \
      -v "$PWD":/ws \
      -v "${WEB_NODE_MODULES_VOLUME}":/ws/web/node_modules \
      -w /ws \
      "$image" \
      bash -lc 'npm --prefix web ci && npm --prefix web run dev -- --host 0.0.0.0 --port "$ANVIL_WEB_PORT" "$@"' \
      bash "$@"
    ;;
  viewer-smoke)
    image="$(build_target viewer)"
    docker run --rm "$image" bash -lc '
      set +e
      xvfb-run -a timeout "$1" /opt/anvil-venv/bin/python scripts/view.py models/anvil_pedestal.xml
      rc=$?
      if [ "$rc" -eq 124 ]; then
        exit 0
      fi
      exit "$rc"
    ' bash "$SMOKE_TIMEOUT"
    ;;
  viewer)
    model="${1:-models/anvil_pedestal.xml}"
    if [[ $# -gt 0 ]]; then
      shift
    fi
    docker_args=(--rm -it -v "$PWD":/ws -w /ws)
    if [[ "$(uname -s)" == "Darwin" ]]; then
      display_value="${DISPLAY:-host.docker.internal:0}"
      docker_args+=(
        -e "DISPLAY=${display_value}"
        -e "LIBGL_ALWAYS_INDIRECT=${LIBGL_ALWAYS_INDIRECT:-0}"
        -e QT_X11_NO_MITSHM=1
      )
    else
      if [[ -z "${DISPLAY:-}" ]]; then
        echo "DISPLAY is not set. Start an X11 session, then retry." >&2
        exit 2
      fi
      docker_args+=(
        -e "DISPLAY=${DISPLAY}"
        -e QT_X11_NO_MITSHM=1
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
      )
      if [[ -e /dev/dri ]]; then
        docker_args+=(--device /dev/dri)
      fi
    fi
    image="$(build_target viewer)"
    docker run "${docker_args[@]}" "$image" /opt/anvil-venv/bin/python scripts/view.py "$model" "$@"
    ;;
  ros-test)
    image="$(build_target ros2-test)"
    docker run --rm "$image" pytest tests/ -v --color=yes "$@"
    ;;
  *)
    echo "unknown Docker command: $command" >&2
    usage >&2
    exit 2
    ;;
esac
