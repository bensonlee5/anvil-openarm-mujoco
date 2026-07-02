#!/usr/bin/env bash
# Build and run this repo's split Docker targets.
set -euo pipefail

cd "$(dirname "$0")/.."

IMAGE_PREFIX="${ANVIL_DOCKER_IMAGE_PREFIX:-anvil-openarm}"
DOCKERFILE="${ANVIL_DOCKERFILE:-docker/Dockerfile}"
DOCKER_REBUILD="${ANVIL_DOCKER_REBUILD:-0}"
WEB_PORT="${ANVIL_WEB_PORT:-5173}"
WEB_NODE_MODULES_VOLUME="${ANVIL_WEB_NODE_MODULES_VOLUME:-anvil-openarm-web-node-modules}"
SMOKE_TIMEOUT="${ANVIL_VIEWER_SMOKE_TIMEOUT:-8s}"
TARGETS=(python web viewer ros2-test)

usage() {
  cat <<'EOF'
Usage: scripts/run_docker.sh <command> [args...]

Commands:
  build [target|all] Build missing Docker image(s) once for reuse
  rebuild [target|all] Force rebuild Docker image(s)
  generate          Regenerate models/anvil_*.xml in the Python container
  check              Run scripts/check_model.py in the Python container
  test [args...]     Run pytest in the Python container
  wrist-headless     Run the wrist sweep in headless mode
  web-build          Build the hosted browser demo
  web-dev            Serve the hosted browser demo on localhost:5173
  viewer-smoke       Smoke-test the official MuJoCo viewer under Xvfb
  viewer [xml]       Run the official MuJoCo viewer through X11 forwarding
  ros-bridge [args...] Run the ROS 2 bridge in a ROS Jazzy container
  ros-test [args...] Run the ROS 2 bridge integration test suite

Targets: python, web, viewer, ros2-test, all
Set ANVIL_DOCKER_REBUILD=1 to force a rebuild before any run command.
EOF
}

image_name() {
  local target="$1"
  printf '%s-%s\n' "$IMAGE_PREFIX" "$target"
}

image_exists() {
  docker image inspect "$1" >/dev/null 2>&1
}

build_target() {
  local target="$1"
  local image
  image="$(image_name "$target")"
  docker build -f "$DOCKERFILE" --target "$target" -t "$image" . >&2
  printf '%s\n' "$image"
}

ensure_target() {
  local target="$1"
  local image
  image="$(image_name "$target")"
  if [[ "$DOCKER_REBUILD" == "1" || "$DOCKER_REBUILD" == "true" ]] || ! image_exists "$image"; then
    build_target "$target"
  else
    printf 'using existing image %s\n' "$image" >&2
    printf '%s\n' "$image"
  fi
}

build_requested_targets() {
  local mode="$1"
  local target="${2:-all}"
  if [[ "$target" == "all" ]]; then
    for item in "${TARGETS[@]}"; do
      if [[ "$mode" == "force" ]]; then
        build_target "$item" >/dev/null
      else
        ensure_target "$item" >/dev/null
      fi
    done
    return
  fi
  for item in "${TARGETS[@]}"; do
    if [[ "$target" == "$item" ]]; then
      if [[ "$mode" == "force" ]]; then
        build_target "$target" >/dev/null
      else
        ensure_target "$target" >/dev/null
      fi
      return
    fi
  done
  echo "unknown Docker target: $target" >&2
  usage >&2
  exit 2
}

web_install_if_needed='
  set -euo pipefail
  stamp="web/node_modules/.package-lock.sha256"
  current="$(sha256sum web/package-lock.json | awk "{print \$1}")"
  installed=""
  if [ -f "$stamp" ]; then
    installed="$(cat "$stamp")"
  fi
  if [ "$installed" != "$current" ] || [ ! -x web/node_modules/.bin/vite ]; then
    npm --prefix web ci
    printf "%s\n" "$current" > "$stamp"
  fi
'

run_python() {
  local image
  image="$(ensure_target python)"
  docker run --rm -v "$PWD":/ws -w /ws "$image" "$@"
}

run_web_workspace() {
  local image
  image="$(ensure_target web)"
  docker run --rm \
    -v "$PWD":/ws \
    -v "${WEB_NODE_MODULES_VOLUME}":/ws/web/node_modules \
    -w /ws \
    "$image" \
    bash -lc "${web_install_if_needed}
      exec \"\$@\"
    " bash "$@"
}

command="${1:-}"
if [[ -z "$command" || "$command" == "-h" || "$command" == "--help" ]]; then
  usage
  exit 0
fi
shift

case "$command" in
  build)
    build_requested_targets ensure "${1:-all}"
    ;;
  rebuild)
    build_requested_targets force "${1:-all}"
    ;;
  generate)
    run_python uv run python scripts/make_anvil_model.py "$@"
    ;;
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
    run_web_workspace npm --prefix web run build -- "$@"
    ;;
  web-dev)
    image="$(ensure_target web)"
    docker run --rm -it \
      -p "${WEB_PORT}:${WEB_PORT}" \
      -e "ANVIL_WEB_PORT=${WEB_PORT}" \
      -v "$PWD":/ws \
      -v "${WEB_NODE_MODULES_VOLUME}":/ws/web/node_modules \
      -w /ws \
      "$image" \
      bash -lc "${web_install_if_needed}
        npm --prefix web run dev -- --host 0.0.0.0 --port \"\$ANVIL_WEB_PORT\" \"\$@\"
      " \
      bash "$@"
    ;;
  viewer-smoke)
    image="$(ensure_target viewer)"
    docker run --rm -v "$PWD":/ws -w /ws "$image" bash -lc '
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
    image="$(ensure_target viewer)"
    docker run "${docker_args[@]}" "$image" /opt/anvil-venv/bin/python scripts/view.py "$model" "$@"
    ;;
  ros-bridge)
    image="$(ensure_target ros2-test)"
    docker_args=(--rm -it -v "$PWD":/ws -w /ws)
    docker_args+=(-e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}")
    if [[ "$(uname -s)" == "Linux" ]]; then
      docker_args+=(--network host)
    fi
    docker run "${docker_args[@]}" "$image" python3 -m bridge.ros2_bridge "$@"
    ;;
  ros-test)
    image="$(ensure_target ros2-test)"
    docker run --rm -v "$PWD":/ws -w /ws "$image" pytest tests/ -v --color=yes "$@"
    ;;
  *)
    echo "unknown Docker command: $command" >&2
    usage >&2
    exit 2
    ;;
esac
