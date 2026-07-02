#!/usr/bin/env bash
# Build and run the ROS 2 integration test suite in Docker (ROS 2 Jazzy).
# Usage: scripts/run_ros2_tests.sh [extra pytest args...]
set -euo pipefail
cd "$(dirname "$0")/.."

docker build -f docker/Dockerfile.ros2-test -t anvil-openarm-ros2-test .
docker run --rm anvil-openarm-ros2-test pytest tests/ -v --color=yes "$@"
