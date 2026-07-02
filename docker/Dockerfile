# Multi-target Docker runtime for this repo.
#
# Build targets:
#   python    - headless model checks, pytest, and wrist sweep
#   web       - browser demo asset export, dev server, and production build
#   viewer    - official MuJoCo desktop viewer with X11/Xvfb dependencies
#   ros2-test - ROS 2 Jazzy bridge integration tests

FROM python:3.13-slim AS python

ENV PYTHONUNBUFFERED=1 \
    UV_PROJECT_ENVIRONMENT=/opt/anvil-venv \
    PATH="/root/.local/bin:/opt/anvil-venv/bin:${PATH}"

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        libegl1 \
        libgl1 \
        libglfw3 \
        libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -LsSf https://astral.sh/uv/install.sh | sh

WORKDIR /ws
COPY pyproject.toml uv.lock ./
RUN uv sync --frozen --all-groups

COPY . .
RUN test -f upstream/openarm_mujoco/v2/openarm_bimanual.xml \
    || (echo "upstream/openarm_mujoco is missing; run: git submodule update --init" >&2; exit 1)

CMD ["uv", "run", "pytest"]


FROM node:22-bookworm-slim AS web

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        python-is-python3 \
        python3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY web/package.json web/package-lock.json ./web/
RUN npm --prefix web ci

COPY . .
RUN test -f upstream/openarm_mujoco/v2/openarm_bimanual.xml \
    || (echo "upstream/openarm_mujoco is missing; run: git submodule update --init" >&2; exit 1)

CMD ["npm", "--prefix", "web", "run", "dev"]


FROM python AS viewer

ENV MUJOCO_GL=glfw \
    XDG_RUNTIME_DIR=/tmp/runtime-root

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        libice6 \
        libsm6 \
        libx11-6 \
        libxcursor1 \
        libxext6 \
        libxi6 \
        libxinerama1 \
        libxrandr2 \
        libxrender1 \
        libxxf86vm1 \
        xauth \
        xvfb \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p "$XDG_RUNTIME_DIR" \
    && chmod 700 "$XDG_RUNTIME_DIR"

CMD ["/opt/anvil-venv/bin/python", "scripts/view.py", "models/anvil_pedestal.xml"]


FROM ros:jazzy AS ros2-test

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH="/opt/venv/bin:${PATH}"

RUN pip install --no-cache-dir "mujoco>=3.6" numpy "pytest>=8,<9"

WORKDIR /ws
COPY . .
RUN test -f upstream/openarm_mujoco/v2/openarm_bimanual.xml \
    || (echo "upstream/openarm_mujoco is missing; run: git submodule update --init" >&2; exit 1)

# /ros_entrypoint.sh from the base image sources /opt/ros/jazzy/setup.bash.
CMD ["pytest", "tests/", "-v", "--color=yes"]
