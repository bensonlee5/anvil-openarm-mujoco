"""Integration tests for the ROS 2 bridge (bridge/ros2_bridge.py).

Require a ROS 2 environment (rclpy importable); skipped otherwise. Run them
via the Docker harness on machines without ROS 2:

    scripts/run_docker.sh ros-test
"""

import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from std_msgs.msg import Float64MultiArray  # noqa: E402

from bridge.ros2_bridge import (  # noqa: E402
    COMMANDED_EE_TOPICS,
    COMMAND_TOPICS,
    EE_POSE_TOPICS,
    OpenArmMujocoBridge,
)

DEG = math.pi / 180.0
WALL_TIMEOUT = 30.0


class Harness:
    """Bridge node + probe node spun together on one executor."""

    def __init__(self, time_scale: float = 10.0):
        rclpy.init()
        # speed the sim up relative to wall time so tests stay fast
        self.bridge = OpenArmMujocoBridge(
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "time_scale",
                    rclpy.parameter.Parameter.Type.DOUBLE,
                    time_scale,
                )
            ]
        )
        self.probe = rclpy.create_node("probe")
        self.joint_msgs: list[JointState] = []
        self.ee_msgs: list[PoseStamped] = []
        self.probe.create_subscription(
            JointState, "/joint_states", self.joint_msgs.append, 10
        )
        self.probe.create_subscription(
            PoseStamped, EE_POSE_TOPICS["l"], self.ee_msgs.append, 10
        )
        self.cmd_pubs = {
            side: self.probe.create_publisher(Float64MultiArray, topic, 10)
            for side, topic in COMMAND_TOPICS.items()
        }
        self.ee_cmd_pubs = {
            side: self.probe.create_publisher(PoseStamped, topic, 10)
            for side, topic in COMMANDED_EE_TOPICS.items()
        }
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.probe)

    def spin_until(self, cond, timeout: float = WALL_TIMEOUT) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if cond():
                return True
        return False

    def command(self, side: str, values) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        self.cmd_pubs[side].publish(msg)

    def command_ee(self, side: str, position, orientation_xyzw) -> None:
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = float(orientation_xyzw[0])
        msg.pose.orientation.y = float(orientation_xyzw[1])
        msg.pose.orientation.z = float(orientation_xyzw[2])
        msg.pose.orientation.w = float(orientation_xyzw[3])
        self.ee_cmd_pubs[side].publish(msg)

    def latest_joint(self, name: str) -> float:
        msg = self.joint_msgs[-1]
        return msg.position[list(msg.name).index(name)]

    def close(self) -> None:
        self.executor.remove_node(self.bridge)
        self.executor.remove_node(self.probe)
        self.bridge.destroy_node()
        self.probe.destroy_node()
        rclpy.try_shutdown()


@pytest.fixture()
def harness():
    h = Harness()
    yield h
    h.close()


def test_joint_states_published(harness):
    assert harness.spin_until(lambda: len(harness.joint_msgs) >= 5)
    msg = harness.joint_msgs[-1]
    assert "openarm_left_joint1" in msg.name
    assert "openarm_right_finger_joint1" in msg.name
    assert len(msg.name) == len(msg.position) == len(msg.velocity) == len(msg.effort)


def test_ee_pose_published(harness):
    assert harness.spin_until(lambda: len(harness.ee_msgs) >= 3)
    q = harness.ee_msgs[-1].pose.orientation
    assert math.isclose(
        q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z, 1.0, abs_tol=1e-6
    )
    assert harness.ee_msgs[-1].header.frame_id == "world"


def test_command_moves_arm(harness):
    assert harness.spin_until(lambda: len(harness.joint_msgs) >= 1)
    target = [0.0] * 8
    target[3] = 1.0  # left elbow to 1 rad
    harness.command("l", target)

    def converged():
        if not harness.joint_msgs:
            return False
        # keep re-publishing in case the first message raced the subscriber
        harness.command("l", target)
        return abs(harness.latest_joint("openarm_left_joint4") - 1.0) < 0.05

    assert harness.spin_until(converged), (
        f"left joint4 = {harness.latest_joint('openarm_left_joint4'):.3f}, wanted 1.0"
    )


def test_commanded_ee_moves_tcp(harness):
    assert harness.spin_until(lambda: len(harness.ee_msgs) >= 3)
    initial = harness.ee_msgs[-1].pose
    target = (
        initial.position.x + 0.02,
        initial.position.y,
        initial.position.z,
    )
    orientation = (
        initial.orientation.x,
        initial.orientation.y,
        initial.orientation.z,
        initial.orientation.w,
    )
    initial_error = _pose_distance(initial, target)

    def moved_closer():
        harness.command_ee("l", target, orientation)
        if not harness.ee_msgs:
            return False
        current = harness.ee_msgs[-1].pose
        return (
            _pose_distance(current, target) < initial_error * 0.65
            and _pose_distance(current, (
                initial.position.x,
                initial.position.y,
                initial.position.z,
            ))
            > 0.005
        )

    assert harness.spin_until(moved_closer)


def test_command_clamped_to_anvil_j6_limit(harness):
    assert harness.spin_until(lambda: len(harness.joint_msgs) >= 1)
    over = [0.0] * 8
    over[5] = 2.0  # way past the +70 deg Anvil limit

    def settled_at_limit():
        harness.command("r", over)
        if not harness.joint_msgs:
            return False
        q = harness.latest_joint("openarm_right_joint6")
        return abs(q - 70 * DEG) < 3 * DEG

    assert harness.spin_until(settled_at_limit)
    # and never past the mechanical limit
    q = harness.latest_joint("openarm_right_joint6")
    assert q <= 70 * DEG + 1e-2


def test_malformed_command_rejected_without_crash(harness):
    assert harness.spin_until(lambda: len(harness.joint_msgs) >= 1)
    harness.command("l", [0.0] * 3)  # wrong length: warned, ignored
    n = len(harness.joint_msgs)
    # bridge keeps publishing afterwards
    assert harness.spin_until(lambda: len(harness.joint_msgs) >= n + 5)


def _pose_distance(pose, xyz) -> float:
    return math.sqrt(
        (pose.position.x - xyz[0]) ** 2
        + (pose.position.y - xyz[1]) ** 2
        + (pose.position.z - xyz[2]) ** 2
    )
