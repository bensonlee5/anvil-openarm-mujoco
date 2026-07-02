"""ROS 2 bridge exposing the MuJoCo Anvil OpenARM 2.0 with the real robot's
joint-state, joint-command, and commanded-EE topic surface so clients can move
between simulation and hardware with minimal changes.

Interface (matching https://docs.anvil.bot/software/technical-reference):

  published
    /joint_states                sensor_msgs/JointState (pos, vel, effort)
    /ee_pose_left, /ee_pose_right
                                 geometry_msgs/PoseStamped (TCP pose;
                                 the real robot uses a custom CommandedEEPose
                                 type here — see README)
    /clock                       rosgraph_msgs/Clock (sim time)

  subscribed
    /follower_l_forward_position_controller/commands   std_msgs/Float64MultiArray
    /follower_r_forward_position_controller/commands   std_msgs/Float64MultiArray
        8 values ordered J1..J7 then finger_joint1 (7 accepted: gripper
        unchanged). Targets are clamped to the Anvil 2.0 joint limits.
    /commanded_ee_left, /commanded_ee_right
        geometry_msgs/PoseStamped by default, or Anvil's custom CommandedEEPose
        when commanded_ee_msg_type is set to that installed message type.

  parameters
    model_xml              path to MJCF (default models/anvil_openarm_bimanual.xml)
    publish_rate_hz        state publish + stepping rate (default 200.0)
    time_scale             sim seconds per wall second (default 1.0; tests use >1)
    commanded_ee_msg_type  message type for /commanded_ee_* subscriptions
                           (default geometry_msgs/msg/PoseStamped; set to auto
                           or the installed Anvil custom message type)

Run (after sourcing ROS 2):
    python3 -m bridge.ros2_bridge --ros-args -p time_scale:=1.0
"""

import math

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from anvil_openarm_spec import (
    COMMANDED_EE_TOPICS,
    EE_POSE_TOPICS,
    JOINT_COMMAND_TOPICS,
    WORLD_FRAME,
)
from bridge.sim_core import DEFAULT_MODEL, OpenArmSim

COMMAND_TOPICS = JOINT_COMMAND_TOPICS
DEFAULT_COMMANDED_EE_MSG_TYPE = "geometry_msgs/msg/PoseStamped"
AUTO_COMMANDED_EE_MSG_TYPE = "auto"


class OpenArmMujocoBridge(Node):
    def __init__(self, **node_kwargs):
        super().__init__("anvil_openarm_mujoco_bridge", **node_kwargs)
        model_xml = (
            self.declare_parameter("model_xml", str(DEFAULT_MODEL))
            .get_parameter_value()
            .string_value
        )
        rate = (
            self.declare_parameter("publish_rate_hz", 200.0)
            .get_parameter_value()
            .double_value
        )
        time_scale = (
            self.declare_parameter("time_scale", 1.0)
            .get_parameter_value()
            .double_value
        )
        commanded_ee_msg_type = (
            self.declare_parameter(
                "commanded_ee_msg_type", DEFAULT_COMMANDED_EE_MSG_TYPE
            )
            .get_parameter_value()
            .string_value
        )

        self.sim = OpenArmSim(model_xml)
        period = 1.0 / rate
        self._steps_per_tick = max(1, round(time_scale * period / self.sim.timestep))
        self._commanded_ee_subs = {}
        self._commanded_ee_type_timer = None

        self._pub_joints = self.create_publisher(JointState, "/joint_states", 10)
        self._pub_clock = self.create_publisher(Clock, "/clock", 10)
        self._pub_ee = {
            side: self.create_publisher(PoseStamped, topic, 10)
            for side, topic in EE_POSE_TOPICS.items()
        }
        self._subs = [
            self.create_subscription(
                Float64MultiArray,
                topic,
                lambda msg, side=side: self._on_command(side, msg),
                10,
            )
            for side, topic in COMMAND_TOPICS.items()
        ]
        if commanded_ee_msg_type.strip().lower() == AUTO_COMMANDED_EE_MSG_TYPE:
            self._commanded_ee_type_timer = self.create_timer(
                0.25, self._try_auto_commanded_ee_subscriptions
            )
            self.get_logger().info(
                "waiting to auto-detect /commanded_ee_* message type"
            )
        else:
            msg_type = self._load_commanded_ee_msg_type(commanded_ee_msg_type)
            self._create_commanded_ee_subscriptions(
                msg_type, commanded_ee_msg_type, tuple(COMMANDED_EE_TOPICS)
            )
        self._timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"loaded {model_xml}; stepping {self._steps_per_tick} x "
            f"{self.sim.timestep * 1000:.0f} ms every {period * 1000:.0f} ms"
        )

    # -- callbacks -----------------------------------------------------------

    def _on_command(self, side: str, msg: Float64MultiArray) -> None:
        try:
            self.sim.command_side(side, list(msg.data))
        except ValueError as exc:
            self.get_logger().warning(f"rejected command for '{side}': {exc}")

    def _on_commanded_ee(self, side: str, msg) -> None:
        try:
            pose = msg.pose
            header = getattr(msg, "header", None)
            frame_id = getattr(header, "frame_id", "")
            if frame_id and frame_id != WORLD_FRAME:
                self.get_logger().warning(
                    f"rejected commanded EE for '{side}': frame '{frame_id}' is "
                    f"not supported; publish targets in '{WORLD_FRAME}'"
                )
                return
            gripper = getattr(msg, "gripper", None)
            position = [
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ]
            quaternion = [
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ]
            self.sim.command_ee(side, position, quaternion, gripper)
        except (AttributeError, ValueError) as exc:
            self.get_logger().warning(
                f"rejected commanded EE for '{side}': {exc}"
            )

    def _tick(self) -> None:
        self.sim.step(n=self._steps_per_tick)
        stamp = self._sim_stamp()

        names, qpos, qvel, effort = self.sim.joint_states()
        js = JointState()
        js.header.stamp = stamp
        js.name = names
        js.position = [float(v) for v in qpos]
        js.velocity = [float(v) for v in qvel]
        js.effort = [float(v) for v in effort]
        self._pub_joints.publish(js)

        for side, pub in self._pub_ee.items():
            pos, quat = self.sim.ee_pose(side)  # TCP site, wxyz
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = "world"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (
                float(pos[0]),
                float(pos[1]),
                float(pos[2]),
            )
            ps.pose.orientation.w = float(quat[0])
            ps.pose.orientation.x = float(quat[1])
            ps.pose.orientation.y = float(quat[2])
            ps.pose.orientation.z = float(quat[3])
            pub.publish(ps)

        clk = Clock()
        clk.clock = stamp
        self._pub_clock.publish(clk)

    def _sim_stamp(self):
        t = self.sim.time
        sec = int(t)
        stamp = Time()
        stamp.sec = sec
        stamp.nanosec = int(round((t - sec) * 1e9)) % 1_000_000_000
        return stamp

    def _load_commanded_ee_msg_type(self, type_name: str):
        try:
            return get_message(type_name)
        except Exception as exc:
            raise RuntimeError(
                f"could not load commanded_ee_msg_type '{type_name}'. "
                "Install/source the package that provides the message, set "
                "commanded_ee_msg_type:=auto, or use "
                f"{DEFAULT_COMMANDED_EE_MSG_TYPE} for local testing."
            ) from exc

    def _create_commanded_ee_subscriptions(
        self, msg_type, type_name: str, sides: tuple[str, ...]
    ) -> None:
        for side in sides:
            if side in self._commanded_ee_subs:
                continue
            self._commanded_ee_subs[side] = self.create_subscription(
                msg_type,
                COMMANDED_EE_TOPICS[side],
                lambda msg, side=side: self._on_commanded_ee(side, msg),
                10,
            )
        self.get_logger().info(
            f"subscribed to /commanded_ee_* as {type_name}"
        )

    def _try_auto_commanded_ee_subscriptions(self) -> None:
        topic_types = dict(self.get_topic_names_and_types())
        for side, topic in COMMANDED_EE_TOPICS.items():
            if side in self._commanded_ee_subs:
                continue
            type_name = self._choose_commanded_ee_type(topic_types.get(topic, []))
            if not type_name:
                continue
            try:
                msg_type = get_message(type_name)
            except Exception as exc:
                self.get_logger().warning(
                    f"found {topic} type {type_name}, but could not load it: {exc}"
                )
                continue
            self._create_commanded_ee_subscriptions(msg_type, type_name, (side,))

        if len(self._commanded_ee_subs) == len(COMMANDED_EE_TOPICS):
            self.destroy_timer(self._commanded_ee_type_timer)
            self._commanded_ee_type_timer = None

    @staticmethod
    def _choose_commanded_ee_type(type_names: list[str]) -> str | None:
        if not type_names:
            return None
        for preferred in ("CommandedEEPose", "PoseStamped"):
            for type_name in type_names:
                if type_name.endswith(f"/{preferred}"):
                    return type_name
        return type_names[0]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenArmMujocoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
