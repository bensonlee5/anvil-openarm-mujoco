"""Unit tests for the ROS-free MuJoCo sim core (bridge/sim_core.py).

These run without ROS 2 installed:  uv run pytest tests/test_sim_core.py
"""

from dataclasses import replace
import math

import numpy as np
import pytest

import mujoco
from anvil_openarm_spec import (
    GRIPPER_METERS_RANGE,
    TCP_SITE_BODY_NAMES,
    TCP_SITE_NAMES,
)
from bridge.sim_core import (
    ARM_COMMAND_ORDER,
    DEFAULT_IK_TUNING,
    OpenArmSim,
    _apply_joint_velocity_limits,
    _clamp_vector_norm,
    _nullspace_bias,
    _selectively_damped_least_squares,
)

DEG = math.pi / 180.0


@pytest.fixture()
def sim():
    return OpenArmSim()


def test_command_order_matches_anvil_docs():
    # Anvil: commands start at joint1 and end with the gripper (finger_joint1)
    assert ARM_COMMAND_ORDER == [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
        "finger_joint1",
    ]


def test_actuator_mapping(sim):
    # 8 command slots per arm, both arms mapped
    for side in ("l", "r"):
        assert len(sim.command_actuators(side)) == len(ARM_COMMAND_ORDER)


def test_joint_states_shape(sim):
    names, qpos, qvel, effort = sim.joint_states()
    assert len(names) == len(qpos) == len(qvel) == len(effort)
    assert "openarm_left_joint6" in names
    assert "openarm_right_finger_joint1" in names
    assert np.all(np.isfinite(qpos)) and np.all(np.isfinite(qvel))


def test_command_clamps_to_anvil_ranges(sim):
    # J6 commanded far beyond +70 deg must clamp to the Anvil limit
    cmd = [0.0] * 8
    cmd[5] = 2.0  # rad, > 1.2217
    applied = sim.command_side("l", cmd)
    assert applied[5] == pytest.approx(70 * DEG, abs=1e-3)

    # J1 commanded beyond -135 deg must clamp
    cmd = [0.0] * 8
    cmd[0] = -3.0
    applied = sim.command_side("r", cmd)
    assert applied[0] == pytest.approx(-135 * DEG, abs=1e-3)


def test_command_length_validation(sim):
    with pytest.raises(ValueError):
        sim.command_side("l", [0.0] * 5)
    # 7 values (no gripper) is accepted per-docs tolerance
    sim.command_side("l", [0.0] * 7)


def test_position_command_converges(sim):
    cmd = [0.0] * 8
    cmd[3] = 1.0  # joint4 (elbow) to 1 rad
    sim.command_side("l", cmd)
    sim.step(seconds=3.0)
    names, qpos, _, _ = sim.joint_states()
    j4 = qpos[names.index("openarm_left_joint4")]
    assert j4 == pytest.approx(1.0, abs=0.05)


def test_extended_j6_reachable_and_limited(sim):
    # command J6 to the Anvil max on both arms and hold
    for side in ("l", "r"):
        cmd = [0.0] * 8
        cmd[5] = 70 * DEG
        sim.command_side(side, cmd)
    sim.step(seconds=3.0)
    names, qpos, _, _ = sim.joint_states()
    for jname in ("openarm_left_joint6", "openarm_right_joint6"):
        q = qpos[names.index(jname)]
        assert q == pytest.approx(70 * DEG, abs=3 * DEG)
        # never past the mechanical limit
        assert q <= 70 * DEG + 1e-3


def test_time_advances(sim):
    t0 = sim.time
    sim.step(n=100)
    assert sim.time == pytest.approx(t0 + 100 * sim.timestep)


def test_ee_pose(sim):
    pos0, quat0 = sim.ee_pose("l")
    assert np.linalg.norm(quat0) == pytest.approx(1.0, abs=1e-6)
    cmd = [0.0] * 8
    cmd[3] = 1.2
    sim.command_side("l", cmd)
    sim.step(seconds=2.0)
    pos1, _ = sim.ee_pose("l")
    assert np.linalg.norm(np.array(pos1) - np.array(pos0)) > 0.05


def test_ee_pose_uses_tcp_site_not_gripper_base(sim):
    mujoco.mj_forward(sim.model, sim.data)
    for side in ("l", "r"):
        pos, quat = sim.ee_pose(side)
        sid = mujoco.mj_name2id(
            sim.model, mujoco.mjtObj.mjOBJ_SITE, TCP_SITE_NAMES[side]
        )
        bid = mujoco.mj_name2id(
            sim.model, mujoco.mjtObj.mjOBJ_BODY, TCP_SITE_BODY_NAMES[side]
        )
        site_quat = np.empty(4)
        mujoco.mju_mat2Quat(site_quat, sim.data.site_xmat[sid])

        assert pos == pytest.approx(sim.data.site_xpos[sid], abs=1e-9)
        assert quat == pytest.approx(site_quat, abs=1e-9)
        assert np.linalg.norm(pos - sim.data.xpos[bid]) > 0.1


def test_command_ee_moves_tcp_toward_target(sim):
    pos0, quat0 = sim.ee_pose("l")
    target = pos0 + np.array([0.02, 0.0, 0.0])
    initial_error = np.linalg.norm(target - pos0)

    applied = sim.command_ee("l", target, quat0)
    assert applied.shape == (8,)

    sim.step(seconds=2.0)
    pos1, _ = sim.ee_pose("l")
    assert np.linalg.norm(target - pos1) < initial_error * 0.4


def test_command_ee_caps_far_target_by_joint_velocity_limits(sim):
    pos0, quat0 = sim.ee_pose("l")
    current_q = np.array(
        [
            sim.data.qpos[sim.model.jnt_qposadr[jid]]
            for jid in sim._arm_joint_ids["l"]
        ]
    )
    target = pos0 + np.array([1.5, 0.8, 0.4])

    applied = sim.command_ee("l", target, quat0)
    cap = (
        np.array(DEFAULT_IK_TUNING.joint_velocity_limits)
        * DEFAULT_IK_TUNING.control_dt
    )

    assert np.all(np.isfinite(applied))
    assert np.all(np.abs(applied[:7] - current_q) <= cap + 1e-9)
    assert np.linalg.norm(applied[:7] - current_q) > 0.01


def test_command_ee_gripper_meters_follow_side_convention(sim):
    for side in ("l", "r"):
        pos, quat = sim.ee_pose(side)
        aid = sim.command_actuators(side)[7]
        lo, hi = sim.model.actuator_ctrlrange[aid]
        closed = lo if abs(lo) < abs(hi) else hi
        open_ = hi if abs(hi) > abs(lo) else lo

        applied_closed = sim.command_ee(side, pos, quat, GRIPPER_METERS_RANGE[0])
        applied_open = sim.command_ee(side, pos, quat, GRIPPER_METERS_RANGE[1])

        assert applied_closed[7] == pytest.approx(closed)
        assert applied_open[7] == pytest.approx(open_)


def test_sdls_keeps_near_singular_updates_bounded():
    tuning = replace(
        DEFAULT_IK_TUNING,
        min_singular_value=1e-9,
        singular_direction_step=0.03,
    )
    jac = np.array([[1.0, 0.0], [0.0, 1e-4]])
    err = np.array([0.2, 0.2])

    dq, jac_inv = _selectively_damped_least_squares(jac, err, tuning)

    assert np.all(np.isfinite(dq))
    assert np.all(np.isfinite(jac_inv))
    assert abs(dq[1]) < tuning.singular_direction_step
    assert abs(dq[1]) < 0.01  # a pseudo-inverse would try ~2000 rad here


def test_joint_velocity_limit_helper_is_joint_specific():
    current = np.zeros(3)
    desired = np.array([10.0, -10.0, 0.2])
    limited = _apply_joint_velocity_limits(
        current, desired, velocity_limits=[1.0, 2.0, 3.0], control_dt=0.25
    )

    assert limited == pytest.approx([0.25, -0.5, 0.2])


def test_nullspace_bias_prefers_nominal_and_joint_limit_centering():
    tuning = replace(DEFAULT_IK_TUNING, nullspace_gain=0.1, joint_limit_gain=0.1)
    q = np.zeros(7)
    nominal = np.zeros(7)
    nominal[3] = 1.0
    lo = np.array([-1.0, -1.0, -1.0, 0.0, -1.0, -1.0, -1.0])
    hi = np.array([1.0, 1.0, 1.0, 2.0, 1.0, 1.0, 1.0])

    bias = _nullspace_bias(q, nominal, lo, hi, tuning)

    assert bias[3] > 0.0  # elbow joint is biased away from its lower limit

    q_near_upper = q.copy()
    q_near_upper[0] = 0.95
    bias = _nullspace_bias(q_near_upper, nominal, lo, hi, tuning)
    assert bias[0] < 0.0


def test_vector_clamping_preserves_direction():
    vec = np.array([3.0, 4.0, 0.0])
    clamped = _clamp_vector_norm(vec, 0.5)

    assert np.linalg.norm(clamped) == pytest.approx(0.5)
    assert clamped / np.linalg.norm(clamped) == pytest.approx(vec / 5.0)
