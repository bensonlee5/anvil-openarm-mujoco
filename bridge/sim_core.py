"""ROS-free MuJoCo simulation core for the Anvil OpenARM 2.0.

Wraps the generated bimanual model with the command interface the real
Anvil robot exposes: per-arm position commands ordered J1..J7 then the
gripper (finger_joint1), as documented in
https://docs.anvil.bot/software/technical-reference/commanding-robot-movement.
Commands are clamped to the actuator ctrlranges, which carry the Anvil
OpenARM 2.0 joint limits. End-effector poses are reported at the generated
follower_{l,r}_hand_tcp sites. Cartesian commanded-EE inputs are converted to
joint targets with a local Anvil-style IK approximation: selectively damped
least squares, target delta clamping, joint-specific velocity caps, and a
nullspace posture/joint-limit bias.
"""

from dataclasses import dataclass
import math
from pathlib import Path

import mujoco
import numpy as np

from anvil_openarm_spec import (
    ARM_COMMAND_ORDER,
    GRIPPER_METERS_RANGE,
    SIDE_ACT_PREFIX,
    TCP_SITE_NAMES,
)

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = ROOT / "models" / "anvil_openarm_bimanual.xml"
IK_NULLSPACE_NOMINALS = {
    "l": (0.0, -0.45, 0.0, 1.15, 0.0, 0.0, 0.0),
    "r": (0.0, 0.45, 0.0, 1.15, 0.0, 0.0, 0.0),
}


@dataclass(frozen=True)
class IkTuning:
    """Tunable constants for the local commanded-EE IK approximation."""

    max_iters: int = 80
    control_dt: float = 1.0 / 30.0
    position_tolerance: float = 0.004
    rotation_tolerance: float = 0.05
    orientation_weight: float = 0.35
    max_position_step: float = 0.035
    max_rotation_step: float = 0.16
    base_damping: float = 0.015
    singularity_damping: float = 0.08
    singularity_threshold: float = 0.10
    min_singular_value: float = 1e-5
    singular_direction_step: float = 0.04
    max_iteration_joint_step: float = 0.04
    nullspace_gain: float = 0.04
    joint_limit_gain: float = 0.025
    joint_limit_activation: float = 0.58
    joint_velocity_limits: tuple[float, ...] = (2.4, 2.0, 2.4, 2.4, 3.0, 3.0, 3.5)


DEFAULT_IK_TUNING = IkTuning()


class OpenArmSim:
    """Bimanual Anvil OpenARM 2.0 in MuJoCo with real-robot-shaped commands."""

    def __init__(
        self,
        model_path: str | Path = DEFAULT_MODEL,
        ik_tuning: IkTuning = DEFAULT_IK_TUNING,
    ):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.ik_tuning = ik_tuning
        kid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if kid >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, kid)
            self.data.ctrl[:] = self.model.key_ctrl[kid]

        self._cmd_actuators = {
            side: self._resolve_command_actuators(side) for side in ("l", "r")
        }
        self._arm_joint_ids = self._resolve_arm_joint_ids()
        self._arm_joint_limits = self._resolve_arm_joint_limits()
        self._nullspace_nominals = self._resolve_nullspace_nominals()
        self._tcp_sites = self._resolve_tcp_sites()
        # hold the initial pose: point every mapped actuator at its joint's
        # current position (clamped) so nothing jumps at startup
        for side, aids in self._cmd_actuators.items():
            for aid in aids:
                jid = self.model.actuator_trnid[aid, 0]
                q = self.data.qpos[self.model.jnt_qposadr[jid]]
                lo, hi = self.model.actuator_ctrlrange[aid]
                self.data.ctrl[aid] = float(np.clip(q, lo, hi))

        # joints reported in joint_states: every hinge/slide joint
        self._state_joints = [
            j
            for j in range(self.model.njnt)
            if self.model.jnt_type[j]
            in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)
        ]
        self._state_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
            for j in self._state_joints
        ]

    def _resolve_command_actuators(self, side: str) -> list[int]:
        """Actuator ids for a side, in the real robot's command order."""
        aids = []
        for jname in ARM_COMMAND_ORDER:
            # actuator names follow upstream: left_joint1_ctrl, left_finger1_ctrl
            short = jname.replace("finger_joint1", "finger1")
            aname = f"{SIDE_ACT_PREFIX[side]}{short}_ctrl"
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
            if aid < 0:
                raise ValueError(f"actuator '{aname}' not found in model")
            aids.append(aid)
        return aids

    def _resolve_arm_joint_ids(self) -> dict[str, list[int]]:
        joints = {}
        for side, aids in self._cmd_actuators.items():
            joints[side] = [
                int(self.model.actuator_trnid[aid, 0]) for aid in aids[:7]
            ]
        return joints

    def _resolve_arm_joint_limits(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        limits = {}
        for side, joint_ids in self._arm_joint_ids.items():
            ranges = np.array([self.model.jnt_range[jid] for jid in joint_ids])
            limits[side] = (ranges[:, 0], ranges[:, 1])
        return limits

    def _resolve_nullspace_nominals(self) -> dict[str, np.ndarray]:
        nominals = {}
        for side, values in IK_NULLSPACE_NOMINALS.items():
            lo, hi = self._arm_joint_limits[side]
            nominals[side] = np.clip(np.array(values, dtype=float), lo, hi)
        return nominals

    def _resolve_tcp_sites(self) -> dict[str, int]:
        sites = {}
        for side, name in TCP_SITE_NAMES.items():
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
            if sid < 0:
                raise ValueError(f"TCP site '{name}' not found in model")
            sites[side] = sid
        return sites

    # -- state ---------------------------------------------------------------

    @property
    def timestep(self) -> float:
        return float(self.model.opt.timestep)

    @property
    def time(self) -> float:
        return float(self.data.time)

    def command_actuators(self, side: str) -> list[int]:
        return list(self._cmd_actuators[side])

    def joint_states(self):
        """(names, position, velocity, effort) for all hinge/slide joints."""
        qpos = np.array(
            [self.data.qpos[self.model.jnt_qposadr[j]] for j in self._state_joints]
        )
        qvel = np.array(
            [self.data.qvel[self.model.jnt_dofadr[j]] for j in self._state_joints]
        )
        effort = np.array(
            [
                self.data.qfrc_actuator[self.model.jnt_dofadr[j]]
                for j in self._state_joints
            ]
        )
        return list(self._state_names), qpos, qvel, effort

    def ee_pose(self, side: str):
        """(position xyz, quaternion wxyz) of the side's TCP site."""
        try:
            sid = self._tcp_sites[side]
        except KeyError as exc:
            raise ValueError(f"unknown side '{side}'") from exc
        mujoco.mj_forward(self.model, self.data)
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, self.data.site_xmat[sid])
        return self.data.site_xpos[sid].copy(), quat

    # -- commands ------------------------------------------------------------

    def command_side(self, side: str, values) -> np.ndarray:
        """Apply per-arm position targets in the real robot's command order.

        Accepts 8 values (J1..J7 + gripper) or 7 (gripper left unchanged),
        mirroring the tolerance suggested by Anvil's docs. Targets are
        clamped to the actuator ctrlranges (the Anvil 2.0 joint limits).
        Returns the clamped targets actually applied.
        """
        values = np.asarray(values, dtype=float)
        aids = self._cmd_actuators[side]
        if values.shape[0] not in (len(aids), len(aids) - 1):
            raise ValueError(
                f"expected {len(aids)} (or {len(aids) - 1}) command values "
                f"for side '{side}', got {values.shape[0]}"
            )
        applied = np.empty(values.shape[0])
        for i, v in enumerate(values):
            aid = aids[i]
            lo, hi = self.model.actuator_ctrlrange[aid]
            target = float(np.clip(v, lo, hi))
            self.data.ctrl[aid] = target
            applied[i] = target
        return applied

    def command_ee(
        self,
        side: str,
        position,
        quaternion,
        gripper_m: float | None = None,
    ) -> np.ndarray:
        """Approximate Anvil's commanded-EE path with local IK.

        `position` is world xyz and `quaternion` is world wxyz for
        follower_{l,r}_hand_tcp. If `gripper_m` is passed, it is interpreted as
        the Anvil CommandedEEPose gripper opening in metres and mapped to this
        MuJoCo gripper's revolute command range.

        Returns the eight joint-space targets applied to the position
        actuators (J1..J7 + gripper).
        """
        if side not in self._cmd_actuators:
            raise ValueError(f"unknown side '{side}'")
        target_pos = np.asarray(position, dtype=float)
        if target_pos.shape != (3,) or not np.all(np.isfinite(target_pos)):
            raise ValueError("target position must contain three finite values")

        target_quat = np.asarray(quaternion, dtype=float)
        use_orientation = True
        if target_quat.shape != (4,) or not np.all(np.isfinite(target_quat)):
            raise ValueError("target quaternion must contain four finite values")
        qnorm = float(np.linalg.norm(target_quat))
        if qnorm < 1e-9:
            use_orientation = False
            target_quat = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            target_quat = target_quat / qnorm
        if gripper_m is not None and not np.isfinite(float(gripper_m)):
            raise ValueError("gripper opening must be finite")

        tuning = self.ik_tuning
        if len(tuning.joint_velocity_limits) != 7:
            raise ValueError("IK tuning must contain seven joint velocity limits")

        ik_data = mujoco.MjData(self.model)
        ik_data.qpos[:] = self.data.qpos
        ik_data.qvel[:] = self.data.qvel
        ik_data.ctrl[:] = self.data.ctrl
        mujoco.mj_forward(self.model, ik_data)

        sid = self._tcp_sites[side]
        joint_ids = self._arm_joint_ids[side]
        dof_ids = [int(self.model.jnt_dofadr[jid]) for jid in joint_ids]
        qpos_ids = [int(self.model.jnt_qposadr[jid]) for jid in joint_ids]
        joint_lo, joint_hi = self._arm_joint_limits[side]
        current_q = np.array([self.data.qpos[qpos_id] for qpos_id in qpos_ids])
        nominal_q = self._nullspace_nominals[side]

        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        for _ in range(tuning.max_iters):
            mujoco.mj_jacSite(self.model, ik_data, jacp, jacr, sid)
            current_quat = np.empty(4)
            mujoco.mju_mat2Quat(current_quat, ik_data.site_xmat[sid])
            pos_err = target_pos - ik_data.site_xpos[sid]
            rot_err = (
                _quat_error_vector(target_quat, current_quat)
                if use_orientation
                else np.zeros(3)
            )
            if (
                np.linalg.norm(pos_err) < tuning.position_tolerance
                and np.linalg.norm(rot_err) < tuning.rotation_tolerance
            ):
                break

            err = np.concatenate(
                [
                    _clamp_vector_norm(pos_err, tuning.max_position_step),
                    tuning.orientation_weight
                    * _clamp_vector_norm(rot_err, tuning.max_rotation_step),
                ]
            )
            jac = np.vstack(
                [
                    jacp[:, dof_ids],
                    tuning.orientation_weight * jacr[:, dof_ids],
                ]
            )
            q = np.array([ik_data.qpos[qpos_id] for qpos_id in qpos_ids])
            dq_task, jac_inv = _selectively_damped_least_squares(jac, err, tuning)
            dq_null = _nullspace_bias(q, nominal_q, joint_lo, joint_hi, tuning)
            null_projector = np.eye(len(joint_ids)) - jac_inv @ jac
            dq = dq_task + null_projector @ dq_null
            dq = _clamp_joint_step(dq, tuning.max_iteration_joint_step)
            q_next = np.clip(q + dq, joint_lo, joint_hi)
            for qpos_id, value in zip(qpos_ids, q_next):
                ik_data.qpos[qpos_id] = value
            mujoco.mj_forward(self.model, ik_data)

        solved_q = np.array([ik_data.qpos[qpos_id] for qpos_id in qpos_ids])
        targets = _apply_joint_velocity_limits(
            current_q, solved_q, tuning.joint_velocity_limits, tuning.control_dt
        )
        targets = np.clip(targets, joint_lo, joint_hi)
        command = np.empty(8)
        command[:7] = targets
        command[7] = (
            self._gripper_meters_to_ctrl(side, gripper_m)
            if gripper_m is not None
            else self.data.ctrl[self._cmd_actuators[side][7]]
        )
        return self.command_side(side, command)

    def _gripper_meters_to_ctrl(self, side: str, gripper_m: float) -> float:
        lo_m, hi_m = GRIPPER_METERS_RANGE
        s = np.clip((float(gripper_m) - lo_m) / (hi_m - lo_m), 0.0, 1.0)
        aid = self._cmd_actuators[side][7]
        lo, hi = self.model.actuator_ctrlrange[aid]
        closed = lo if abs(lo) < abs(hi) else hi
        open_ = hi if abs(hi) > abs(lo) else lo
        return float(closed + s * (open_ - closed))

    # -- stepping ------------------------------------------------------------

    def step(self, n: int | None = None, seconds: float | None = None) -> None:
        """Advance the simulation by n steps or a duration in sim seconds."""
        if (n is None) == (seconds is None):
            raise ValueError("pass exactly one of n= or seconds=")
        if seconds is not None:
            n = max(1, round(seconds / self.timestep))
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)


def _selectively_damped_least_squares(
    jac: np.ndarray, err: np.ndarray, tuning: IkTuning
) -> tuple[np.ndarray, np.ndarray]:
    """Return an SDLS-style joint update and generalized inverse.

    This follows the public Anvil description at the behavior level: each
    singular direction receives its own damping/cap, rather than using one
    scalar damping constant for the whole Jacobian.
    """
    u, singular_values, vt = np.linalg.svd(jac, full_matrices=False)
    joint_count = jac.shape[1]
    task_count = jac.shape[0]
    dq = np.zeros(joint_count)
    jac_inv = np.zeros((joint_count, task_count))

    for i, sigma in enumerate(singular_values):
        if sigma < tuning.min_singular_value:
            continue
        alpha = float(u[:, i] @ err)
        damping = _singular_direction_damping(sigma, tuning)
        coeff = sigma / (sigma * sigma + damping * damping)
        if abs(alpha) > 1e-12:
            coeff = min(coeff, tuning.singular_direction_step / abs(alpha))
        contribution = coeff * alpha * vt[i, :]
        dq += contribution
        jac_inv += coeff * np.outer(vt[i, :], u[:, i])

    return dq, jac_inv


def _singular_direction_damping(sigma: float, tuning: IkTuning) -> float:
    if sigma >= tuning.singularity_threshold:
        return tuning.base_damping
    depth = (tuning.singularity_threshold - sigma) / tuning.singularity_threshold
    return tuning.base_damping + tuning.singularity_damping * depth * depth


def _nullspace_bias(
    q: np.ndarray,
    nominal_q: np.ndarray,
    joint_lo: np.ndarray,
    joint_hi: np.ndarray,
    tuning: IkTuning,
) -> np.ndarray:
    posture = tuning.nullspace_gain * (nominal_q - q)

    center = 0.5 * (joint_lo + joint_hi)
    half_range = np.maximum(0.5 * (joint_hi - joint_lo), 1e-6)
    normalized = (q - center) / half_range
    activation = np.clip(
        (np.abs(normalized) - tuning.joint_limit_activation)
        / (1.0 - tuning.joint_limit_activation),
        0.0,
        1.0,
    )
    limit_avoidance = (
        -tuning.joint_limit_gain * activation * normalized * half_range
    )

    return posture + limit_avoidance


def _apply_joint_velocity_limits(
    current_q: np.ndarray,
    desired_q: np.ndarray,
    velocity_limits,
    control_dt: float,
) -> np.ndarray:
    max_delta = np.asarray(velocity_limits, dtype=float) * float(control_dt)
    if max_delta.shape != current_q.shape:
        raise ValueError("velocity limit shape must match joint vector")
    delta = np.clip(desired_q - current_q, -max_delta, max_delta)
    return current_q + delta


def _clamp_joint_step(dq: np.ndarray, max_norm: float) -> np.ndarray:
    return np.clip(dq, -max_norm, max_norm)


def _clamp_vector_norm(vec: np.ndarray, max_norm: float) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm <= max_norm or norm < 1e-12:
        return vec
    return vec * (max_norm / norm)


def _quat_error_vector(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Small-angle rotation vector that moves current wxyz toward target wxyz."""
    if float(np.dot(target, current)) < 0.0:
        target = -target
    err = _quat_multiply(target, _quat_conjugate(current))
    if err[0] < 0.0:
        err = -err
    return 2.0 * err[1:]


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def _quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    out = np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )
    norm = float(np.linalg.norm(out))
    if norm < math.sqrt(np.finfo(float).eps):
        return np.array([1.0, 0.0, 0.0, 0.0])
    return out / norm
