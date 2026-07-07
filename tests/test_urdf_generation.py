"""Tests for the generated Anvil URDF and its parity with the MJCF model."""

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

import anvil_openarm_spec as spec
from scripts import make_anvil_urdf

URDF_PATH = make_anvil_urdf.OUT_DIR / make_anvil_urdf.OUT_NAME
MJCF_PATH = make_anvil_urdf.OUT_DIR / "anvil_openarm_bimanual.xml"

TOL = 2e-3  # rad; upstream XML rounds to ~5 significant figures


def urdf_root() -> ET.Element:
    return ET.parse(URDF_PATH).getroot()


def revolute_joints(root: ET.Element) -> dict[str, ET.Element]:
    # Direct children only: the <ros2_control> block nests its own <joint>
    # elements, which findall on the root does not descend into.
    return {
        j.get("name"): j
        for j in root.findall("joint")
        if j.get("type") == "revolute"
    }


def floats(attr: str) -> np.ndarray:
    return np.array([float(v) for v in attr.split()])


def rpy_to_mat(rpy: np.ndarray) -> np.ndarray:
    """URDF fixed-axis roll/pitch/yaw: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def origin_transform(joint: ET.Element) -> tuple[np.ndarray, np.ndarray]:
    origin = joint.find("origin")
    xyz = floats(origin.get("xyz", "0 0 0"))
    rpy = floats(origin.get("rpy", "0 0 0"))
    return rpy_to_mat(rpy), xyz


def test_generated_urdf_is_reproducible(tmp_path):
    assert make_anvil_urdf.generate(tmp_path, verbose=False) == 0
    assert (tmp_path / make_anvil_urdf.OUT_NAME).read_text() == URDF_PATH.read_text()


def test_tcp_rpy_matches_tcp_quat():
    """The spec's URDF rpy and MJCF quat describe the same rotation."""
    quat_mat = np.zeros(9)
    mujoco.mju_quat2Mat(quat_mat, np.array(spec.TCP_SITE_QUAT))
    np.testing.assert_allclose(
        rpy_to_mat(np.array(spec.TCP_SITE_RPY)),
        quat_mat.reshape(3, 3),
        atol=1e-8,
    )


def test_patched_limits_match_spec():
    joints = revolute_joints(urdf_root())
    for prefix in spec.SIDE_PREFIX.values():
        for jname, (lo, hi) in spec.ARM_JOINT_RANGES.items():
            limit = joints[f"{prefix}{jname}"].find("limit")
            assert float(limit.get("lower")) == pytest.approx(lo, abs=TOL)
            assert float(limit.get("upper")) == pytest.approx(hi, abs=TOL)
    for jname, (lo, hi) in spec.SIDE_JOINT_RANGES.items():
        limit = joints[jname].find("limit")
        assert float(limit.get("lower")) == pytest.approx(lo, abs=TOL)
        assert float(limit.get("upper")) == pytest.approx(hi, abs=TOL)


def test_joint_parity_with_mjcf():
    """Every named joint agrees with the generated MJCF: axis and range."""
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    urdf_joints = revolute_joints(urdf_root())
    mjcf_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        for j in range(model.njnt)
    }
    assert set(urdf_joints) == mjcf_names
    for name, joint in urdf_joints.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        np.testing.assert_array_equal(
            floats(joint.find("axis").get("xyz")), model.jnt_axis[jid], err_msg=name
        )
        limit = joint.find("limit")
        assert float(limit.get("lower")) == pytest.approx(
            model.jnt_range[jid][0], abs=1e-12
        ), name
        assert float(limit.get("upper")) == pytest.approx(
            model.jnt_range[jid][1], abs=1e-12
        ), name


def test_tcp_fixed_frames():
    root = urdf_root()
    links = {li.get("name") for li in root.findall("link")}
    fixed = {
        j.get("name"): j for j in root.findall("joint") if j.get("type") == "fixed"
    }
    for side, name in spec.TCP_SITE_NAMES.items():
        assert name in links
        joint = fixed[f"{name}_joint"]
        assert joint.find("parent").get("link") == spec.TCP_SITE_BODY_NAMES[side]
        assert joint.find("child").get("link") == name
        rot, xyz = origin_transform(joint)
        np.testing.assert_allclose(xyz, spec.TCP_SITE_POS, atol=1e-12)
        quat_mat = np.zeros(9)
        mujoco.mju_quat2Mat(quat_mat, np.array(spec.TCP_SITE_QUAT))
        np.testing.assert_allclose(rot, quat_mat.reshape(3, 3), atol=1e-8)


def test_tcp_world_pose_parity_with_mjcf():
    """FK through the whole URDF arm at zero pose lands the TCP exactly where
    the MJCF site sits, relative to the arm's base link — i.e. the two
    descriptions agree kinematically, not just per-attribute."""
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    root = urdf_root()
    child_to_joint = {j.find("child").get("link"): j for j in root.findall("joint")}

    for side in ("l", "r"):
        base = f"{spec.SIDE_PREFIX[side]}base_link"
        # Compose joint origins from the TCP back up to the base link; every
        # revolute joint is at its zero angle, so only origins contribute.
        rot = np.eye(3)
        pos = np.zeros(3)
        link = spec.TCP_SITE_NAMES[side]
        while link != base:
            joint = child_to_joint[link]
            j_rot, j_pos = origin_transform(joint)
            rot = j_rot @ rot
            pos = j_pos + j_rot @ pos
            link = joint.find("parent").get("link")

        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, base)
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, spec.TCP_SITE_NAMES[side])
        assert bid >= 0 and sid >= 0
        base_rot = data.xmat[bid].reshape(3, 3)
        rel_pos = base_rot.T @ (data.site_xpos[sid] - data.xpos[bid])
        rel_rot = base_rot.T @ data.site_xmat[sid].reshape(3, 3)
        np.testing.assert_allclose(pos, rel_pos, atol=1e-6, err_msg=side)
        np.testing.assert_allclose(rot, rel_rot, atol=1e-6, err_msg=side)


def test_wrist_bracket_visuals():
    root = urdf_root()
    top_materials = {m.get("name"): m for m in root.findall("material")}
    assert set(top_materials) == {
        spec.WRIST_BRACKET_MATERIAL,
        spec.WRIST_BRACKET_SCREW_MATERIAL,
    }
    for name, rgba in (
        (spec.WRIST_BRACKET_MATERIAL, spec.WRIST_BRACKET_RGBA),
        (spec.WRIST_BRACKET_SCREW_MATERIAL, spec.WRIST_BRACKET_SCREW_RGBA),
    ):
        got = floats(top_materials[name].find("color").get("rgba"))
        np.testing.assert_allclose(got, rgba, atol=1e-9)

    links = {li.get("name"): li for li in root.findall("link")}
    for side in ("l", "r"):
        link = links[spec.WRIST_BRACKET_BODY_NAMES[side]]
        sy = -1.0 if side == "r" else 1.0
        prefix = spec.WRIST_BRACKET_MESH_NAMES[side]
        visuals = {
            v.get("name"): v
            for v in link.findall("visual")
            if v.get("name", "").startswith(prefix)
        }

        # The CAD STL mesh, right side mirrored via a negative y scale
        # (exactly like the MJCF mesh asset).
        mesh_visual = visuals.pop(prefix)
        mesh = mesh_visual.find("geometry/mesh")
        assert mesh.get("filename") == make_anvil_urdf.BRACKET_MESH_PATH
        np.testing.assert_allclose(
            floats(mesh.get("scale")),
            floats(spec.WRIST_BRACKET_MESH_SCALES[side]),
            atol=1e-15,
        )
        assert mesh_visual.find("material").get("name") == spec.WRIST_BRACKET_MATERIAL

        # Screw heads and forearm standoff as cylinders matching the spec
        # ((from), (to), radius) definitions, mirrored in y.
        expected = {
            name: (cyl, spec.WRIST_BRACKET_SCREW_MATERIAL)
            for name, cyl in zip(
                spec.wrist_bracket_screw_geom_names(side).values(),
                spec.WRIST_BRACKET_SCREW_CYLINDERS.values(),
            )
        }
        expected.update(
            {
                name: (cyl, spec.WRIST_BRACKET_MATERIAL)
                for name, cyl in zip(
                    spec.wrist_bracket_link5_geom_names(side).values(),
                    spec.WRIST_BRACKET_LINK5_CYLINDERS.values(),
                )
            }
        )
        assert set(visuals) == set(expected)
        for name, (cylinder, material) in expected.items():
            visual = visuals[name]
            start, end, radius = cylinder
            start = np.array(start) * [1.0, sy, 1.0]
            end = np.array(end) * [1.0, sy, 1.0]
            xyz = floats(visual.find("origin").get("xyz"))
            np.testing.assert_allclose(xyz, (start + end) / 2, atol=1e-12)
            cyl_el = visual.find("geometry/cylinder")
            assert float(cyl_el.get("radius")) == radius
            length = np.linalg.norm(end - start)
            np.testing.assert_allclose(float(cyl_el.get("length")), length, atol=1e-12)
            # The rpy must rotate the URDF cylinder's local z onto the spec
            # cylinder's axis.
            rot, _ = origin_transform(visual)
            axis = (end - start) / length
            np.testing.assert_allclose(
                np.abs(rot @ [0.0, 0.0, 1.0]), np.abs(axis), atol=1e-9
            )
            assert visual.find("material").get("name") == material
        # Visual-only, exactly like the MJCF bracket: no collision blocks.
        for coll in link.findall("collision"):
            assert not coll.get("name", "").startswith("anvil_wrist_bracket")


def test_mesh_paths_resolve():
    meshes = [
        m.get("filename")
        for m in urdf_root().iter("mesh")
    ]
    assert meshes, "expected mesh references in the URDF"
    assert make_anvil_urdf.BRACKET_MESH_PATH in meshes
    for filename in meshes:
        assert filename.startswith(
            (make_anvil_urdf.MESH_PREFIX, "assets/")
        ), filename
        assert (make_anvil_urdf.OUT_DIR / filename).resolve().is_file(), filename
