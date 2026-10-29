import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from hand_eye_calib import geometry


def test_transform_roundtrip():
    rotation = Rotation.from_euler('xyz', [0.3, -0.2, 1.1]).as_matrix()
    translation = np.array([0.1, -0.2, 0.3])
    transform = geometry.make_transform(rotation, translation)

    inverse = geometry.invert_transform(transform)
    np.testing.assert_allclose(transform @ inverse, np.eye(4), atol=1e-12)

    point = np.array([0.4, -0.1, 0.7])
    homogeneous = np.append(point, 1.0)
    back = inverse @ (transform @ homogeneous)
    np.testing.assert_allclose(back, homogeneous, atol=1e-12)


def test_rotation_conversions_are_consistent():
    rotation = Rotation.from_rotvec([0.4, 0.3, -0.8]).as_matrix()

    np.testing.assert_allclose(
        geometry.rotvec_to_matrix(geometry.matrix_to_rotvec(rotation)), rotation,
        atol=1e-12)
    np.testing.assert_allclose(
        geometry.quat_to_matrix(geometry.matrix_to_quat(rotation)), rotation,
        atol=1e-12)
    np.testing.assert_allclose(
        geometry.rpy_to_matrix(geometry.matrix_to_rpy(rotation)), rotation,
        atol=1e-12)


def test_pose_delta_deg_mm():
    first = geometry.make_transform(np.eye(3), [0.0, 0.0, 0.0])
    second = geometry.make_transform(
        Rotation.from_euler('z', 10.0, degrees=True).as_matrix(),
        [0.010, 0.0, 0.0])

    angle_deg, distance_mm = geometry.pose_delta_deg_mm(first, second)
    assert angle_deg == pytest.approx(10.0, abs=1e-6)
    assert distance_mm == pytest.approx(10.0, abs=1e-6)


def test_average_transform_is_robust_to_quaternion_signs():
    rotation = Rotation.from_euler('xyz', [0.1, 0.2, 0.3]).as_matrix()
    translation = np.array([0.2, 0.1, -0.3])
    transforms = []
    for offset in (-0.002, 0.0, 0.002):
        transforms.append(geometry.make_transform(rotation, translation + offset))

    average = geometry.average_transform(transforms)
    np.testing.assert_allclose(average[:3, :3], rotation, atol=1e-9)
    np.testing.assert_allclose(average[:3, 3], translation, atol=1e-9)


def test_optical_to_link_matches_ros_camera_convention():
    optical = geometry.make_transform(np.eye(3), [0.01, 0.02, 0.03])
    link = geometry.optical_to_link(optical)

    # x_optical is z_link, y_optical is -x_link, z_optical is -y_link.
    rotation = link[:3, :3]
    np.testing.assert_allclose(rotation[:, 0], [0.0, 0.0, 1.0], atol=1e-12)
    np.testing.assert_allclose(rotation[:, 1], [-1.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(rotation[:, 2], [0.0, -1.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(link[:3, 3], optical[:3, 3], atol=1e-12)
