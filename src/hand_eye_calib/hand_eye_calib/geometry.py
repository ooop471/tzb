"""4x4 rigid transform helpers shared by the capture node and the solver."""

from __future__ import annotations

from typing import Iterable, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

# Rotation from a ROS camera optical frame (x right, y down, z forward)
# to the ROS camera body/link frame (x forward, y left, z up).
R_LINK_FROM_OPTICAL = np.array(
    [
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=float,
)


def make_transform(rotation: np.ndarray | Sequence | None = None,
                   translation: np.ndarray | Sequence | None = None) -> np.ndarray:
    """Build a 4x4 homogeneous transform."""
    transform = np.eye(4)
    if rotation is not None:
        transform[:3, :3] = np.asarray(rotation, dtype=float).reshape(3, 3)
    if translation is not None:
        transform[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return transform


def transform_from_xyz_rpy(xyz: Sequence[float], rpy: Sequence[float]) -> np.ndarray:
    """Transform from a translation and fixed-axis (roll, pitch, yaw) angles."""
    return make_transform(rpy_to_matrix(rpy), xyz)


def transform_from_xyz_quat(xyz: Sequence[float], quat_xyzw: Sequence[float]) -> np.ndarray:
    """Transform from a translation and an (x, y, z, w) quaternion."""
    return make_transform(quat_to_matrix(quat_xyzw), xyz)


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Invert a rigid 4x4 transform."""
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def rpy_to_matrix(rpy: Sequence[float]) -> np.ndarray:
    roll, pitch, yaw = (float(v) for v in rpy)
    return Rotation.from_euler('xyz', [roll, pitch, yaw]).as_matrix()


def matrix_to_rpy(rotation: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(rotation).as_euler('xyz')


def quat_to_matrix(quat_xyzw: Sequence[float]) -> np.ndarray:
    return Rotation.from_quat(np.asarray(quat_xyzw, dtype=float).reshape(4)).as_matrix()


def matrix_to_quat(rotation: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(rotation).as_quat()


def rotvec_to_matrix(rotvec: Sequence[float]) -> np.ndarray:
    return Rotation.from_rotvec(np.asarray(rotvec, dtype=float).reshape(3)).as_matrix()


def matrix_to_rotvec(rotation: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(rotation).as_rotvec()


def rotation_angle_deg(rotation: np.ndarray) -> float:
    """Geodesic angle of a rotation matrix, in degrees."""
    return float(np.degrees(Rotation.from_matrix(rotation).magnitude()))


def pose_delta_deg_mm(first: np.ndarray, second: np.ndarray) -> tuple[float, float]:
    """Rotation difference in degrees and translation difference in millimetres."""
    relative = first[:3, :3].T @ second[:3, :3]
    angle_deg = rotation_angle_deg(relative)
    distance_mm = float(np.linalg.norm(second[:3, 3] - first[:3, 3]) * 1000.0)
    return angle_deg, distance_mm


def average_transform(transforms: Iterable[np.ndarray]) -> np.ndarray:
    """Average a set of rigid transforms (translation mean, rotation via quaternion mean)."""
    stacked = np.asarray(list(transforms), dtype=float)
    if stacked.ndim != 3 or stacked.shape[0] == 0:
        raise ValueError('average_transform needs at least one 4x4 transform')

    translation = stacked[:, :3, 3].mean(axis=0)
    quaternions = Rotation.from_matrix(stacked[:, :3, :3]).as_quat()

    reference = quaternions[0]
    signs = np.where(quaternions @ reference < 0.0, -1.0, 1.0)
    quaternions = quaternions * signs[:, None]

    scatter = quaternions.T @ quaternions
    _, eigenvectors = np.linalg.eigh(scatter)
    quaternion = eigenvectors[:, -1]
    quaternion /= np.linalg.norm(quaternion)
    if quaternion @ reference < 0.0:
        quaternion = -quaternion

    return make_transform(Rotation.from_quat(quaternion).as_matrix(), translation)


def transform_to_dict(transform: np.ndarray) -> dict:
    """Serialisable summary of a transform for the session YAML."""
    rotation = transform[:3, :3]
    return {
        'matrix': [[float(v) for v in row] for row in transform],
        'xyz': [float(v) for v in transform[:3, 3]],
        'quat_xyzw': [float(v) for v in matrix_to_quat(rotation)],
        'rpy_deg': [float(v) for v in np.degrees(matrix_to_rpy(rotation))],
    }


def optical_to_link(transform: np.ndarray) -> np.ndarray:
    """Re-express a transform whose rotation maps to optical axes using body axes."""
    return make_transform(transform[:3, :3] @ R_LINK_FROM_OPTICAL, transform[:3, 3])


def format_matrix(matrix: np.ndarray, precision: int = 6) -> str:
    """Format a 4x4 matrix for console output."""
    return np.array2string(np.asarray(matrix, dtype=float), precision=precision,
                           suppress_small=True, max_line_width=120)
