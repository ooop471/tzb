"""Solve the eye-in-hand calibration equation A X = X B.

``A`` is the end-effector pose in the robot base frame, ``B`` is the board pose
in the camera frame and ``X`` is the camera pose in the end-effector frame:
``T_base_target = T_base_ee @ X @ T_cam_target`` stays constant for a fixed
board, which is exactly the classic hand-eye problem.

OpenCV provides closed-form initial guesses (Tsai, Park, Horaud, Andreff,
Daniilidis); the best one is refined with a nonlinear least-squares pass and
gross outliers are removed iteratively.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from hand_eye_calib.geometry import (average_transform, make_transform,
                                     matrix_to_rotvec, rotation_angle_deg)

CV_METHODS = {
    'TSAI': cv2.CALIB_HAND_EYE_TSAI,
    'PARK': cv2.CALIB_HAND_EYE_PARK,
    'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
    'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}
METHOD_ORDER = ('PARK', 'TSAI', 'HORAUD', 'ANDREFF', 'DANIILIDIS')

MIN_SAMPLES = 3


class NotEnoughSamplesError(ValueError):
    """Raised when fewer than three usable samples were captured."""


@dataclass
class CalibrationResult:
    T_ee_camera: np.ndarray
    T_base_target: np.ndarray
    T_base_camera: np.ndarray
    method: str
    refined: bool
    total_samples: int
    used: list[int]
    rejected: list[int]
    rotation_rms_deg: float
    rotation_max_deg: float
    translation_rms_mm: float
    translation_max_mm: float
    per_sample: list[dict] = field(default_factory=list)


def _nearest_rotation(matrix: np.ndarray) -> np.ndarray:
    """Project a (possibly reflection-valued) 3x3 matrix onto SO(3)."""
    left, _, right = np.linalg.svd(np.asarray(matrix, dtype=float))
    sign = 1.0 if np.linalg.det(left @ right) >= 0.0 else -1.0
    return left @ np.diag([1.0, 1.0, sign]) @ right


def _closed_form(base_ee: Sequence[np.ndarray], cam_target: Sequence[np.ndarray],
                 method: str) -> np.ndarray:
    rotations_gripper = [T[:3, :3] for T in base_ee]
    translations_gripper = [T[:3, 3].reshape(3, 1) for T in base_ee]
    rotations_target = [T[:3, :3] for T in cam_target]
    translations_target = [T[:3, 3].reshape(3, 1) for T in cam_target]

    rotation, translation = cv2.calibrateHandEye(
        rotations_gripper, translations_gripper,
        rotations_target, translations_target,
        method=CV_METHODS[method])
    # Degenerate motions can make OpenCV return a left-handed matrix.
    rotation = _nearest_rotation(np.asarray(rotation).reshape(3, 3))
    return make_transform(rotation, np.asarray(translation).reshape(3))


def _refine(base_ee: Sequence[np.ndarray], cam_target: Sequence[np.ndarray],
            initial: np.ndarray, max_nfev: int = 200) -> np.ndarray:
    def residuals(params: np.ndarray) -> np.ndarray:
        candidate = make_transform(Rotation.from_rotvec(params[:3]).as_matrix(),
                                   params[3:])
        reference = base_ee[0] @ candidate @ cam_target[0]
        values: list[float] = []
        for end_effector, target in zip(base_ee, cam_target):
            predicted = end_effector @ candidate @ target
            delta = reference[:3, :3].T @ predicted[:3, :3]
            values.extend(Rotation.from_matrix(delta).as_rotvec())
            # Translation in millimetres balances rotation in radians.
            values.extend((predicted[:3, 3] - reference[:3, 3]) * 1000.0)
        return np.asarray(values)

    start = np.concatenate([matrix_to_rotvec(initial[:3, :3]), initial[:3, 3]])
    try:
        solution = least_squares(residuals, start, method='trf', x_scale='jac',
                                 loss='soft_l1', f_scale=1.0, max_nfev=max_nfev)
    except (ValueError, np.linalg.LinAlgError):
        return initial
    if not np.all(np.isfinite(solution.x)):
        return initial
    return make_transform(Rotation.from_rotvec(solution.x[:3]).as_matrix(),
                          solution.x[3:])


def evaluate_pose(transform: np.ndarray, base_ee: Sequence[np.ndarray],
                  cam_target: Sequence[np.ndarray]):
    """Per-sample deviation from the mean predicted board pose in the base frame."""
    predicted = [end_effector @ transform @ target
                 for end_effector, target in zip(base_ee, cam_target)]
    reference = average_transform(predicted)
    angles = []
    distances = []
    for pose in predicted:
        angles.append(rotation_angle_deg(reference[:3, :3].T @ pose[:3, :3]))
        distances.append(float(np.linalg.norm(pose[:3, 3] - reference[:3, 3]) * 1000.0))
    return angles, distances, reference


def _solve_candidate(base_ee, cam_target, method: str, refine: bool,
                     max_rot_deg: float, max_trans_mm: float):
    try:
        closed_form = _closed_form(base_ee, cam_target, method)
    except (cv2.error, ValueError, np.linalg.LinAlgError):
        return None, float('inf')

    solution = _refine(base_ee, cam_target, closed_form) if refine else closed_form
    angles, distances, _ = evaluate_pose(solution, base_ee, cam_target)
    cost = float(np.mean((np.asarray(angles) / max_rot_deg) ** 2
                         + (np.asarray(distances) / max_trans_mm) ** 2))
    return solution, cost


def solve_hand_eye(base_ee: Sequence[np.ndarray], cam_target: Sequence[np.ndarray],
                   method: str = 'AUTO', refine: bool = True,
                   min_samples: int = 4, max_rot_deg: float = 2.0,
                   max_trans_mm: float = 10.0,
                   labels: Optional[Sequence[str]] = None) -> CalibrationResult:
    if len(base_ee) != len(cam_target):
        raise ValueError('base_ee and cam_target must have the same length')
    if len(base_ee) < MIN_SAMPLES:
        raise NotEnoughSamplesError(
            f'at least {MIN_SAMPLES} samples are required, got {len(base_ee)}')

    requested = method.strip().upper()
    candidates = METHOD_ORDER if requested in ('', 'AUTO') else (requested,)
    if any(name not in CV_METHODS for name in candidates):
        raise ValueError(f'unknown method {method!r}; expected AUTO or one of '
                         f'{", ".join(METHOD_ORDER)}')

    keep = list(range(len(base_ee)))
    rejected: list[int] = []
    floor = max(MIN_SAMPLES, int(min_samples))

    while True:
        subset_a = [base_ee[i] for i in keep]
        subset_b = [cam_target[i] for i in keep]

        solution = None
        best_cost = float('inf')
        best_method = candidates[0]
        for name in candidates:
            candidate, cost = _solve_candidate(subset_a, subset_b, name, refine,
                                               max_rot_deg, max_trans_mm)
            if candidate is not None and cost < best_cost:
                solution, best_cost, best_method = candidate, cost, name
        if solution is None:
            raise RuntimeError('all hand-eye solvers failed on the captured samples')

        angles, distances, base_target = evaluate_pose(solution, subset_a, subset_b)
        scores = [max(angle / max_rot_deg, distance / max_trans_mm)
                  for angle, distance in zip(angles, distances)]
        worst = int(np.argmax(scores))

        if scores[worst] > 1.0 and len(keep) > floor:
            rejected.append(keep.pop(worst))
            continue
        break

    labels = list(labels) if labels is not None else [str(i) for i in range(len(base_ee))]
    per_sample = [
        {
            'index': keep[position],
            'label': labels[keep[position]],
            'rotation_deg': float(angles[position]),
            'translation_mm': float(distances[position]),
        }
        for position in range(len(keep))
    ]

    return CalibrationResult(
        T_ee_camera=solution,
        T_base_target=base_target,
        T_base_camera=base_ee[keep[0]] @ solution,
        method=best_method,
        refined=bool(refine),
        total_samples=len(base_ee),
        used=list(keep),
        rejected=sorted(rejected),
        rotation_rms_deg=float(np.sqrt(np.mean(np.square(angles)))),
        rotation_max_deg=float(np.max(angles)),
        translation_rms_mm=float(np.sqrt(np.mean(np.square(distances)))),
        translation_max_mm=float(np.max(distances)),
        per_sample=per_sample,
    )


def rotation_diversity_deg(transforms: Iterable[np.ndarray]) -> float:
    """Largest pairwise rotation difference, used to warn about degenerate data."""
    transforms = list(transforms)
    largest = 0.0
    for first in range(len(transforms)):
        for second in range(first + 1, len(transforms)):
            relative = transforms[first][:3, :3].T @ transforms[second][:3, :3]
            largest = max(largest, rotation_angle_deg(relative))
    return largest
