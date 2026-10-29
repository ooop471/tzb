import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from hand_eye_calib.geometry import invert_transform, make_transform
from hand_eye_calib.solver import (NotEnoughSamplesError, rotation_diversity_deg,
                                   solve_hand_eye)


def _random_transform(rng, translation_scale):
    rotation = Rotation.random(random_state=rng).as_matrix()
    translation = rng.uniform(-translation_scale, translation_scale, size=3)
    return make_transform(rotation, translation)


def _synthetic_dataset(rng, count=8):
    camera_from_ee = _random_transform(rng, 0.1)
    base_from_target = _random_transform(rng, 0.4)
    base_from_ee = []
    cam_from_target = []
    for _ in range(count):
        end_effector = _random_transform(rng, 0.3)
        base_from_ee.append(end_effector)
        cam_from_target.append(invert_transform(end_effector @ camera_from_ee)
                               @ base_from_target)
    return camera_from_ee, base_from_target, base_from_ee, cam_from_target


@pytest.mark.parametrize('method', ['AUTO', 'TSAI', 'PARK', 'HORAUD', 'ANDREFF',
                                    'DANIILIDIS'])
def test_solve_recovers_exact_transform(method):
    rng = np.random.default_rng(7)
    expected, _, base_ee, cam_target = _synthetic_dataset(rng)

    result = solve_hand_eye(base_ee, cam_target, method=method)

    np.testing.assert_allclose(result.T_ee_camera, expected, atol=1e-6)
    assert result.rotation_max_deg < 1e-4
    assert result.translation_max_mm < 1e-4
    assert result.used == list(range(len(base_ee)))


def test_rejects_gross_outlier():
    rng = np.random.default_rng(11)
    expected, _, base_ee, cam_target = _synthetic_dataset(rng)
    broken = cam_target[3].copy()
    broken[:3, :3] = Rotation.from_euler('y', 25.0, degrees=True).as_matrix() @ broken[:3, :3]
    broken[:3, 3] += np.array([0.05, -0.02, 0.03])
    cam_target[3] = broken

    result = solve_hand_eye(base_ee, cam_target)

    assert 3 in result.rejected
    np.testing.assert_allclose(result.T_ee_camera, expected, atol=1e-4)


def test_requires_three_samples():
    with pytest.raises(NotEnoughSamplesError):
        solve_hand_eye([np.eye(4), np.eye(4)], [np.eye(4), np.eye(4)])


def test_rotation_diversity():
    identity = np.eye(4)
    tilted = make_transform(
        Rotation.from_euler('x', 20.0, degrees=True).as_matrix(), [0.0, 0.0, 0.0])
    assert rotation_diversity_deg([identity, tilted]) == pytest.approx(20.0, abs=1e-6)
    assert rotation_diversity_deg([identity]) == 0.0


def test_nearest_rotation_projects_reflections():
    from hand_eye_calib.solver import _nearest_rotation

    reflected = np.diag([1.0, 1.0, -1.0])
    projected = _nearest_rotation(reflected)

    assert np.linalg.det(projected) == pytest.approx(1.0)
    np.testing.assert_allclose(projected @ projected.T, np.eye(3), atol=1e-12)
