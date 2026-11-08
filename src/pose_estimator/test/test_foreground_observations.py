"""Pure foreground contract: metric geometry, angular support and no gravity prior."""
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pose_estimator.foreground_observations import extract_foreground


def block(center=(0., 0., 1.), n=12):
    u, v = np.meshgrid(np.linspace(-.025, .025, n), np.linspace(-.025, .025, n))
    return np.column_stack((u.ravel(), v.ravel(), (.008 * np.sin(np.pi * u / .025)).ravel())) + center


def test_compact_block_and_rotations_have_same_center():
    a = block()
    result = extract_foreground(a, 366., 366.)
    assert result.reason == 'accepted'
    assert np.linalg.norm(result.center - [0., 0., 1.]) < .003
    for axis in ['x', 'z']:
        q = Rotation.from_euler(axis, 15, degrees=True).as_matrix()
        rotated = extract_foreground(a @ q.T, 366., 366.)
        assert rotated.reason == 'accepted'
        assert np.linalg.norm(rotated.center - result.center @ q.T) < .004


def test_range_support_has_fraction_and_absolute_floor():
    a = block(center=(0., 0., 3.), n=6)
    assert extract_foreground(a, 366., 366.).reason == 'accepted'
    assert extract_foreground(a[:8], 366., 366.).reason == 'insufficient_points'
    # Same sparse count very close to camera cannot support expected block pixels.
    assert extract_foreground(a - [0, 0, 2.8], 366., 366.).center is None


def test_ground_only_large_and_block_sized_planes_are_rejected():
    u, v = np.meshgrid(np.linspace(-.3, .3, 80), np.linspace(-.3, .3, 80))
    large = np.column_stack((u.ravel(), v.ravel(), np.ones(u.size)))
    assert extract_foreground(large, 366., 366.).reason == 'dominant_background'
    small = block(); small[:, 2] = 1.
    assert extract_foreground(small, 366., 366.).reason == 'planar_background'


def test_oversized_background_not_carved_into_false_blocks():
    u, v = np.meshgrid(np.linspace(-.2, .2, 60), np.linspace(-.2, .2, 60))
    background = np.column_stack((u.ravel(), v.ravel(), np.ones(u.size)))
    scene = np.concatenate((background, block(center=(0., 0., .8))))
    assert extract_foreground(scene, 366., 366.).reason == 'dominant_background'


def test_finite_validation_and_no_input_mutation():
    a = block(); before = a.copy()
    assert extract_foreground(np.concatenate((a, [[np.nan, 0, 1], [0, 0, -1]])), 366., 366.).center is not None
    assert np.array_equal(a, before)
    assert extract_foreground(a, 0, 366.).reason == 'invalid_parameters'
    assert extract_foreground(np.ones(3), 366., 366.).reason == 'invalid_shape'


def test_tiny_compact_fragment_cannot_support_an_enormous_detector_roi():
    u, v = np.meshgrid(np.arange(100)*.05, np.arange(100)*.05)
    disconnected_background = np.column_stack((u.ravel(), v.ravel(), np.ones(u.size)))
    scene = np.concatenate((disconnected_background, block(center=(-.2, -.2, .8))))
    result = extract_foreground(scene, 366., 366.)
    assert result.center is None
    assert result.reason == 'roi_occupancy'
