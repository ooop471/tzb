"""Seed is acquisition: same final-quality gate and global ICP trust semantics."""
from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pose_estimator.pose_estimation_node import PoseEstimationNode


class Log:
    def warn(self, *args, **kwargs):
        pass
    def debug(self, *args, **kwargs):
        pass


def make_shell(seed_metrics):
    calls = []
    class Shell:
        _register = PoseEstimationNode._register
        _metrics_ok = PoseEstimationNode._metrics_ok
        def get_logger(self):
            return Log()
        def _pose_jump_ok(self, old, new):
            return True, 'unchanged'
        def _run_protected_icp(self, scene, initial, global_mode):
            calls.append((initial.copy(), global_mode))
            return initial.copy(), seed_metrics if global_mode else {'coverage': 0., 'median': 1.}
    s = Shell()
    s._p = {'tracking_min_scene_coverage': .35, 'tracking_max_scene_median': .006,
            'icp_min_scene_coverage': .45, 'icp_max_scene_median': .005,
            'global_ransac_enable': False}
    s._tracking = True
    s._last_transform = np.eye(4)
    s.model = SimpleNamespace(centroid=np.array([.008, .008, .008]))
    return s, calls


def test_failed_tracking_reseeds_model_center_and_uses_global_icp():
    s, calls = make_shell({'coverage': .6, 'median': .002})
    foreground = np.array([.1, .2, .7])
    transform, coverage, median = s._register(None, foreground_center=foreground)
    assert [global_mode for _, global_mode in calls] == [False, True]
    assert np.allclose(transform[:3, :3] @ s.model.centroid + transform[:3, 3], foreground)
    assert s._tracking
    assert s._last_reg_timing['reg_mode'] == 'SEEDED'


def test_reseed_cannot_bypass_existing_stricter_final_quality_gate():
    # Good enough for tracking, but below the existing global final coverage .45.
    s, calls = make_shell({'coverage': .4, 'median': .004})
    transform, _, _ = s._register(None, foreground_center=np.array([.1, .2, .7]))
    assert transform is None
    assert not s._tracking
    assert len(calls) == 2
