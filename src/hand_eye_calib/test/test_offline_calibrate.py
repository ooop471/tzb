from dataclasses import dataclass

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from hand_eye_calib.board import BoardConfig, BoardDetection
from hand_eye_calib.geometry import invert_transform, make_transform, matrix_to_rotvec
from hand_eye_calib.offline_calibrate import main
from hand_eye_calib.session import CalibrationSession


@dataclass
class _Stamp:
    sec: int
    nanosec: int = 0


def _transform(rng, translation_scale):
    return make_transform(Rotation.random(random_state=rng).as_matrix(),
                          rng.uniform(-translation_scale, translation_scale, 3))


def _build_session(root, count=8, seed=3):
    rng = np.random.default_rng(seed)
    camera_from_ee = _transform(rng, 0.08)
    base_from_target = _transform(rng, 0.4)

    session = CalibrationSession(root=root, board=BoardConfig(),
                                 frames={'base': 'base_link', 'ee': 'tcp_link',
                                         'camera': 'camera_color_optical_frame'})
    for index in range(count):
        base_from_ee = _transform(rng, 0.3)
        cam_from_target = invert_transform(base_from_ee @ camera_from_ee) @ base_from_target
        detection = BoardDetection(
            rvec=matrix_to_rotvec(cam_from_target[:3, :3]),
            tvec=cam_from_target[:3, 3],
            corner_count=24,
            reproj_error_px=0.3,
        )
        session.add_sample(np.zeros((8, 8, 3), np.uint8), _Stamp(index),
                           base_from_ee, detection)
    return session, camera_from_ee


def test_offline_calibration_end_to_end(tmp_path):
    session, expected = _build_session(tmp_path)
    output = tmp_path / 'result' / 'hand_eye_result.yaml'

    status = main(['--session', str(session.directory), '--output', str(output)])

    assert status == 0
    assert output.is_file()
    with output.open() as stream:
        result = yaml.safe_load(stream)

    matrix = np.array(result['T_ee_camera']['matrix'])
    np.testing.assert_allclose(matrix, expected, atol=1e-4)
    assert result['samples']['used'] == list(range(8))
    assert result['samples']['rejected'] == []
    assert 'T_ee_camera_link' in result
    assert result['residuals']['rotation_max_deg'] < 1e-3
    assert result['residuals']['translation_max_mm'] < 1e-3
    assert result['frames']['ee'] == 'tcp_link'


def test_offline_calibration_can_remove_the_session(tmp_path):
    session, _ = _build_session(tmp_path, count=5)
    output = tmp_path / 'hand_eye_result.yaml'

    status = main(['--session', str(session.directory), '--output', str(output),
                   '--clean'])

    assert status == 0
    assert not session.directory.exists()


def test_offline_calibration_reports_missing_session(tmp_path, capsys):
    status = main(['--session', str(tmp_path / 'none'), '--output',
                   str(tmp_path / 'out.yaml')])

    assert status == 2
    assert 'no capture session found' in capsys.readouterr().err


def test_offline_calibration_reports_too_few_samples(tmp_path, capsys):
    session, _ = _build_session(tmp_path, count=2)
    status = main(['--session', str(session.directory), '--output',
                   str(tmp_path / 'out.yaml')])

    assert status == 2
    assert 'at least 3' in capsys.readouterr().err
