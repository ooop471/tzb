from dataclasses import dataclass

import numpy as np
import pytest

from hand_eye_calib.board import BoardConfig, BoardDetection
from hand_eye_calib.session import CalibrationSession


@dataclass
class _Stamp:
    sec: int
    nanosec: int = 0


def _detection(offset: float = 0.0) -> BoardDetection:
    return BoardDetection(
        rvec=np.array([0.0, 0.0, offset]),
        tvec=np.array([0.1 * offset, 0.0, 0.5]),
        corner_count=20,
        reproj_error_px=0.25,
    )


def _pose(index: int) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = [0.01 * index, 0.0, 0.2]
    return transform


def test_session_roundtrip_and_fresh_start(tmp_path):
    board = BoardConfig()
    session = CalibrationSession(root=tmp_path, board=board,
                                 frames={'base': 'base_link', 'ee': 'tcp_link'})
    session.set_camera_info({'k': [800.0, 0.0, 320.0, 0.0, 800.0, 240.0, 0.0, 0.0, 1.0],
                             'd': [0.0] * 5})
    for index in range(3):
        session.add_sample(np.zeros((8, 8, 3), np.uint8), _Stamp(index), _pose(index),
                           _detection(0.01 * index))

    assert len(session) == 3
    assert (session.directory / 'images' / '000.png').is_file()

    data = CalibrationSession.load(str(tmp_path))
    assert data.count == 3
    assert data.frames['base'] == 'base_link'
    assert data.board['type'] == 'charuco'
    np.testing.assert_allclose(data.base_ee_matrices()[2], _pose(2))
    np.testing.assert_allclose(data.cam_target_matrices()[1],
                               _detection(0.01).camera_from_board)

    # A new run must not reuse anything from the previous calibration.
    fresh = CalibrationSession(root=tmp_path, board=board)
    assert len(fresh) == 0
    assert not (fresh.directory / 'images' / '000.png').exists()
    reloaded = CalibrationSession.load(str(tmp_path))
    assert reloaded.count == 0


def test_empty_session_is_removed(tmp_path):
    session = CalibrationSession(root=tmp_path, board=BoardConfig())
    session.discard_if_empty()
    assert not session.directory.exists()
    with pytest.raises(FileNotFoundError):
        CalibrationSession.load(str(tmp_path))
