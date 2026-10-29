import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from hand_eye_calib.board import BoardConfig, BoardDetector

CAMERA_MATRIX = np.array([[800.0, 0.0, 600.0],
                          [0.0, 800.0, 450.0],
                          [0.0, 0.0, 1.0]])


def test_charuco_board_detected_in_generated_image():
    config = BoardConfig()
    detector = BoardDetector(config)
    image = detector._charuco_board.generateImage((700, 500), marginSize=20,
                                                  borderBits=1)
    intrinsics = np.array([[800.0, 0.0, 350.0],
                           [0.0, 800.0, 250.0],
                           [0.0, 0.0, 1.0]])

    detection = detector.detect(image, intrinsics)

    assert detection is not None
    assert detection.corner_count == 24
    assert detection.reproj_error_px < 1.0
    assert detection.tvec[2] > 0.0
    # The generated board is fronto-parallel to the image plane.
    angle = np.degrees(Rotation.from_matrix(detection.rotation).magnitude())
    assert angle < 2.0
    drawn = detector.draw(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR), detection,
                          intrinsics, None)
    assert drawn.shape[2] == 3


def _render_chessboard(config, camera_matrix, transform, size=(900, 1200)):
    height, width = size
    image = np.full((height, width), 255, np.uint8)
    square = config.square_length
    rotation = transform[:3, :3]
    translation = transform[:3, 3]

    def project(point):
        point_camera = rotation @ point + translation
        pixel, _ = cv2.projectPoints(point_camera.reshape(1, 3), np.zeros(3),
                                     np.zeros(3), camera_matrix, None)
        return pixel.reshape(2)

    for column in range(-1, config.squares_x):
        for row in range(-1, config.squares_y):
            if (column + row) % 2:
                continue
            corners = np.array([
                [column * square, row * square, 0.0],
                [(column + 1) * square, row * square, 0.0],
                [(column + 1) * square, (row + 1) * square, 0.0],
                [column * square, (row + 1) * square, 0.0],
            ])
            polygon = np.array([project(point) for point in corners], np.int32)
            cv2.fillConvexPoly(image, polygon, 0)
    return image


def test_chessboard_board_pose_matches_rendered_geometry():
    config = BoardConfig(board_type='chessboard', squares_x=9, squares_y=6,
                         square_length=0.025)
    detector = BoardDetector(config)
    truth = np.eye(4)
    truth[:3, :3] = Rotation.from_euler('xyz', [0.4, -0.25, 0.15]).as_matrix()
    truth[:3, 3] = [0.02, -0.01, 0.45]

    detection = detector.detect(_render_chessboard(config, CAMERA_MATRIX, truth),
                                CAMERA_MATRIX)

    assert detection is not None
    assert detection.corner_count == config.squares_x * config.squares_y
    assert detection.reproj_error_px < 1.0

    # A plain chessboard is ambiguous under a 180 deg rotation around its
    # normal, so compare the board centre (invariant to that flip) and normal.
    centre = np.array([(config.squares_x - 1) / 2.0,
                       (config.squares_y - 1) / 2.0, 0.0]) * config.square_length
    expected_centre = truth[:3, :3] @ centre + truth[:3, 3]
    measured_centre = detection.rotation @ centre + detection.translation
    np.testing.assert_allclose(measured_centre, expected_centre, atol=2e-3)

    normal_dot = abs(float(detection.rotation[:, 2] @ truth[:3, 2]))
    assert normal_dot == pytest.approx(1.0, abs=1e-3)


def test_invalid_board_configuration_is_rejected():
    with pytest.raises(ValueError):
        BoardConfig(board_type='aprilgrid')
    with pytest.raises(ValueError):
        BoardConfig(marker_length=0.05)  # larger than the square


def test_chessboard_falls_back_to_transposed_pattern(monkeypatch):
    detector = BoardDetector(BoardConfig(board_type='chessboard', squares_x=8,
                                         squares_y=11, square_length=0.03))
    fake_corners = np.zeros((11 * 8, 1, 2), np.float32)

    def fake_find(gray, pattern):
        return pattern == (11, 8), fake_corners

    monkeypatch.setattr(detector, '_find_chessboard_corners', fake_find)
    found = detector._detect_chessboard(np.zeros((100, 100), np.uint8))

    assert found is not None
    assert detector.uses_swapped_pattern
    assert detector.active_pattern == (11, 8)
    assert found[0].shape == (88, 3)
