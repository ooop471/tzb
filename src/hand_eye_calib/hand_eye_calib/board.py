"""Calibration board definitions and pose detection.

Supports a ChArUco board (default) and a classic chessboard.  Both return the
board pose in the camera optical frame as a 4x4 matrix.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from hand_eye_calib.geometry import make_transform, rotvec_to_matrix

BOARD_TYPES = ('charuco', 'chessboard')

_SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)


@dataclass(frozen=True)
class BoardConfig:
    """Geometry of the physical board.

    ``squares_x``/``squares_y`` are the number of squares for a ChArUco board
    and the number of *inner corners* for a chessboard.
    """

    board_type: str = 'charuco'
    squares_x: int = 5
    squares_y: int = 7
    square_length: float = 0.03
    marker_length: float = 0.0225
    dictionary: str = 'DICT_5X5_100'
    min_corners: int = 6
    legacy_pattern: bool = False

    def __post_init__(self) -> None:
        board_type = self.board_type.strip().lower()
        object.__setattr__(self, 'board_type', board_type)
        if board_type not in BOARD_TYPES:
            raise ValueError(f'board_type must be one of {BOARD_TYPES}, got {self.board_type!r}')
        if self.squares_x < 2 or self.squares_y < 2:
            raise ValueError('squares_x and squares_y must be at least 2')
        if self.square_length <= 0.0:
            raise ValueError('square_length must be positive (metres)')
        if board_type == 'charuco' and self.marker_length <= 0.0:
            raise ValueError('marker_length must be positive (metres)')
        if board_type == 'charuco' and self.marker_length >= self.square_length:
            raise ValueError('marker_length must be smaller than square_length')

    @property
    def is_charuco(self) -> bool:
        return self.board_type == 'charuco'

    def to_dict(self) -> dict:
        return {
            'type': self.board_type,
            'squares_x': int(self.squares_x),
            'squares_y': int(self.squares_y),
            'square_length': float(self.square_length),
            'marker_length': float(self.marker_length),
            'dictionary': self.dictionary,
            'legacy_pattern': bool(self.legacy_pattern),
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'BoardConfig':
        return cls(
            board_type=data.get('type', 'charuco'),
            squares_x=int(data.get('squares_x', 5)),
            squares_y=int(data.get('squares_y', 7)),
            square_length=float(data.get('square_length', 0.03)),
            marker_length=float(data.get('marker_length', 0.0225)),
            dictionary=data.get('dictionary', 'DICT_5X5_100'),
            legacy_pattern=bool(data.get('legacy_pattern', False)),
        )


@dataclass
class BoardDetection:
    """Pose of the calibration board in the camera optical frame."""

    rvec: np.ndarray
    tvec: np.ndarray
    corner_count: int
    reproj_error_px: float
    image_points: np.ndarray = field(repr=False, default_factory=lambda: np.zeros((0, 2)))
    object_points: np.ndarray = field(repr=False, default_factory=lambda: np.zeros((0, 3)))

    @property
    def rotation(self) -> np.ndarray:
        return rotvec_to_matrix(self.rvec)

    @property
    def translation(self) -> np.ndarray:
        return self.tvec.reshape(3)

    @property
    def camera_from_board(self) -> np.ndarray:
        """``p_camera = T_camera_board @ p_board``."""
        return make_transform(self.rotation, self.translation)


class BoardDetector:
    """Detect the configured board and estimate its pose."""

    def __init__(self, config: BoardConfig):
        self.config = config
        self._charuco_board = None
        self._charuco_detector = None
        self._object_grid = None

        if config.is_charuco:
            dictionary_id = getattr(cv2.aruco, config.dictionary, None)
            if dictionary_id is None:
                raise ValueError(f'unknown ArUco dictionary {config.dictionary!r}')
            dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            self._charuco_board = cv2.aruco.CharucoBoard(
                (config.squares_x, config.squares_y),
                config.square_length,
                config.marker_length,
                dictionary,
            )
            if config.legacy_pattern and hasattr(self._charuco_board, 'setLegacyPattern'):
                self._charuco_board.setLegacyPattern(True)
            self._charuco_detector = cv2.aruco.CharucoDetector(self._charuco_board)
        else:
            self._object_grid = self._build_grid(config.squares_x, config.squares_y)
            self._pattern = (config.squares_x, config.squares_y)
            self._pattern_confirmed = False
            self.uses_swapped_pattern = False

    @staticmethod
    def _build_grid(columns: int, rows: int) -> np.ndarray:
        grid = np.zeros((columns * rows, 3), dtype=np.float32)
        grid[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
        return grid

    @property
    def active_pattern(self) -> tuple[int, int]:
        """Inner-corner layout currently used for a chessboard."""
        if self.config.is_charuco:
            return self.config.squares_x, self.config.squares_y
        return self._pattern

    def detect(self, image_bgr: np.ndarray, camera_matrix: np.ndarray,
               dist_coeffs: Optional[np.ndarray] = None) -> Optional[BoardDetection]:
        if image_bgr is None or image_bgr.size == 0:
            return None
        if image_bgr.ndim == 2:
            gray = image_bgr
        else:
            gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        if self.config.is_charuco:
            found = self._detect_charuco(gray)
        else:
            found = self._detect_chessboard(gray)
        if found is None:
            return None
        object_points, image_points = found
        return self._solve_pose(object_points, image_points, camera_matrix, dist_coeffs)

    # -- detection ---------------------------------------------------------
    def _detect_charuco(self, gray: np.ndarray):
        corners, ids, _, _ = self._charuco_detector.detectBoard(gray)
        if ids is None or corners is None or len(ids) < max(4, self.config.min_corners):
            return None
        object_points, image_points = self._charuco_board.matchImagePoints(corners, ids)
        if object_points is None or image_points is None:
            return None
        if len(object_points) < max(4, self.config.min_corners):
            return None
        return object_points.reshape(-1, 3).astype(np.float64), \
            image_points.reshape(-1, 2).astype(np.float64)

    def _detect_chessboard(self, gray: np.ndarray):
        patterns = [self._pattern]
        if not self._pattern_confirmed:
            swapped = (self._pattern[1], self._pattern[0])
            if swapped != self._pattern:
                patterns.append(swapped)

        for pattern in patterns:
            found, corners = self._find_chessboard_corners(gray, pattern)
            if not found or corners is None:
                continue
            if pattern != self._pattern:
                # A chessboard is symmetric, so columns/rows can be swapped by
                # mistake in the configuration; lock onto the detected layout.
                self._pattern = pattern
                self._object_grid = self._build_grid(*pattern)
                self.uses_swapped_pattern = True
            self._pattern_confirmed = True
            image_points = corners.reshape(-1, 2).astype(np.float64)
            if len(image_points) != len(self._object_grid):
                return None
            return (self._object_grid * np.float32(self.config.square_length)
                    ).astype(np.float64), image_points
        return None

    @staticmethod
    def _find_chessboard_corners(gray: np.ndarray, pattern):
        flags = (cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
                 | cv2.CALIB_CB_ACCURACY)
        found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=flags)
        if not found:
            found, corners = cv2.findChessboardCorners(
                gray, pattern,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                           _SUBPIX_CRITERIA)
        return found, corners

    # -- pose --------------------------------------------------------------
    @staticmethod
    def _solve_pose(object_points: np.ndarray, image_points: np.ndarray,
                    camera_matrix: np.ndarray,
                    dist_coeffs: Optional[np.ndarray]) -> Optional[BoardDetection]:
        camera_matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        distortion = (np.zeros(5, dtype=np.float64) if dist_coeffs is None
                      else np.asarray(dist_coeffs, dtype=np.float64).reshape(-1))

        flags = cv2.SOLVEPNP_IPPE if len(object_points) >= 4 else cv2.SOLVEPNP_ITERATIVE
        ok, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix,
                                      distortion, flags=flags)
        if not ok:
            ok, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix,
                                          distortion, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                return None
        try:
            rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points,
                                              camera_matrix, distortion, rvec, tvec)
        except cv2.error:
            pass

        projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix,
                                         distortion)
        error = float(np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1).mean())

        return BoardDetection(
            rvec=rvec.reshape(3),
            tvec=tvec.reshape(3),
            corner_count=int(len(image_points)),
            reproj_error_px=error,
            image_points=image_points,
            object_points=object_points,
        )

    def draw(self, image_bgr: np.ndarray, detection: BoardDetection,
             camera_matrix: np.ndarray, dist_coeffs: Optional[np.ndarray],
             axis_length: float = 0.05) -> np.ndarray:
        """Draw detected corners and the board axes onto a copy of the image."""
        canvas = image_bgr.copy()
        if canvas.ndim == 2:
            canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
        if self.config.is_charuco:
            corners = detection.image_points.reshape(-1, 1, 2).astype(np.float32)
            cv2.aruco.drawDetectedCornersCharuco(canvas, corners, None, (0, 255, 0))
        else:
            corners = detection.image_points.reshape(-1, 1, 2).astype(np.float32)
            cv2.drawChessboardCorners(canvas, self._pattern, corners, True)
        distortion = (np.zeros(5, dtype=np.float64) if dist_coeffs is None
                      else np.asarray(dist_coeffs, dtype=np.float64).reshape(-1))
        cv2.drawFrameAxes(canvas, np.asarray(camera_matrix, dtype=np.float64), distortion,
                          detection.rvec.reshape(3, 1), detection.tvec.reshape(3, 1),
                          axis_length)
        return canvas
