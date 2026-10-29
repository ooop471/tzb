"""One calibration run's scratch data.

A new node start always replaces the previous capture directory, so samples
from an earlier calibration can never leak into the next one.  The data lives
under the system temp directory by default and is meant to be consumed by
``offline_calibrate`` right after capture.
"""

from __future__ import annotations

import os
import shutil
import tempfile
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml

from hand_eye_calib.board import BoardConfig, BoardDetection
from hand_eye_calib.geometry import transform_to_dict

SESSION_DIR_NAME = 'current'
METADATA_FILE = 'samples.yaml'
IMAGES_DIR = 'images'


def default_session_root() -> Path:
    """Directory that holds the active capture session."""
    override = os.environ.get('HAND_EYE_SESSION_ROOT', '').strip()
    if override:
        return Path(override).expanduser()
    return Path(tempfile.gettempdir()) / 'hand_eye_calib'


def resolve_session_dir(path: Optional[str] = None) -> Path:
    """Accept a root, a session directory, or nothing (latest session)."""
    if path:
        candidate = Path(path).expanduser()
    else:
        candidate = default_session_root() / SESSION_DIR_NAME
    if (candidate / METADATA_FILE).is_file():
        return candidate
    if (candidate / SESSION_DIR_NAME / METADATA_FILE).is_file():
        return candidate / SESSION_DIR_NAME
    return candidate


@dataclass
class SessionData:
    directory: Path
    created: str = ''
    frames: dict = field(default_factory=dict)
    board: dict = field(default_factory=dict)
    camera_info: dict = field(default_factory=dict)
    samples: list = field(default_factory=list)

    @property
    def count(self) -> int:
        return len(self.samples)

    def base_ee_matrices(self) -> list[np.ndarray]:
        return [_matrix(sample['base_ee']) for sample in self.samples]

    def cam_target_matrices(self) -> list[np.ndarray]:
        return [_matrix(sample['cam_target']) for sample in self.samples]

    def labels(self) -> list[str]:
        return [str(sample.get('index', i)) for i, sample in enumerate(self.samples)]


def _matrix(entry: dict) -> np.ndarray:
    return np.asarray(entry['matrix'], dtype=float).reshape(4, 4)


class CalibrationSession:
    """Writes one capture session to disk, replacing whatever was there before."""

    def __init__(self, root: Optional[str] = None, board: Optional[BoardConfig] = None,
                 frames: Optional[dict] = None, wipe: bool = True):
        self.root = Path(root).expanduser() if root else default_session_root()
        self.directory = self.root / SESSION_DIR_NAME
        if wipe:
            # Never reuse earlier measurements: a recalibration (for example
            # after moving the camera) starts from an empty set of samples.
            shutil.rmtree(self.directory, ignore_errors=True)
        (self.directory / IMAGES_DIR).mkdir(parents=True, exist_ok=True)

        self.created = datetime.now().astimezone().isoformat(timespec='seconds')
        self.board = board
        self.frames = dict(frames or {})
        self.camera_info: dict = {}
        self._samples: list[dict] = []
        self.flush()

    @property
    def samples(self) -> list[dict]:
        return list(self._samples)

    def __len__(self) -> int:
        return len(self._samples)

    def set_camera_info(self, camera_info: dict) -> None:
        self.camera_info = camera_info
        self.flush()

    def add_sample(self, image_bgr: np.ndarray, stamp, base_ee: np.ndarray,
                   detection: BoardDetection,
                   base_camera: Optional[np.ndarray] = None) -> int:
        index = len(self._samples)
        relative_image = f'{IMAGES_DIR}/{index:03d}.png'
        cv2.imwrite(str(self.directory / relative_image), image_bgr)

        sample = {
            'index': index,
            'image': relative_image,
            'stamp': {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)},
            'base_ee': transform_to_dict(base_ee),
            'cam_target': {
                **transform_to_dict(detection.camera_from_board),
                'rvec': [float(v) for v in detection.rvec],
                'tvec': [float(v) for v in detection.tvec],
                'corner_count': int(detection.corner_count),
                'reproj_px': float(detection.reproj_error_px),
            },
            'base_camera': (transform_to_dict(base_camera)
                            if base_camera is not None else None),
        }
        self._samples.append(sample)
        self.flush()
        return index

    def last_base_ee(self) -> Optional[np.ndarray]:
        if not self._samples:
            return None
        return _matrix(self._samples[-1]['base_ee'])

    def as_dict(self) -> dict:
        return {
            'version': 1,
            'created': self.created,
            'frames': dict(self.frames),
            'board': self.board.to_dict() if self.board is not None else {},
            'camera_info': dict(self.camera_info),
            'samples': self._samples,
        }

    def flush(self) -> None:
        target = self.directory / METADATA_FILE
        temporary = target.with_suffix('.yaml.tmp')
        with temporary.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(self.as_dict(), stream, sort_keys=False,
                           default_flow_style=False)
        temporary.replace(target)

    def finalize(self) -> None:
        self.flush()

    def discard_if_empty(self) -> None:
        if not self._samples:
            shutil.rmtree(self.directory, ignore_errors=True)

    def clean(self) -> None:
        shutil.rmtree(self.directory, ignore_errors=True)

    @staticmethod
    def load(path: Optional[str] = None) -> SessionData:
        directory = resolve_session_dir(path)
        metadata = directory / METADATA_FILE
        if not metadata.is_file():
            raise FileNotFoundError(
                f'no capture session found at {directory} '
                f'(expected {METADATA_FILE}); run the capture node first')
        with metadata.open('r', encoding='utf-8') as stream:
            raw = yaml.safe_load(stream) or {}
        return SessionData(
            directory=directory,
            created=str(raw.get('created', '')),
            frames=dict(raw.get('frames') or {}),
            board=dict(raw.get('board') or {}),
            camera_info=dict(raw.get('camera_info') or {}),
            samples=list(raw.get('samples') or []),
        )
