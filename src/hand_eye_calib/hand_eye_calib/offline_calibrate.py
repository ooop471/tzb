"""Compute the eye-in-hand calibration from the latest capture session.

Typical use after the capture node stored samples::

    ros2 run hand_eye_calib offline_calibrate

It reads the newest session automatically (nothing needs to be passed on the
command line), prints the 4x4 transform and writes it to ``hand_eye_result.yaml``.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import yaml

if __package__ in (None, ''):  # allow running this file directly from a source tree
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hand_eye_calib.geometry import (format_matrix, matrix_to_quat, matrix_to_rpy,
                                     optical_to_link, transform_to_dict)
from hand_eye_calib.session import CalibrationSession, SessionData
from hand_eye_calib.solver import (NotEnoughSamplesError, rotation_diversity_deg,
                                   solve_hand_eye)

DEFAULT_OUTPUT = 'hand_eye_result.yaml'
LINE = '=' * 68
THIN = '-' * 68


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Solve the 4x4 camera pose from the captured hand-eye samples.')
    parser.add_argument('--session', default=None,
                        help='capture directory or its root (default: newest session)')
    parser.add_argument('--output', default=DEFAULT_OUTPUT,
                        help=f'result YAML path (default: ./{DEFAULT_OUTPUT})')
    parser.add_argument('--method', default='AUTO',
                        help='AUTO, TSAI, PARK, HORAUD, ANDREFF or DANIILIDIS')
    parser.add_argument('--no-refine', action='store_true',
                        help='skip the nonlinear least-squares refinement')
    parser.add_argument('--min-samples', type=int, default=4,
                        help='never reject below this many samples (default: 4)')
    parser.add_argument('--max-rot-deg', type=float, default=2.0,
                        help='outlier threshold on rotation residual (default: 2.0 deg)')
    parser.add_argument('--max-trans-mm', type=float, default=10.0,
                        help='outlier threshold on translation residual (default: 10 mm)')
    parser.add_argument('--clean', action='store_true',
                        help='delete the capture session after a successful solve')
    return parser


def _print_transform(title: str, description: str, transform: np.ndarray) -> None:
    rotation = transform[:3, :3]
    quaternion = matrix_to_quat(rotation)
    rpy = np.degrees(matrix_to_rpy(rotation))
    print(title)
    print(f'  {description}')
    print(format_matrix(transform))
    print('  xyz [m]   : '
          + '  '.join(f'{value: .6f}' for value in transform[:3, 3]))
    print('  quat xyzw : '
          + '  '.join(f'{value: .6f}' for value in quaternion))
    print('  rpy [deg] : '
          + '  '.join(f'{value: .3f}' for value in rpy))
    print()


def _payload(data: SessionData, result) -> dict:
    return {
        'version': 1,
        'generated': datetime.now().astimezone().isoformat(timespec='seconds'),
        'method': result.method,
        'refined': result.refined,
        'frames': dict(data.frames),
        'board': dict(data.board),
        'samples': {
            'total': result.total_samples,
            'used': result.used,
            'rejected': result.rejected,
        },
        'T_ee_camera': {
            'description': 'p_ee = T_ee_camera @ p_camera_optical',
            **transform_to_dict(result.T_ee_camera),
        },
        'T_ee_camera_link': {
            'description': 'p_ee = T_ee_camera_link @ p_camera_link (ROS body axes)',
            **transform_to_dict(optical_to_link(result.T_ee_camera)),
        },
        'T_base_target': {
            'description': 'mean board pose in the base frame (should be constant)',
            **transform_to_dict(result.T_base_target),
        },
        'T_base_camera': {
            'description': 'camera pose in the base frame for the first used sample',
            **transform_to_dict(result.T_base_camera),
        },
        'residuals': {
            'rotation_rms_deg': result.rotation_rms_deg,
            'rotation_max_deg': result.rotation_max_deg,
            'translation_rms_mm': result.translation_rms_mm,
            'translation_max_mm': result.translation_max_mm,
            'per_sample': result.per_sample,
        },
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        data = CalibrationSession.load(args.session)
    except FileNotFoundError as exc:
        print(f'error: {exc}', file=sys.stderr)
        print('hint: start the capture node, press SPACE for at least 4 poses, '
              'then run this script again', file=sys.stderr)
        return 2

    if len(data.samples) < 3:
        print(f'error: only {len(data.samples)} sample(s) captured; at least 3 are '
              'needed (5 or more recommended)', file=sys.stderr)
        return 2

    board = data.board
    if board:
        geometry_text = (f'{board.get("squares_x")}x{board.get("squares_y")} '
                         + ('squares' if board.get('type') == 'charuco'
                            else 'inner corners'))
        print(f'board: {board.get("type")} {geometry_text}, '
              f'square={float(board.get("square_length", 0.0)) * 1000.0:.1f} mm'
              + (f', marker={float(board.get("marker_length", 0.0)) * 1000.0:.1f} mm'
                 if board.get('type') == 'charuco' else ''))

    base_ee = data.base_ee_matrices()
    cam_target = data.cam_target_matrices()
    diversity = rotation_diversity_deg(base_ee)

    try:
        result = solve_hand_eye(
            base_ee, cam_target,
            method=args.method,
            refine=not args.no_refine,
            min_samples=args.min_samples,
            max_rot_deg=args.max_rot_deg,
            max_trans_mm=args.max_trans_mm,
            labels=data.labels(),
        )
    except NotEnoughSamplesError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    except (RuntimeError, ValueError) as exc:
        print(f'error: calibration failed: {exc}', file=sys.stderr)
        return 1

    frame_text = ', '.join(f'{key}={value}' for key, value in data.frames.items()
                           if value)
    print(LINE)
    print('eye-in-hand calibration result')
    print(LINE)
    print(f'session   : {data.directory}')
    print(f'samples   : {result.total_samples} captured, {len(result.used)} used, '
          f'{len(result.rejected)} rejected')
    print(f'frames    : {frame_text or "unknown"}')
    print(f'method    : {result.method}'
          + (' (refined)' if result.refined else ' (closed form)'))
    print(f'diversity : {diversity:.1f} deg between end-effector orientations')
    print(THIN)
    _print_transform('T_ee_camera   (4x4)', 'p_ee = T_ee_camera @ p_camera_optical',
                     result.T_ee_camera)
    _print_transform('T_ee_camera_link   (4x4)',
                     'p_ee = T_ee_camera_link @ p_camera_link (ROS body axes)',
                     optical_to_link(result.T_ee_camera))
    print('residuals : rotation rms {:.3f} deg / max {:.3f} deg, '
          'translation rms {:.3f} mm / max {:.3f} mm'.format(
              result.rotation_rms_deg, result.rotation_max_deg,
              result.translation_rms_mm, result.translation_max_mm))
    for sample in result.per_sample:
        print('  sample {:>3s}: rotation {:6.3f} deg  translation {:6.3f} mm'.format(
            sample['label'], sample['rotation_deg'], sample['translation_mm']))
    if result.rejected:
        print('rejected  : samples ' + ', '.join(
            str(data.labels()[index]) for index in result.rejected))
    print(LINE)

    if diversity < 15.0:
        print('warning: the end-effector orientations barely change; '
              'move the wrist to different tilts for a reliable result')
    if len(result.used) < 5:
        print('warning: fewer than 5 samples were used; accuracy will be limited')
    if result.translation_max_mm > 5.0 or result.rotation_max_deg > 1.0:
        print('warning: residuals are large; check board dimensions, image blur '
              'and that the arm was standing still while capturing')

    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(_payload(data, result), stream, sort_keys=False,
                       default_flow_style=False)
    print(f'written to {output}')

    if args.clean:
        shutil.rmtree(data.directory, ignore_errors=True)
        print(f'removed capture session {data.directory}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
