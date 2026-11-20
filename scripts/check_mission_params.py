#!/usr/bin/env python3
"""Parse installed ROS parameters without creating nodes or starting drivers."""
import sys
from pathlib import Path


def main():
    from ament_index_python.packages import get_package_share_directory
    from rclpy.utilities import remove_ros_args

    paths = [Path(arg) for arg in sys.argv[1:]]
    if not paths:
        paths = [Path(get_package_share_directory(package)) / relative for package, relative in (
            ('rover_state', 'config/mission.yaml'),
            ('rover_explore', 'config/rover.yaml'),
            ('fast_lio', 'config/mid360.yaml'),
            ('bringup', 'config/nav2_params.yaml'),
        )]
    for path in paths:
        if not path.is_file():
            raise RuntimeError(f'Missing parameter file: {path}')
        try:
            # This invokes rcl's C YAML parser, unlike yaml.safe_load, which
            # accepts mixed integer/double sequences that ROS 2 rejects.
            remove_ros_args(['mission-preflight', '--ros-args', '--params-file', str(path)])
        except Exception as exc:
            raise RuntimeError(f'Invalid ROS parameter file {path}: {exc}') from exc
        print(f'ROS parameters valid: {path}', flush=True)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(1)
