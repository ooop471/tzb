#!/usr/bin/env python3
"""Wait for configured serial devices without opening them or starting drivers."""
import argparse
import os
from pathlib import Path
import stat
import sys
import time
import yaml


def configured_devices(nav_root):
    arm = Path(os.environ.get('ARM_WS', nav_root.parent / 'tzb_ws'))
    chassis = Path(os.environ.get('CHASSIS_WS', nav_root.parent / 'serial_t'))
    # Match the two parameter files used by start_mission and chassis serial.launch.py.
    configs = [arm / 'install/rm_serial_driver/share/rm_serial_driver/config/serial_params.yaml',
               chassis / 'src/serial/driver/config/serial_params.yaml']
    devices = []
    for config in configs:
        data = yaml.safe_load(config.read_text())
        names = {node['ros__parameters']['device_name'] for node in data.values()
                 if isinstance(node, dict) and isinstance(node.get('ros__parameters'), dict)
                 and 'device_name' in node['ros__parameters']}
        if len(names) != 1 or not isinstance(next(iter(names)), str) or not next(iter(names)):
            raise ValueError(f'Expected one nonempty serial device_name in {config}')
        path = Path(next(iter(names))).expanduser()
        if not path.is_absolute():
            raise ValueError(f'Serial device must have an absolute path: {config}')
        devices.append(path)
    if devices[0].resolve() == devices[1].resolve():
        raise ValueError('Chassis and arm must use different serial devices')
    return devices


def device_problem(path):
    try:
        if not stat.S_ISCHR(path.stat().st_mode):
            return f'{path}: not a character device'
    except OSError:
        return f'{path}: not present'
    if not os.access(path, os.R_OK | os.W_OK):
        return f'{path}: user lacks read/write permission'
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nav-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--timeout', type=float, default=90.)
    args = parser.parse_args()
    if not 0 < args.timeout <= 600:
        parser.error('timeout must be in (0, 600] seconds')
    devices = configured_devices(args.nav_root.resolve())
    deadline = time.monotonic() + args.timeout
    previous = None
    while True:
        problems = [problem for path in devices if (problem := device_problem(path))]
        if not problems:
            print('Serial devices ready: ' + ', '.join(map(str, devices)), flush=True)
            return 0
        if problems != previous:
            print('Waiting for serial devices: ' + '; '.join(problems), flush=True)
            previous = problems
        if time.monotonic() >= deadline:
            print('Serial device readiness timed out; mission was not started.', file=sys.stderr)
            return 1
        time.sleep(min(.5, max(0., deadline - time.monotonic())))


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f'Device readiness error: {exc}', file=sys.stderr)
        raise SystemExit(1)
