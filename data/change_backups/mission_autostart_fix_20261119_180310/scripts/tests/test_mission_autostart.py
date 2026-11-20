#!/usr/bin/env python3
"""Offline checks: never launch ROS, open serial ports or command the rover."""
import fcntl
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class AutostartTests(unittest.TestCase):
    def wait_devices(self, arm_device, chassis_device):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            arm = root / 'arm'
            chassis = root / 'chassis'
            configs = [arm / 'install/rm_serial_driver/share/rm_serial_driver/config/serial_params.yaml',
                       chassis / 'src/serial/driver/config/serial_params.yaml']
            for config, device in zip(configs, [arm_device, chassis_device]):
                config.parent.mkdir(parents=True)
                config.write_text(f'driver:\n  ros__parameters:\n    device_name: {device}\n')
            env = dict(os.environ, ARM_WS=str(arm), CHASSIS_WS=str(chassis))
            return subprocess.run(['python3', str(ROOT / 'scripts/wait_mission_devices.py'),
                                   '--timeout', '.03'], env=env, capture_output=True, text=True, timeout=5)

    def test_distinct_character_devices_ready(self):
        result = self.wait_devices('/dev/null', '/dev/zero')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('Serial devices ready', result.stdout)

    def test_missing_device_times_out(self):
        result = self.wait_devices('/dev/rover_nonexistent_test_device', '/dev/zero')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('mission was not started', result.stderr)

    def test_regular_file_rejected(self):
        result = self.wait_devices(str(Path(__file__).resolve()), '/dev/zero')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('not a character device', result.stdout)

    def test_same_device_rejected(self):
        result = self.wait_devices('/dev/null', '/dev/null')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('different serial devices', result.stderr)

    def test_duplicate_launch_blocked_before_preflight(self):
        path = ROOT / 'log/mission/.mission.lock'
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('a+') as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = subprocess.run(['bash', str(ROOT / 'scripts/start_mission.bash')],
                                    capture_output=True, text=True, timeout=5)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Another mission is already running', result.stderr)
            # Read-only command previews remain usable while a mission runs.
            preview = subprocess.run(['bash', str(ROOT / 'scripts/start_mission.bash'), '--dry-run'],
                                     capture_output=True, text=True, timeout=5)
            self.assertEqual(preview.returncode, 0, preview.stderr)


if __name__ == '__main__':
    unittest.main()
