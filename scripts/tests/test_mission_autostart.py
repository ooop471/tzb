#!/usr/bin/env python3
"""Offline checks: never launch ROS, open serial ports or command the rover."""
import fcntl
import ast
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[2]


class AutostartTests(unittest.TestCase):
    def test_direct_arm_process_receives_graceful_stop(self):
        # Exercise the supervisor with a harmless stand-in driver, never hardware.
        source = (ROOT / 'scripts/start_mission.bash').read_text().split("<<'MISSION_PYTHON'\n", 1)[1]
        tree = ast.parse(source.rsplit('\nMISSION_PYTHON', 1)[0])
        definitions = [node for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))
                       or isinstance(node, ast.ClassDef) and node.name == 'ProcessManager']
        namespace = {}
        exec(compile(ast.Module(body=definitions, type_ignores=[]), '<supervisor>', 'exec'), namespace)
        manager = namespace['ProcessManager']()
        with tempfile.TemporaryDirectory() as directory:
            ready = Path(directory) / 'ready'
            stopped = Path(directory) / 'stopped'
            code = ('import signal,sys,time; from pathlib import Path; '
                    'signal.signal(signal.SIGINT, lambda sig, frame: '
                    '(Path(sys.argv[2]).write_text(str(sig)), sys.exit(0))); '
                    'Path(sys.argv[1]).touch(); time.sleep(30)')
            manager.spawn('arm_serial', ['python3', '-c', code, str(ready), str(stopped)])
            try:
                deadline = time.monotonic() + 3
                while not ready.exists() and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertTrue(ready.exists())
                manager.stop('arm_serial', 2)
                self.assertEqual(stopped.read_text(), '2')
                self.assertFalse(manager.members('arm_serial'))
            finally:
                manager.cleanup(2)

    def check_params(self, paths):
        return subprocess.run([
            'bash', '-c', 'source /opt/ros/humble/setup.bash && exec python3 "$@"',
            'check-params', str(ROOT / 'scripts/check_mission_params.py'), *map(str, paths),
        ], capture_output=True, text=True, timeout=10)

    def test_mixed_numeric_array_rejected_by_ros_parser(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mission.yaml'
            path.write_text('mission_state:\n  ros__parameters:\n    entry_navigation.goal: [1.0, 0, 0]\n')
            result = self.check_params([path])
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Sequence should be of same type', result.stderr)
            self.assertIn(str(path), result.stderr)

    def test_double_array_accepted_by_ros_parser(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'mission.yaml'
            path.write_text('mission_state:\n  ros__parameters:\n    entry_navigation.goal: [1.0, 0.0, 0.0]\n')
            result = self.check_params([path])
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_source_and_installed_mission_params_parse(self):
        result = self.check_params([
            ROOT / 'src/state/config/mission.yaml',
            ROOT / 'install/rover_state/share/rover_state/config/mission.yaml',
        ])
        self.assertEqual(result.returncode, 0, result.stderr)

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
