#!/usr/bin/env bash
set -eo pipefail
export NAV_MISSION_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec python3 - "$@" <<'MISSION_PYTHON'
#!/usr/bin/env python3
"""Sensors -> data validation/retry -> serial feedback -> algorithms."""
import argparse
import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime

ROOT = Path(os.environ['NAV_MISSION_ROOT'])
ENV_SCRIPT = '#!/usr/bin/env bash\n# Keep the two different rm_serial_driver packages in separate environments.\nset -eo pipefail\nnav_root="$1"; shift\narm_root="${ARM_WS:-$(dirname "$nav_root")/tzb_ws}"\nchassis_root="${CHASSIS_WS:-$(dirname "$nav_root")/serial_t}"\nscope="${1:?environment scope required}"; shift\nunset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH\nsource /opt/ros/humble/setup.bash\ncase "$scope" in\n  arm|camera)\n    cd "$arm_root"\n    if [[ -r scripts/local_deps.bash ]]; then source scripts/local_deps.bash; fi\n    source install/setup.bash ;;\n  nav)\n    if [[ -r "$arm_root/scripts/local_deps.bash" ]]; then source "$arm_root/scripts/local_deps.bash"; fi\n    source "$arm_root/install/setup.bash"\n    source "$nav_root/.deps/livox-sdk2/setup.bash"\n    source "$nav_root/install/setup.bash" ;;\n  chassis) source "$chassis_root/install/setup.bash" ;;\n  *) printf \'Unknown environment: %s\\n\' "$scope" >&2; exit 2 ;;\nesac\n# Use ROS\'s default domain; never assign a custom domain ID.\nunset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS\ncd "$nav_root"\nexec "$@"\n'
SENSOR_PROBE = '#!/usr/bin/env python3\n"""Require a new message on every specified topic, rather than discovery alone."""\nimport argparse\nimport time\nimport rclpy\nfrom rclpy.node import Node\nfrom rclpy.qos import qos_profile_sensor_data\nfrom rosidl_runtime_py.utilities import get_message\n\n\ndef main():\n    parser = argparse.ArgumentParser()\n    parser.add_argument(\'--timeout\', type=float, default=30)\n    parser.add_argument(\'topics\', nargs=\'+\')\n    args = parser.parse_args()\n    rclpy.init()\n    node = Node(\'mission_sensor_readiness\')\n    subscriptions = {}\n    seen = set()\n    deadline = time.monotonic() + args.timeout\n    try:\n        while time.monotonic() < deadline and seen != set(args.topics):\n            available = dict(node.get_topic_names_and_types())\n            for topic in args.topics:\n                if topic not in subscriptions and topic in available:\n                    try:\n                        cls = get_message(available[topic][0])\n                        subscriptions[topic] = node.create_subscription(\n                            cls, topic, lambda msg, name=topic: seen.add(name), qos_profile_sensor_data)\n                    except (ImportError, AttributeError, ModuleNotFoundError, ValueError) as exc:\n                        node.get_logger().warning(f\'{topic}: {exc}\')\n            rclpy.spin_once(node, timeout_sec=0.2)\n        missing = sorted(set(args.topics) - seen)\n        print(\'Missing data: \' + \', \'.join(missing) if missing else \'All sensor topics received fresh data\', flush=True)\n        return 1 if missing else 0\n    finally:\n        node.destroy_node()\n        rclpy.shutdown()\n\n\nif __name__ == \'__main__\':\n    raise SystemExit(main())\n'

ARM = Path(os.environ.get('ARM_WS', ROOT.parent / 'tzb_ws'))
CHASSIS = Path(os.environ.get('CHASSIS_WS', ROOT.parent / 'serial_t'))


def command(scope, *args):
    return ['bash', '-c', ENV_SCRIPT, 'mission-env', str(ROOT), scope, *map(str, args)]




class ProcessManager:
    """Track this run's processes, including children that create new sessions."""
    def __init__(self):
        import ctypes
        import uuid
        self.token = uuid.uuid4().hex
        self.processes = {}
        # Adopt orphaned descendants so they can be reaped after launch exits.
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(36, 1, 0, 0, 0) != 0:  # PR_SET_CHILD_SUBREAPER, Linux
            raise OSError(ctypes.get_errno(), 'Cannot enable child subreaper')

    def members(self, name):
        marker = f'NAV_MISSION_OWNER={self.token}:{name}'.encode()
        result = {}
        for directory in Path('/proc').iterdir():
            if not directory.name.isdigit():
                continue
            try:
                stat = (directory / 'stat').read_text().rsplit(')', 1)[1].split()
                if stat[0] == 'Z':
                    continue
                if marker in (directory / 'environ').read_bytes().split(b'\0'):
                    result[int(directory.name)] = stat[19]  # process start time
            except (OSError, IndexError, ValueError):
                continue
        return result

    def send(self, members, sig):
        for pid, born in members.items():
            try:
                stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
                if stat[19] == born and stat[0] != 'Z':
                    os.kill(pid, sig)
            except (OSError, IndexError):
                pass

    def spawn(self, name, cmd, output=None):
        # A Ctrl+C cannot arrive between Popen and recording its PID.
        blocked = {signal.SIGINT, signal.SIGTERM, signal.SIGHUP}
        previous = signal.pthread_sigmask(signal.SIG_BLOCK, blocked)
        try:
            env = os.environ.copy()
            env['NAV_MISSION_OWNER'] = f'{self.token}:{name}'
            # Reset the inherited mask before exec without a threaded preexec_fn.
            wrapper = ('import os,signal,sys; signal.pthread_sigmask(signal.SIG_SETMASK, []); '
                       'os.execvpe(sys.argv[1],sys.argv[1:],os.environ)')
            proc = subprocess.Popen([sys.executable, '-c', wrapper, *cmd], env=env,
                                    stdin=subprocess.DEVNULL, stdout=output,
                                    stderr=subprocess.STDOUT if output else None,
                                    start_new_session=True)
            self.processes[name] = proc
            return proc
        finally:
            signal.pthread_sigmask(signal.SIG_SETMASK, previous)

    def wait_empty(self, name, timeout):
        deadline = time.monotonic() + timeout
        while True:
            self.processes[name].poll()
            if not self.members(name):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.1)

    def stop(self, name, grace):
        proc = self.processes.get(name)
        if proc is None:
            return
        members = self.members(name)
        if members:
            # Let ros2 launch signal its own children once, preserving arm shutdown.
            if proc.poll() is None:
                self.send({pid: born for pid, born in members.items() if pid == proc.pid}, signal.SIGINT)
            else:
                self.send(members, signal.SIGINT)
            if not self.wait_empty(name, grace):
                print(f'{name}: graceful shutdown timed out; terminating remaining processes', flush=True)
                self.send(self.members(name), signal.SIGTERM)
                if not self.wait_empty(name, 3):
                    self.send(self.members(name), signal.SIGKILL)
                    if not self.wait_empty(name, 3):
                        raise RuntimeError(f'{name}: processes could not be stopped: {self.members(name)}')
        proc.wait(timeout=3)

    def check(self, cmd):
        proc = self.spawn('readiness', cmd)
        try:
            return proc.wait()
        finally:
            self.stop('readiness', 2)
            self.processes.pop('readiness', None)

    def cleanup(self, algorithms_grace):
        errors = []
        for name in ('readiness', 'algorithms', 'arm_serial', 'chassis', 'camera', 'lidar'):
            try:
                self.stop(name, algorithms_grace if name == 'algorithms' else 10)
            except Exception as exc:
                errors.append(str(exc))
        # Reap adopted orphan children after all direct Popen children were waited.
        while True:
            try:
                pid, _ = os.waitpid(-1, os.WNOHANG)
                if pid == 0:
                    break
            except ChildProcessError:
                break
        leftovers = {name: self.members(name) for name in self.processes if self.members(name)}
        if errors or leftovers:
            raise RuntimeError(f'Cleanup incomplete: {errors}; remaining processes: {leftovers}')
        print('Cleanup complete: no processes from this run remain.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigation-only', action='store_true', help='Skip arm serial/algorithms, grasp and unloading')
    parser.add_argument('--check', action='store_true', help='Check files/environments without starting or changing anything')
    parser.add_argument('--dry-run', action='store_true', help='Print startup commands only')
    parser.add_argument('--retries', type=int, default=3, help='Maximum attempts per sensor')
    parser.add_argument('--sensor-timeout', type=float, default=40)
    args = parser.parse_args()
    if args.retries < 1 or args.sensor_timeout <= 0:
        parser.error('retries and timeout must be positive')
    modules = {
        'camera': command('camera', 'ros2', 'launch', 'pose_estimator', 'camera_switch.launch.py'),
        'lidar': command('nav', 'ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py'),
        'algorithms': command('nav', 'ros2', 'launch', 'bringup', 'bringup.launch.py'),
        'chassis': command('chassis', 'ros2', 'launch', 'rm_serial_driver', 'serial.launch.py'),
        'arm_serial': command('arm', 'ros2', 'run', 'rm_serial_driver', 'node', '--ros-args',
                              '--params-file', ARM / 'install/rm_serial_driver/share/rm_serial_driver/config/serial_params.yaml',
                              '-r', '__node:=arm_serial_driver', '-r', '/cmd_vel:=/arm_task/unused_chassis_command'),
    }
    if args.navigation_only:
        modules['algorithms'].append('navigation_only:=true')
    if args.dry_run:
        import shlex
        for name, cmd in modules.items():
            if args.navigation_only and name == 'arm_serial':
                continue
            print(name + ': ' + shlex.join(cmd))
        print('Validate camera RGB + aligned depth points, lidar + IMU; retry sensors before algorithms.')
        return 0
    # Hold the lock for the entire run, including graceful arm/driver cleanup.
    # --check and --dry-run remain read-only and do not acquire this lock.
    run_lock = None
    if not args.check:
        lock_path = ROOT / 'log/mission/.mission.lock'
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        run_lock = lock_path.open('a+')
        try:
            fcntl.flock(run_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another mission is already running; stop the existing mission or rover-mission.service before starting again')
        run_lock.seek(0)
        run_lock.truncate()
        run_lock.write(str(os.getpid()) + '\n')
        run_lock.flush()
    required = [ROOT / 'install/setup.bash', ROOT / '.deps/livox-sdk2/setup.bash',
                ARM / 'install/setup.bash', CHASSIS / 'install/setup.bash',
                ARM / 'src/lite_motion_planner/launch/bring_launch.launch.py',
                ARM / 'install/rm_serial_driver/share/rm_serial_driver/config/serial_params.yaml']
    for file in required:
        if not file.is_file():
            raise RuntimeError(f'Missing required file: {file}')
    for scope, pkg in [('nav', 'bringup'), ('camera', 'pose_estimator'), ('arm', 'rm_serial_driver'), ('chassis', 'rm_serial_driver')]:
        subprocess.run(command(scope, 'ros2', 'pkg', 'prefix', pkg), check=True)
    if args.check:
        print('Preflight passed. No drivers started; no files changed.')
        return 0
    logs = ROOT / 'log/mission' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    logs.mkdir(parents=True)
    print(f'Logs: {logs}', flush=True)
    manager = ProcessManager()
    processes = manager.processes
    handles = []

    def start(name):
        output = open(logs / f'{name}.log', 'a', buffering=1)
        handles.append(output)
        proc = manager.spawn(name, modules[name], output)
        print(f'Started {name}: PID {proc.pid}', flush=True)

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    try:
        topics = {'camera': ['/camera/color/image_raw', '/camera/depth/color/points'],
                  'lidar': ['/livox/lidar', '/livox/imu']}
        for name in ('camera', 'lidar'):
            for attempt in range(1, args.retries + 1):
                start(name)
                check = manager.check(command('nav', 'python3', '-c', SENSOR_PROBE,
                                               '--timeout', args.sensor_timeout, *topics[name]))
                if check == 0 and processes[name].poll() is None:
                    print(f'{name}: data check passed', flush=True)
                    break
                manager.stop(name, 10)
                print(f'{name}: attempt {attempt}/{args.retries} failed', flush=True)
            else:
                raise RuntimeError(f'{name} failed after {args.retries} attempts; see {logs}')
        start('chassis')
        if not args.navigation_only:
            start('arm_serial')
            feedback = manager.check(command('nav', 'python3', '-c', SENSOR_PROBE,
                                              '--timeout', '15', '/serial/arm_joint_state'))
            if feedback != 0:
                raise RuntimeError('No arm serial feedback; algorithms were not started')
        time.sleep(1)
        for name in ('chassis',) if args.navigation_only else ('chassis', 'arm_serial'):
            if processes[name].poll() is not None:
                raise RuntimeError(f'{name} exited before algorithm startup')
        start('algorithms')
        time.sleep(3)
        if processes['algorithms'].poll() is not None:
            raise RuntimeError('Algorithms exited; inspect algorithms.log')
        print('All stages started. Task begins automatically when inputs/serial/arm are ready. Ctrl+C to stop.', flush=True)
        while True:
            for name, proc in processes.items():
                if proc.poll() is not None:
                    raise RuntimeError(f'{name} exited ({proc.returncode}); see {logs / (name + ".log")}')
            time.sleep(0.5)
    except KeyboardInterrupt:
        print('Stopping algorithms first; keep serial feedback available for arm shutdown.', flush=True)
    finally:
        # The arm shutdown home/hold watchdog needs up to 55+10 seconds.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
        try:
            manager.cleanup(20 if args.navigation_only else 75)
        finally:
            for output in handles:
                output.close()
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(1)

MISSION_PYTHON
