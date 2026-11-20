#!/usr/bin/env python3
"""Mission lifecycle wrapper: execute the unmodified standalone grasp launch."""
import ast
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from arm_task_interfaces.action import ExecuteArmTask
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, String
import yaml


def launch_default(path, name):
    for call in ast.walk(ast.parse(Path(path).read_text())):
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == 'DeclareLaunchArgument' and call.args and isinstance(call.args[0], ast.Constant) and call.args[0].value == name:
            for keyword in call.keywords:
                if keyword.arg == 'default_value':
                    return str(ast.literal_eval(keyword.value))
    raise RuntimeError(f'Missing {name} default in {path}')


def grasp_command(params_file=None):
    # All perception/planning/hand-eye/pose parameters come from this entry.
    # Camera and serial are already owned by start_mission.bash.
    command = ['ros2', 'launch', 'lite_motion_planner', 'bring_launch.launch.py',
            'autostart:=true', 'listen_navigation_goal_status:=false',
            'use_serial_driver:=false', 'use_external_serial:=true',
            'use_nav2arm:=false', 'use_arm2nav:=false', 'use_rviz:=false']
    if params_file:
        command.append(f'planner_params_file:={params_file}')
    return command


def live_group(group_id):
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            state = (entry / 'stat').read_text().rsplit(')', 1)[1].split()[0]
            if state != 'Z' and os.getpgid(int(entry.name)) == group_id:
                return True
        except (OSError, ValueError, IndexError):
            pass
    return False


def owned_members(process):
    marker = getattr(process, 'standalone_owner', None)
    members = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if (entry / 'stat').read_text().rsplit(')', 1)[1].split()[0] == 'Z':
                continue
            if marker:
                match = marker in (entry / 'environ').read_bytes().split(b'\0')
            else:
                match = os.getpgid(int(entry.name)) == process.pid
            if match:
                members.append(int(entry.name))
        except (OSError, ValueError, IndexError):
            pass
    return members


class StandaloneGraspExecutor(Node):
    def __init__(self):
        super().__init__('standalone_grasp_executor')
        self.group = ReentrantCallbackGroup()
        self.guard = threading.RLock()
        self.closing = False
        self.reserved = False
        self.busy = False
        self.session = ''
        self.task_id = 0
        self.success = False
        self.phase = 'idle'
        self.result_code = None
        self.reposition_target = None
        self.grasp_started_stamp = 0.
        self.grasp_proc = None
        self.detection_proc = None
        self.last_joint = 0.
        self.joint_history = deque()
        self.folded_tolerance = float(self.declare_parameter('folded_joint_tolerance', 0.15).value)
        if not math.isfinite(self.folded_tolerance) or not 0 < self.folded_tolerance <= 0.20:
            raise ValueError('folded_joint_tolerance must be in (0, 0.20] radians')
        planner = Path(get_package_share_directory('lite_motion_planner'))
        parameters = yaml.safe_load((planner / 'config/tzb_planner.yaml').read_text())['/**']['ros__parameters']
        self.ready_positions = parameters['tzb.pre_ini_joint_positions']
        if len(self.ready_positions) != 6:
            raise RuntimeError('Standalone pre_ini pose must contain six measured arm joints')
        self.model = str(self.declare_parameter('model_path', '').value) or launch_default(planner / 'launch/bring_launch.launch.py', 'model_path')
        self.target_class = str(self.declare_parameter('target_class', 'blue_block').value)
        self.target_range = float(self.declare_parameter('target_max_range', 1.5).value)
        self.timeout = float(self.declare_parameter('grasp_timeout', 90.).value)
        self.log_root = Path(os.environ.get('NAV_MISSION_ROOT', str(Path.home() / 'nav_explore_ws'))) / 'log/standalone_grasp'
        self.log_root.mkdir(parents=True, exist_ok=True)
        self.precise_near_x = float(self.declare_parameter('precise_near_x', .43).value)
        self.precise_far_x = float(self.declare_parameter('precise_far_x', .57).value)
        parameters.update({'tzb.chassis_adjustment_enabled': True,
                           'tzb.prepare_only': False,
                           'tzb.chassis_near_x': self.precise_near_x,
                           'tzb.chassis_far_x': self.precise_far_x,
                           'tzb.max_target_attempts': 1})
        self.precise_params = self.log_root / 'precise_position_planner.yaml'
        self.precise_params.write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
        prepare_parameters = dict(parameters)
        prepare_parameters.update({'tzb.prepare_only': True, 'tzb.chassis_adjustment_enabled': False,
                                   'tzb.arm_state_wait_timeout_sec': 3.0})
        self.prepare_params = self.log_root / 'prepare_arm_planner.yaml'
        self.prepare_params.write_text(yaml.safe_dump({'/**': {'ros__parameters': prepare_parameters}}))
        self.state_pub = self.create_publisher(String, '/arm_task/state', 10)
        self.create_subscription(JointState, '/serial/arm_joint_state', self.on_joints, qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(Int8, '/tzb_catch/result', self.on_result, 10, callback_group=self.group)
        self.create_subscription(PoseStamped, '/tzb_catch/chassis_adjustment', self.on_reposition, 10, callback_group=self.group)
        self.server = ActionServer(self, ExecuteArmTask, '/arm_task/execute', execute_callback=self.execute,
                                   goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)
        self.create_timer(.1, self.heartbeat, callback_group=self.group)
        self.start_detection()

    def on_joints(self, msg):
        lookup = dict(zip(msg.name, msg.position))
        try:
            values = [float(lookup[f'J{i}_joint']) for i in range(1, 7)]
        except (KeyError, ValueError):
            return
        if not all(math.isfinite(v) for v in values):
            return
        now = time.monotonic()
        with self.guard:
            self.last_joint = now
            self.joint_history.append((now, values))
            while len(self.joint_history) > 2 and now - self.joint_history[1][0] >= .35:
                self.joint_history.popleft()

    def folded(self):
        with self.guard:
            if time.monotonic() - self.last_joint > .6 or len(self.joint_history) < 3:
                return False
            return (self.joint_history[-1][0] - self.joint_history[0][0] >= .25 and
                    all(max(abs(a-b) for a,b in zip(values, self.ready_positions)) <= self.folded_tolerance for _,values in self.joint_history))

    def heartbeat(self):
        if not rclpy.ok():
            return
        with self.guard:
            if (not self.closing and not self.busy and not self.reserved and self.grasp_proc is None and
                    time.monotonic() >= getattr(self, 'detection_retry_at', 0.) and
                    (self.detection_proc is None or self.detection_proc.poll() is not None)):
                self.start_detection()
            safe = not self.busy and self.grasp_proc is None and self.folded()
            state = {'session': self.session, 'task_id': self.task_id,
                     'phase': 'BUSY' if self.busy else ('READY' if safe else 'ERROR'),
                     'arm_phase': self.phase, 'safe_to_drive': safe, 'success': self.success,
                     'ready_pose': 'pre_ini', 'stamp': self.get_clock().now().nanoseconds/1e9,
                     'execution': 'standalone_launch',
                     'reposition_target': self.reposition_target if self.result_code == -2 else None,
                     'exploration_detector_running': self.detection_proc is not None and self.detection_proc.poll() is None}
        self.state_pub.publish(String(data=json.dumps(state)))

    def spawn(self, command, label):
        stamp = time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns()%1000000:06d}'
        log = self.log_root / f'{stamp}_{label}.log'
        env = os.environ.copy()
        env['NAV_STANDALONE_OWNER'] = uuid.uuid4().hex
        with log.open('wb') as stream:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=stream,
                                       stderr=subprocess.STDOUT, start_new_session=True, env=env)
        process.standalone_owner = ('NAV_STANDALONE_OWNER=' + env['NAV_STANDALONE_OWNER']).encode()
        self.get_logger().info(f'[STANDALONE] start {label} pid={process.pid} log={log}')
        return process

    def start_detection(self):
        if self.closing or self.grasp_proc is not None:
            return
        try:
            self.detection_proc = self.spawn(
                ['ros2', 'launch', 'rover_state', 'target_detection.launch.py',
                 f'model_path:={self.model}', f'target_class:={self.target_class}', f'target_max_range:={self.target_range}'], 'target_detection')
        except Exception as exc:
            self.detection_proc = None
            self.get_logger().error(f'[STANDALONE] detection restart failed: {exc}; retrying independently of arm completion')
        self.detection_retry_at = time.monotonic() + 2.

    def stop_process(self, process, timeout):
        if process is None:
            return
        for sig, grace in [(signal.SIGINT, timeout), (signal.SIGTERM, 3.), (signal.SIGKILL, 3.)]:
            # Signal launch once; it forwards SIGINT to its children. Sending
            # SIGINT to the whole group would also interrupt arm homing twice.
            if sig == signal.SIGINT and process.poll() is None:
                process.send_signal(sig)
            else:
                for pid in owned_members(process):
                    try: os.kill(pid, sig)
                    except ProcessLookupError: pass
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                process.poll()
                if not owned_members(process):
                    return
                time.sleep(.05)
            self.get_logger().error(f'[STANDALONE] waiting for process group {process.pid} after {sig.name}')
        if owned_members(process):
            raise RuntimeError(f'Standalone process group {process.pid} could not be cleaned up')

    def on_result(self, msg):
        with self.guard:
            if self.busy and self.grasp_proc is not None and self.result_code is None and msg.data in (-2, -1, 1):
                self.result_code = int(msg.data)

    def on_reposition(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        xyz = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        with self.guard:
            if (self.busy and self.grasp_proc is not None and msg.header.frame_id == 'base_link' and
                    stamp >= self.grasp_started_stamp and
                    all(math.isfinite(v) for v in xyz)):
                self.reposition_target = {'frame': msg.header.frame_id, 'xyz': xyz, 'stamp': stamp}

    def goal(self, request):
        with self.guard:
            if (self.closing or self.reserved or request.command not in ('prepare', 'grasp') or
                    (request.command == 'grasp' and self.grasp_proc is not None)):
                return GoalResponse.REJECT
            self.reserved = True
            return GoalResponse.ACCEPT

    def execute(self, handle):
        request = handle.request
        with self.guard:
            self.session, self.task_id = request.session, int(request.task_id)
            self.busy = True
            self.success = False
            self.result_code = None
            self.reposition_target = None
            self.phase = request.command
        succeeded = False
        detail = ''
        try:
            if self.closing or handle.is_cancel_requested:
                detail = 'Canceled before standalone launch'
            else:
                preparing = request.command == 'prepare'
                if not preparing and not self.folded():
                    raise RuntimeError('Arm must be folded before starting standalone grasp')
                self.stop_process(self.detection_proc, 8.)
                self.detection_proc = None
                self.grasp_started_stamp = self.get_clock().now().nanoseconds/1e9
                params = self.prepare_params if preparing else self.precise_params
                label = 'prepare' if preparing else 'grasp'
                self.grasp_proc = self.spawn(grasp_command(params), f'{label}_{self.task_id}')
                deadline = time.monotonic() + (30. if preparing else self.timeout)
                while not self.closing and not handle.is_cancel_requested and time.monotonic() < deadline:
                    with self.guard:
                        code = self.result_code
                    if code is not None or self.grasp_proc.poll() is not None:
                        break
                    time.sleep(.05)
                with self.guard:
                    code = self.result_code
                succeeded = code == 1 and not handle.is_cancel_requested and not self.closing
                reason = ('completed' if code == 1 else 'reported failure' if code == -1 else
                          'canceled' if handle.is_cancel_requested or self.closing else
                          'process exited without result' if self.grasp_proc.poll() is not None else 'execution timeout')
                detail = f'Standalone {reason}, result={code}; cleaning up and confirming folded feedback'
                self.phase = 'cleanup'
                self.stop_process(self.grasp_proc, 60.)
                self.grasp_proc = None
                # Result is not released until both the process and actual arm
                # feedback agree that the next drive phase can start.
                deadline = time.monotonic() + 2.
                while not self.closing and not self.folded() and time.monotonic() < deadline:
                    time.sleep(.05)
        except Exception as exc:
            detail = str(exc)
            self.get_logger().error(f'[STANDALONE] {detail}')
        finally:
            # Cleanup/vision restart errors must still terminate the action and
            # release BUSY. A remaining arm process is reported explicitly.
            try:
                if self.grasp_proc is not None:
                    self.stop_process(self.grasp_proc, 60.)
                    self.grasp_proc = None
            except Exception as exc:
                succeeded = False
                detail += f'; arm cleanup failed: {exc}'
                self.get_logger().error(detail)
            try:
                if self.detection_proc is None and self.grasp_proc is None and not self.closing:
                    self.start_detection()
            except Exception as exc:
                self.get_logger().error(f'Detection restart failed: {exc}; arm result still returned')
            with self.guard:
                self.busy = False
                self.reserved = False
                self.success = succeeded and self.folded() and self.grasp_proc is None
                self.phase = 'idle' if self.grasp_proc is None else 'cleanup_failed'
            self.heartbeat()
        result = ExecuteArmTask.Result()
        result.success = self.success
        result.safe_to_drive = self.folded() and self.grasp_proc is None
        result.phase = 'READY' if result.safe_to_drive else 'ERROR'
        result.message = detail
        if handle.is_cancel_requested:
            handle.canceled()
        elif result.success:
            handle.succeed()
        else:
            handle.abort()
        return result

    def close(self):
        self.closing = True
        self.stop_process(self.detection_proc, 8.)
        self.detection_proc = None
        # The active execute callback owns grasp cleanup, to avoid signaling
        # the same arm process twice during Ctrl+C.
        deadline = time.monotonic() + 65.
        while self.busy and time.monotonic() < deadline:
            time.sleep(.05)
        if self.grasp_proc is not None:
            self.stop_process(self.grasp_proc, 2.)


def main():
    rclpy.init()
    node = StandaloneGraspExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        executor.shutdown(timeout_sec=3.)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
