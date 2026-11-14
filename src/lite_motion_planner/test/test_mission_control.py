"""Exercise the real arm task entry (tzb_catch, mission_controlled) through the
/arm_task/execute action with simulated planning, joint feedback and perception.

The fixture behaves like mission_state (nav_explore_ws/src/state/src/mission_node.cpp):
one goal per task, sent only when the server is ready; feedback is ignored; the
result is checked only for its phase; completion/readiness come from the
/arm_task/state heartbeat; pause/fault/shutdown = cancel the goal.
"""
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from lite_motion_msgs.srv import PlanArmMotion
from arm_task_interfaces.action import ExecuteArmTask

EXECUTABLE = sys.argv.pop(1)
SOURCE = Path(sys.argv.pop(1))
JOINTS = ['J1_joint', 'J2_joint', 'J3_joint', 'J4_joint', 'J5_joint', 'J6_joint']
OBSERVE = [0., 0., -2., 0., 1.57, 0.]
# Production pre_ini (tzb_planner.yaml): the arm pose at which navigation
# centers are observed during exploration.
PRE_INI_FIELD = [0., 1., -2.61, 0., 1.3, 0.]
# J4 is downstream of the camera; this distinct synthetic return pose keeps
# the fixture's fixed camera-space target unchanged across repeated tasks.
POST_GRASP = [0., 0., -2., .24, 1.57, 0.]
SENSOR_FRAME = 'camera_color_optical_frame'
# Non-trivial synthetic observation in the optical frame (bag median position,
# arbitrary rotation) so the whole chain, including orientation, is exercised.
CAMERA_XYZ = [0.0626, 0.2300, 0.5860]
CAMERA_RPY = [0.3, -0.2, 0.5]


# --- independent NumPy reference (adapted from issue4_coords/reference_calculator.py,
# whose URDF FK was validated against pinocchio to 8e-17 m) -------------------

def rpy_matrix(rpy):
    r, p, y = [float(v) for v in rpy]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    K = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(angle) * K + (1. - math.cos(angle)) * (K @ K)
    return T


def transform_xyz_rpy(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_matrix(rpy)
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def quaternion_matrix_xyzw(q):
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_quaternion_xyzw(R):
    t = float(np.trace(R))
    if t > 0.:
        s = math.sqrt(t + 1.) * 2.
        q = np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, .25 * s])
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1. + R[0, 0] - R[1, 1] - R[2, 2]) * 2.
            q = np.array([.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
        elif i == 1:
            s = math.sqrt(1. + R[1, 1] - R[0, 0] - R[2, 2]) * 2.
            q = np.array([(R[0, 1] + R[1, 0]) / s, .25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
        else:
            s = math.sqrt(1. + R[2, 2] - R[0, 0] - R[1, 1]) * 2.
            q = np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, .25 * s, (R[1, 0] - R[0, 1]) / s])
    return q / np.linalg.norm(q)


def urdf_fk(urdf_path, joint_positions, root_link='base_link', target_link='camera_link'):
    by_child = {}
    for j in ET.parse(urdf_path).getroot().findall('joint'):
        origin = j.find('origin')
        axis = j.find('axis')
        by_child[j.find('child').attrib['link']] = dict(
            name=j.attrib['name'], type=j.attrib['type'], parent=j.find('parent').attrib['link'],
            origin=transform_xyz_rpy(
                [float(v) for v in origin.attrib.get('xyz', '0 0 0').split()] if origin is not None else [0, 0, 0],
                [float(v) for v in origin.attrib.get('rpy', '0 0 0').split()] if origin is not None else [0, 0, 0]),
            axis=[float(v) for v in axis.attrib['xyz'].split()] if axis is not None else [1., 0., 0.])
    chain, child = [], target_link
    while child != root_link:
        chain.append(by_child[child])
        child = chain[-1]['parent']
    T = np.eye(4)
    for j in reversed(chain):
        T = T @ j['origin']
        if j['type'] in ('revolute', 'continuous'):
            T = T @ axis_rotation(j['axis'], joint_positions[j['name']])
        elif j['type'] != 'fixed':
            raise ValueError(j['type'])
    return T


def quaternion_from_rpy(rpy):
    return matrix_quaternion_xyzw(rpy_matrix(rpy))


def reference_chain(p, urdf, joints, camera_xyz, camera_quat_xyzw):
    """Grasp/navigation chain: FK(camera_link)·T_ref·pose, orientation·axis_conv,
    then the empirical base-frame correction. Returns (xyz, quat_xyzw)."""
    fk = urdf_fk(urdf, {f'J{i + 1}_joint': float(v) for i, v in enumerate(joints)},
                 'base_link', p['tzb.pose_estimator_reference_frame'])
    t_ref = np.array(p['tzb.pose_estimator_reference_transform_matrix'], dtype=float).reshape(4, 4)
    axis_conv = np.array(p['tzb.pose_estimator_axis_conversion_matrix'], dtype=float).reshape(3, 3)
    pose = np.eye(4)
    pose[:3, :3] = quaternion_matrix_xyzw(camera_quat_xyzw)
    pose[:3, 3] = camera_xyz
    T = fk @ t_ref @ pose
    R = T[:3, :3] @ axis_conv
    corr_R = rpy_matrix(np.radians(p['tzb.target_correction_rpy_deg']))
    v = corr_R @ T[:3, 3] + np.array(p['tzb.target_correction_xyz'], dtype=float)
    if v[2] < p['tzb.target_min_z']:
        v[2] = p['tzb.target_min_z']
    if p['tzb.target_left_y_threshold'] > 0. and v[1] > p['tzb.target_left_y_threshold']:
        v[1] += p['tzb.target_left_y_offset']
    return v, matrix_quaternion_xyzw(corr_R @ R)


def quat_distance(q1, q2):
    """Upper bound of the rotation angle (rad) between unit quaternions."""
    q1, q2 = np.asarray(q1, float), np.asarray(q2, float)
    return 2. * min(np.linalg.norm(q1 - q2), np.linalg.norm(q1 + q2))


# --- fixture ---------------------------------------------------------------

class Harness:
    """Launches the real tzb_catch binary with mocked planner, synthetic joint
    feedback and synthetic perception; drives it like mission_state."""

    def __init__(self, test, node_name, overrides=None):
        self.test = test
        self.node = rclpy.create_node(node_name)
        params = yaml.safe_load((SOURCE / 'config/tzb_planner.yaml').read_text())
        p = params['/**']['ros__parameters']
        self.urdf = str(SOURCE.parent / 'tzb_description/urdf/tzb_description.urdf')
        p.update(urdf_path=self.urdf, srdf_path=str(SOURCE / 'config/tzb_description.srdf'),
                 mesh_package_dirs=[str(SOURCE.parent / 'tzb_description')], enable_visualization=False)
        p.update({'tzb.mission_controlled': True, 'tzb.autostart': False,
                  'tzb.sentry_mode': False, 'tzb.replay_after_complete': False, 'tzb.sequence_loop': False,
                  'tzb.e3_collision_object_enabled': False, 'tzb.publish_tcp_trajectory': False,
                  'tzb.freeze_camera_pointcloud_enabled': False, 'tzb.publish_frozen_camera_tf': False,
                  'tzb.observe_joint_positions': OBSERVE, 'tzb.ini_joint_positions': [.05] + OBSERVE[1:],
                  'tzb.pre_ini_joint_positions': POST_GRASP,
                  'tzb.j6_arrival_tolerance_rad': .4,
                  'tzb.place_joint_positions': [.12] + OBSERVE[1:],
                  'tzb.pose_estimator_continuous_required_sec': .05,
                  'tzb.pose_estimator_wait_timeout_sec': 3., 'tzb.max_target_attempts': 1,
                  'tzb.arm_motion_stable_duration_sec': .05, 'tzb.arm_state_max_age_sec': .2,
                  'tzb.mission_feedback_grace_sec': .2,
                  'tzb.shutdown_home_timeout_sec': 10.})
        p.update(overrides or {})
        self.p = p
        self.params = params
        self.values = dict(joints=list(OBSERVE), phase={}, center=None, centers=[], plans=0, arm_commands=0,
                           feedback=True, publish_perception=True, slow_plans=False, pose_goals=0,
                           pending_center=None, plan_trace=[], pose_requests=[], command_trace=[],
                           joints_at_stamp={}, legacy=None, feedback_msgs=[])
        self.camera_quat = [float(v) for v in quaternion_from_rpy(CAMERA_RPY)]
        n, v = self.node, self.values
        self.feedback = n.create_publisher(JointState, '/serial/arm_joint_state', qos_profile_sensor_data)
        self.legacy_pub = n.create_publisher(String, '/arm_task/command', 10)
        self.centers = n.create_publisher(PoseStamped, '/perception/object_center', 10)
        self.poses = n.create_publisher(PoseStamped, '/pose_estimator/object_pose', 10)
        self.client = ActionClient(n, ExecuteArmTask, '/arm_task/execute')

        def on_command(msg):
            v['arm_commands'] += 1
            v['joints'] = list(msg.position)
            v['command_trace'].append(list(msg.position))

        n.create_subscription(JointState, '/tzb_catch/arm_joint_command', on_command, qos_profile_sensor_data)
        n.create_subscription(String, '/arm_task/state', lambda m: v.update(phase=json.loads(m.data)), 10)

        def on_center(m):
            stamp = (m.header.stamp.sec, m.header.stamp.nanosec)
            entry = dict(stamp=stamp, frame=m.header.frame_id,
                         xyz=[m.pose.position.x, m.pose.position.y, m.pose.position.z],
                         quat=[m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w],
                         joints=v['joints_at_stamp'].get(stamp))
            v['center'] = entry['xyz']
            v['centers'].append(entry)

        n.create_subscription(PoseStamped, '/mission/target_center', on_center, 10)

        def plan(req, response):
            v['plans'] += 1
            v['plan_trace'].append((req.use_pose_goal, list(req.goal_joint_positions)))
            if req.use_pose_goal:
                v['pose_goals'] += 1
                v['pose_requests'].append(dict(
                    frame=req.target_pose.header.frame_id,
                    xyz=[req.target_pose.pose.position.x, req.target_pose.pose.position.y, req.target_pose.pose.position.z],
                    quat=[req.target_pose.pose.orientation.x, req.target_pose.pose.orientation.y,
                          req.target_pose.pose.orientation.z, req.target_pose.pose.orientation.w]))
            response.success = True
            response.trajectory.joint_names = JOINTS
            first = JointTrajectoryPoint(positions=list(req.start_joint_positions))
            last = JointTrajectoryPoint(positions=([.08] + OBSERVE[1:]) if req.use_pose_goal else list(req.goal_joint_positions))
            last.time_from_start.nanosec = 150000000
            if v['slow_plans']:
                last.time_from_start.sec, last.time_from_start.nanosec = 1, 500000000
            response.trajectory.points = [first, last]
            return response

        n.create_service(PlanArmMotion, '/plan_arm_motion', plan)
        n.create_timer(.025, self.tick)
        self.process = None
        self.log = None
        self.directory = None

    def perception_msg(self, stamp):
        point = PoseStamped()
        point.header.stamp = stamp
        point.header.frame_id = SENSOR_FRAME
        point.pose.position.x, point.pose.position.y, point.pose.position.z = CAMERA_XYZ
        (point.pose.orientation.x, point.pose.orientation.y,
         point.pose.orientation.z, point.pose.orientation.w) = self.camera_quat
        return point

    def tick(self):
        v = self.values
        stamp = self.node.get_clock().now().to_msg()
        if v['feedback']:
            m = JointState(name=JOINTS, position=list(v['joints']))
            m.header.stamp = stamp
            self.feedback.publish(m)
            v['joints_at_stamp'][(stamp.sec, stamp.nanosec)] = list(v['joints'])
            if len(v['joints_at_stamp']) > 400:
                del v['joints_at_stamp'][next(iter(v['joints_at_stamp']))]
        if v['publish_perception']:
            point = self.perception_msg(stamp)
            # Deliver the center one camera frame late, after its grasp pose.
            # Cross-topic DDS arrival order must not break identity matching.
            if v['pending_center'] is not None:
                self.centers.publish(v['pending_center'])
            v['pending_center'] = point
            self.poses.publish(point)
        if v['legacy']:
            data = dict(v['legacy'], stamp=self.node.get_clock().now().nanoseconds / 1e9)
            self.legacy_pub.publish(String(data=json.dumps(data)))

    def until(self, predicate, timeout=8, allow_exit=False):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.02)
            if not allow_exit:
                self.test.assertIsNone(self.process.poll(), 'arm task process exited')
            if predicate():
                return True
        return False

    def wait(self, seconds):
        self.until(lambda: False, seconds)

    def ready(self, task, session='arm-test'):
        s = self.values['phase']
        return (s.get('session') == session and s.get('task_id') == task and s.get('phase') == 'READY'
                and s.get('safe_to_drive'))

    # mission_state semantics: one goal per task, sent when the server is ready.
    def send_goal(self, task_id, command, session='arm-test', expected=None, frame='base_link'):
        self.test.assertTrue(self.client.server_is_ready(), 'action server must be ready')
        goal = ExecuteArmTask.Goal()
        goal.session, goal.task_id, goal.command = session, task_id, command
        goal.expected_center.header.frame_id = frame
        goal.expected_center.header.stamp = self.node.get_clock().now().to_msg()
        if expected is not None:
            goal.expected_center.point.x, goal.expected_center.point.y, goal.expected_center.point.z = [float(c) for c in expected]
        future = self.client.send_goal_async(goal)  # feedback callback deliberately omitted
        self.test.assertTrue(self.until(future.done, 3), 'goal response')
        return future.result()

    def expect_rejected(self, *args, **kwargs):
        handle = self.send_goal(*args, **kwargs)
        self.test.assertFalse(handle.accepted, f'goal must be rejected: {args} {kwargs}')
        return handle

    def expect_accepted(self, *args, **kwargs):
        handle = self.send_goal(*args, **kwargs)
        self.test.assertTrue(handle.accepted, f'goal must be accepted: {args} {kwargs}')
        return handle

    def result(self, handle, timeout=15, allow_exit=False):
        future = handle.get_result_async()
        self.test.assertTrue(self.until(future.done, timeout, allow_exit=allow_exit), 'terminal result')
        return future.result()

    def cancel(self, handle):
        future = handle.cancel_goal_async()
        self.test.assertTrue(self.until(future.done, 3), 'cancel response')
        return future.result()

    def __enter__(self):
        self.directory = tempfile.TemporaryDirectory()
        path = Path(self.directory.name) / 'params.yaml'
        path.write_text(yaml.safe_dump(self.params))
        self.log = (Path(self.directory.name) / 'arm.log').open('w+')
        self.process = subprocess.Popen([EXECUTABLE, '--ros-args', '--params-file', str(path)],
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.test.assertTrue(self.until(lambda: bool(self.values['phase']) and self.client.server_is_ready(), 10),
                             'heartbeat and action server')
        return self

    def dump_log(self, tail=12000):
        self.log.flush()
        self.log.seek(0)
        print(self.log.read()[-tail:])

    def log_text(self):
        self.log.flush()
        self.log.seek(0)
        return self.log.read()

    def __exit__(self, exc_type, exc, tb):
        try:
            if exc_type is not None:
                self.dump_log()
        finally:
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
                try:
                    self.process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    self.process.kill()
            keep = os.environ.get('MISSION_ARM_TEST_LOG_DIR')
            if keep:
                os.makedirs(keep, exist_ok=True)
                self.log.flush()
                shutil.copy(self.log.name, os.path.join(keep, self.node.get_name() + '.log'))
            self.log.close()
            self.node.destroy_node()
            self.directory.cleanup()
        return False


class MissionArmTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    def centers_at(self, h, joints, minimum=3):
        want = [round(x, 9) for x in joints]
        return [c for c in h.values['centers'] if c['joints'] is not None and
                [round(x, 9) for x in c['joints']] == want][-minimum:]

    def test_navigation_center_matches_independent_chain(self):
        """/mission/target_center == FK(camera_link)·T_ref·p with the empirical
        correction, at the measured joints of the observation stamp."""
        with Harness(self, 'mission_arm_numeric') as h:
            errors = {}
            for label, joints in [('observe', OBSERVE), ('pre_ini_field', PRE_INI_FIELD),
                                  ('field_lock', [-0.0447, -0.9016, -1.5355, -0.0088, 0.6905, -0.0526])]:
                h.values['joints'] = list(joints)
                h.values['centers'] = []
                self.assertTrue(h.until(lambda: len(self.centers_at(h, joints)) >= 3, 5), label)
                for c in self.centers_at(h, joints):
                    self.assertEqual(c['frame'], h.p['planning_frame'])
                    xyz, quat = reference_chain(h.p, h.urdf, c['joints'], CAMERA_XYZ, h.camera_quat)
                    pos_err = float(np.linalg.norm(np.array(c['xyz']) - xyz))
                    rot_err = quat_distance(c['quat'], quat)
                    errors.setdefault(label, []).append((pos_err, rot_err))
                    self.assertLessEqual(pos_err, 1e-6, f'{label}: position error {pos_err} m, got {c["xyz"]} want {xyz.tolist()}')
                    self.assertLessEqual(rot_err, 1e-6, f'{label}: orientation error {rot_err} rad')
            print('NUMERIC_CHAIN_ERRORS', json.dumps({k: dict(max_pos=max(e[0] for e in v), max_rot=max(e[1] for e in v))
                                                      for k, v in errors.items()}))
            # Sanity: the correction/T_ref actually matter (a chain without them is far away).
            naive = urdf_fk(h.urdf, {f'J{i + 1}_joint': v for i, v in enumerate(OBSERVE)}) @ np.array(CAMERA_XYZ + [1.])
            full = reference_chain(h.p, h.urdf, OBSERVE, CAMERA_XYZ, h.camera_quat)[0]
            self.assertGreater(float(np.linalg.norm(naive[:3] - full)), 0.2)

    def test_old_or_unmatched_observations_produce_no_center(self):
        """Age window and the 0.10 s joint-sample gate are unchanged."""
        with Harness(self, 'mission_arm_gates') as h:
            h.values['publish_perception'] = False
            h.wait(.5)
            h.values['centers'] = []
            now = h.node.get_clock().now()
            # 1. older than the 0.6 s age window
            h.centers.publish(h.perception_msg((now - rclpy.duration.Duration(seconds=1.0)).to_msg()))
            # 2. inside the age window but no joint sample within 0.10 s of the stamp
            h.centers.publish(h.perception_msg((now + rclpy.duration.Duration(seconds=0.13)).to_msg()))
            # 3. wrong frame
            bad = h.perception_msg(now.to_msg())
            bad.header.frame_id = 'camera_link'
            h.centers.publish(bad)
            h.wait(.6)
            self.assertEqual(h.values['centers'], [])
            # 4. stale joint feedback (> arm_state_max_age) even with a matching sample
            h.values['feedback'] = False
            h.wait(.35)
            stamp = (h.node.get_clock().now() - rclpy.duration.Duration(seconds=0.3)).to_msg()
            h.centers.publish(h.perception_msg(stamp))
            h.wait(.4)
            self.assertEqual(h.values['centers'], [])
            # control: a fresh, matched observation is relayed
            h.values['feedback'] = True
            h.wait(.3)
            h.centers.publish(h.perception_msg(h.node.get_clock().now().to_msg()))
            self.assertTrue(h.until(lambda: len(h.values['centers']) == 1, 2))

    def test_action_contract(self):
        with Harness(self, 'mission_arm_contract', {'tzb.force_target_roll': False, 'tzb.force_target_pitch': False,
                                                     'tzb.grasp_yaw_offset_deg': 0.0}) as h:
            v = h.values
            self.assertTrue(h.until(lambda: v['center'] is not None))
            self.assertEqual(v['arm_commands'], 0, 'mission mode must not move at startup')
            self.assertEqual(v['phase']['phase'], 'WAITING')
            expected_center = self.centers_at(h, OBSERVE)[-1]  # nav-chain center at the grasp observe pose
            # --- rejections before any prepare
            h.expect_rejected(1, 'grasp', expected=expected_center['xyz'])
            h.expect_rejected(1, 'dance')
            self.assertEqual(v['phase']['phase'], 'WAITING')
            # --- prepare -> READY, safe_to_drive from measured joints, result phase READY
            handle = h.expect_accepted(1, 'prepare')
            result = h.result(handle)
            self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
            self.assertEqual(result.result.phase, 'READY')
            self.assertTrue(result.result.success)
            self.assertTrue(result.result.safe_to_drive)
            self.assertTrue(h.until(lambda: h.ready(1)))
            self.assertEqual(v['phase']['ready_pose'], 'pre_ini')
            self.assertEqual(v['joints'], POST_GRASP)
            # Real J6 feedback can settle 0.244 rad from the preset, within its 0.4 rad tolerance.
            v['joints'][5] = .244
            h.wait(.3)
            self.assertTrue(h.ready(1), 'arrival and drive readiness must agree on J6 tolerance')
            v['joints'][5] = .45
            self.assertTrue(h.until(lambda: not v['phase'].get('safe_to_drive'), 2))
            v['joints'] = list(POST_GRASP)
            v['joints'][1] += .15
            h.wait(.3)
            self.assertFalse(v['phase'].get('safe_to_drive'), 'other joints retain 0.1 rad tolerance')
            v['joints'] = list(POST_GRASP)
            v['joints'][0] += 2 * math.pi
            self.assertTrue(h.until(lambda: h.ready(1), 2), 'equivalent angles follow arrival semantics')
            v['joints'] = list(POST_GRASP)
            # --- rejections while READY (each would fault mission_state, so they must be contract violations)
            h.expect_rejected(1, 'prepare')                                   # duplicate task_id
            h.expect_rejected(0, 'grasp', expected=expected_center['xyz'])     # older task_id
            h.expect_rejected(2, 'grasp', session='intruder', expected=expected_center['xyz'])  # foreign session
            h.expect_rejected(2, 'grasp', expected=expected_center['xyz'], frame='base_frame')  # wrong frame
            h.expect_rejected(2, 'grasp', expected=expected_center['xyz'], frame='')
            h.expect_rejected(2, 'grasp', expected=[float('nan'), 0., 0.])
            h.expect_rejected(2, 'wave')
            # --- legacy /arm_task/command is ignored by default
            plans = v['plans']
            v['legacy'] = dict(session='arm-test', task_id=2, command='prepare')
            h.wait(.6)
            v['legacy'] = None
            self.assertEqual(v['plans'], plans, 'legacy JSON command must not trigger a task')
            self.assertEqual(v['phase']['task_id'], 1)
            self.assertTrue(h.ready(1))
            # --- grasp with matching expected_center -> success; grasp chain == nav chain
            for task in [2, 3]:
                trace_start, pose_start = len(v['plan_trace']), len(v['pose_requests'])
                handle = h.expect_accepted(task, 'grasp', expected=expected_center['xyz'])
                result = h.result(handle)
                self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
                self.assertEqual(result.result.phase, 'READY')
                self.assertTrue(result.result.success)
                self.assertTrue(h.until(lambda: h.ready(task), 3))
                self.assertTrue(v['phase']['success'])
                self.assertEqual(v['phase']['ready_pose'], 'pre_ini')
                for actual, expected in zip(v['joints'], POST_GRASP):
                    self.assertAlmostEqual(actual, expected, places=5)
                trace = v['plan_trace'][trace_start:]
                first_pose = next(i for i, (is_pose, _) in enumerate(trace) if is_pose)
                self.assertIn((False, [.05] + OBSERVE[1:]), trace[:first_pose])
                self.assertIn((False, OBSERVE), trace[:first_pose],
                              'formal grasp must still observe after reaching its initial pose')
                self.assertEqual(trace[-1], (False, POST_GRASP))
                grasp = v['pose_requests'][-1]  # final descent goal (hover was added 11-06; the first request is hover)
                self.assertEqual(grasp['frame'], h.p['planning_frame'])
                nav = self.centers_at(h, OBSERVE)[-1]
                pos_err = float(np.linalg.norm(np.array(grasp['xyz']) - np.array(nav['xyz'])))
                rot_err = quat_distance(grasp['quat'], nav['quat'])
                print('GRASP_VS_NAV_CHAIN', json.dumps(dict(task=task, pos=pos_err, rot=rot_err)))
                self.assertLessEqual(pos_err, 1e-9, 'grasp chain and navigation chain must agree at identical joints')
                self.assertLessEqual(rot_err, 1e-9)
                # A return to the old observation pose must not be mistaken for arrival at the post-grasp pose.
                v['joints'] = list(OBSERVE)
                self.assertTrue(h.until(lambda: not v['phase'].get('safe_to_drive'), 2))
                v['joints'] = list(POST_GRASP)
                self.assertTrue(h.until(lambda: h.ready(task), 2))
            self.assertEqual(v['pose_goals'], 2)
            # --- cancel while waiting for the target at observe; a second goal meanwhile is rejected
            v['publish_perception'] = False
            handle = h.expect_accepted(4, 'grasp', expected=expected_center['xyz'])
            self.assertTrue(h.until(lambda: v['phase'].get('arm_phase') == 'locate' and v['joints'] == OBSERVE, 5))
            self.assertEqual(v['phase']['phase'], 'BUSY')
            self.assertFalse(v['phase']['safe_to_drive'])
            h.expect_rejected(5, 'prepare')
            plans = v['plans']
            h.cancel(handle)
            result = h.result(handle)
            self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
            self.assertEqual(result.result.phase, 'READY')
            self.assertFalse(result.result.success)
            self.assertTrue(result.result.safe_to_drive)
            self.assertIn('canceled', result.result.message)
            self.assertTrue(h.until(lambda: h.ready(4), 3))
            self.assertFalse(v['phase']['success'])
            self.assertEqual(v['joints'], POST_GRASP)
            self.assertEqual(v['plan_trace'][-1], (False, POST_GRASP), 'cancel recovers to pre_ini')
            self.assertGreater(v['plans'], plans)
            v['publish_perception'] = True
            # --- cancel mid-motion: the segment is held, then recovery to pre_ini (measured)
            v['slow_plans'] = True
            commands = len(v['command_trace'])
            handle = h.expect_accepted(5, 'grasp', expected=expected_center['xyz'])
            # ini differs from pre_ini by J4 0.24 -> 0; cancel once the held sample is
            # outside the 0.1 rad tolerance so a real recovery motion is required
            # (a hold inside tolerance counts as already home, like shutdown homing).
            self.assertTrue(h.until(lambda: v['phase'].get('arm_phase') == 'prepare' and v['joints'][3] < .12, 5))
            h.cancel(handle)
            result = h.result(handle)
            self.assertEqual(result.status, GoalStatus.STATUS_CANCELED)
            self.assertEqual(result.result.phase, 'READY')
            self.assertTrue(result.result.safe_to_drive)
            self.assertTrue(h.until(lambda: h.ready(5), 3))
            self.assertEqual(v['joints'], POST_GRASP)
            self.assertEqual(v['plan_trace'][-1], (False, POST_GRASP), 'held pose outside tolerance must be recovered by motion')
            self.assertIn('_abort_hold_command', h.log_text())
            held = [c for c in v['command_trace'][commands:] if 0. < c[0] < .05]
            self.assertTrue(held, 'the aborted segment must hold a mid-trajectory sample, not jump to its goal')
            v['slow_plans'] = False
            # --- wrong expected_center -> READY, success:false, recovered
            handle = h.expect_accepted(6, 'grasp', expected=[50., 50., 50.])
            result = h.result(handle)
            self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
            self.assertEqual(result.result.phase, 'READY')
            self.assertFalse(result.result.success)
            self.assertTrue(result.result.safe_to_drive)
            self.assertTrue(h.until(lambda: h.ready(6)))
            self.assertFalse(v['phase']['success'])
            self.assertTrue(v['phase']['safe_to_drive'], 'failed grasp must recover for continued exploration')
            self.assertEqual(v['joints'], POST_GRASP)
            self.assertEqual(v['pose_goals'], 2, 'a different observed object must not be grasped')
            # --- a prepare from a new session adopts it; its grasp is then accepted
            h.expect_rejected(7, 'grasp', session='arm-test-2', expected=expected_center['xyz'])
            handle = h.expect_accepted(7, 'prepare', session='arm-test-2')
            self.assertEqual(h.result(handle).result.phase, 'READY')
            self.assertTrue(h.until(lambda: h.ready(7, 'arm-test-2')))
            h.expect_rejected(8, 'grasp', session='arm-test', expected=expected_center['xyz'])  # old session now foreign
            # --- feedback loss drops safe_to_drive
            v['feedback'] = False
            self.assertTrue(h.until(lambda: not v['phase'].get('safe_to_drive'), 2))

    def test_sigint_during_grasp_gives_terminal_result_and_homes(self):
        with Harness(self, 'mission_arm_sigint') as h:
            v = h.values
            self.assertTrue(h.until(lambda: v['center'] is not None))
            expected_center = self.centers_at(h, OBSERVE)[-1]
            handle = h.expect_accepted(1, 'prepare')
            self.assertEqual(h.result(handle).result.phase, 'READY')
            self.assertTrue(h.until(lambda: h.ready(1)))
            v['publish_perception'] = False
            handle = h.expect_accepted(2, 'grasp', expected=expected_center['xyz'])
            self.assertTrue(h.until(lambda: v['phase'].get('arm_phase') == 'locate' and v['joints'] == OBSERVE, 5))
            h.process.send_signal(signal.SIGINT)
            result = h.result(handle, timeout=15, allow_exit=True)
            self.assertEqual(result.status, GoalStatus.STATUS_ABORTED)
            self.assertEqual(result.result.phase, 'READY', 'shutdown homing succeeded -> READY, not ERROR')
            self.assertFalse(result.result.success)
            self.assertIn('shutting down', result.result.message)
            self.assertEqual(h.process.wait(timeout=15), 0)
            self.assertEqual(v['joints'], POST_GRASP, 'shutdown homing (issue 5) must end at pre_ini')
            text = h.log_text()
            self.assertIn('[SHUTDOWN][HOME][OK]', text)
            self.assertIn('[MISSION][result] task_id=2 aborted by shutdown phase=READY', text)
            self.assertEqual(v['phase'].get('phase'), 'SHUTDOWN')


if __name__ == '__main__':
    unittest.main()
