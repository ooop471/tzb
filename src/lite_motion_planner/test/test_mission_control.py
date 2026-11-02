"""Exercise the real arm task entry using simulated planning and joint feedback."""
import json
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
import yaml
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from lite_motion_msgs.srv import PlanArmMotion

EXECUTABLE = sys.argv.pop(1)
SOURCE = Path(sys.argv.pop(1))
JOINTS = ['J1_joint', 'J2_joint', 'J3_joint', 'J4_joint', 'J5_joint', 'J6_joint']
OBSERVE = [0., 0., -2., 0., 1.57, 0.]
# J4 is downstream of the camera; this distinct synthetic return pose keeps
# the fixture's fixed camera-space target unchanged across repeated tasks.
POST_GRASP = [0., 0., -2., .24, 1.57, 0.]


class MissionArmTest(unittest.TestCase):
    def test_prepare_repeat_grasp_identity_and_feedback_interlocks(self):
        rclpy.init()
        node = rclpy.create_node('mission_arm_fixture')
        params = yaml.safe_load((SOURCE / 'config/tzb_planner.yaml').read_text())
        p = params['/**']['ros__parameters']
        p.update(urdf_path=str(SOURCE.parent / 'tzb_description/urdf/tzb_description.urdf'),
                 srdf_path=str(SOURCE / 'config/tzb_description.srdf'),
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
                  'tzb.pose_estimator_wait_timeout_sec': .4, 'tzb.max_target_attempts': 1,
                  'tzb.arm_motion_stable_duration_sec': .05, 'tzb.arm_state_max_age_sec': .2})
        p['tzb.mission_feedback_grace_sec'] = .2
        feedback = node.create_publisher(JointState, '/serial/arm_joint_state', qos_profile_sensor_data)
        commands = node.create_publisher(String, '/arm_task/command', 10)
        centers = node.create_publisher(PoseStamped, '/perception/object_center', 10)
        poses = node.create_publisher(PoseStamped, '/pose_estimator/object_pose', 10)
        values = dict(joints=[.08] + OBSERVE[1:], phase={}, center=None, plans=0, arm_commands=0,
                      feedback=True, command=None, pose_goals=0, pending_center=None, plan_trace=[])

        def on_command(msg):
            values['arm_commands'] += 1
            values['joints'] = list(msg.position)

        node.create_subscription(JointState, '/tzb_catch/arm_joint_command', on_command, qos_profile_sensor_data)
        node.create_subscription(String, '/arm_task/state', lambda m: values.update(phase=json.loads(m.data)), 10)
        node.create_subscription(PoseStamped, '/mission/target_center',
                                 lambda m: values.update(center=[m.pose.position.x, m.pose.position.y, m.pose.position.z]), 10)

        def plan(req, response):
            values['plans'] += 1
            values['plan_trace'].append((req.use_pose_goal, list(req.goal_joint_positions)))
            if req.use_pose_goal:
                values['pose_goals'] += 1
            response.success = True
            response.trajectory.joint_names = JOINTS
            first = JointTrajectoryPoint(positions=list(req.start_joint_positions))
            last = JointTrajectoryPoint(positions=([.08] + OBSERVE[1:]) if req.use_pose_goal else list(req.goal_joint_positions))
            last.time_from_start.nanosec = 150000000
            response.trajectory.points = [first, last]
            return response

        node.create_service(PlanArmMotion, '/plan_arm_motion', plan)

        def tick():
            stamp = node.get_clock().now().to_msg()
            if values['feedback']:
                m = JointState(name=JOINTS, position=values['joints'])
                m.header.stamp = stamp
                feedback.publish(m)
            point = PoseStamped()
            point.header.stamp = stamp
            point.header.frame_id = 'camera_color_optical_frame'
            point.pose.position.z = 1.
            point.pose.orientation.w = 1.
            # Deliver the center one camera frame late, after its grasp pose.
            # Cross-topic DDS arrival order must not break identity matching.
            if values['pending_center'] is not None:
                centers.publish(values['pending_center'])
            values['pending_center'] = point
            poses.publish(point)
            if values['command']:
                data = dict(values['command'], stamp=node.get_clock().now().nanoseconds / 1e9)
                commands.publish(String(data=json.dumps(data)))

        node.create_timer(.025, tick)

        def until(predicate, timeout=8):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
                self.assertIsNone(process.poll(), 'arm task process exited')
                if predicate():
                    return True
            return False

        def ready(task):
            state = values['phase']
            return state.get('task_id') == task and state.get('phase') == 'READY' and state.get('safe_to_drive')

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'params.yaml'
            path.write_text(yaml.safe_dump(params))
            with (Path(directory) / 'arm.log').open('w+') as log:
                process = subprocess.Popen([EXECUTABLE, '--ros-args', '--params-file', str(path)], stdout=log, stderr=subprocess.STDOUT)
                try:
                    self.assertTrue(until(lambda: bool(values['phase']) and values['center'] is not None))
                    self.assertEqual(values['arm_commands'], 0, 'mission mode must not move at startup')
                    values['command'] = dict(session='arm-test', task_id=1, command='prepare')
                    self.assertTrue(until(lambda: ready(1)))
                    self.assertEqual(values['phase']['ready_pose'], 'pre_ini')
                    self.assertEqual(values['joints'], POST_GRASP)
                    # Real J6 feedback can settle 0.244 rad from the preset,
                    # within its existing 0.4 rad arrival tolerance.
                    values['joints'][5] = .244
                    until(lambda: False, .3)
                    self.assertTrue(ready(1), 'arrival and drive readiness must agree on J6 tolerance')
                    values['joints'][5] = .45
                    self.assertTrue(until(lambda: not values['phase'].get('safe_to_drive'), 2))
                    values['joints'] = list(POST_GRASP)
                    values['joints'][1] += .15
                    until(lambda: False, .3)
                    self.assertFalse(values['phase'].get('safe_to_drive'), 'other joints retain 0.1 rad tolerance')
                    values['joints'] = list(POST_GRASP)
                    values['joints'][0] += 2 * 3.141592653589793
                    self.assertTrue(until(lambda: ready(1), 2), 'equivalent angles follow arrival semantics')
                    values['joints'] = list(POST_GRASP)
                    initial = values['plans']
                    until(lambda: False, .3)
                    self.assertEqual(values['plans'], initial, 'duplicate prepare must not be queued')
                    for task in [2, 3]:
                        trace_start = len(values['plan_trace'])
                        values['command'] = dict(session='arm-test', task_id=task, command='grasp',
                                                 expected_center=values['center'])
                        self.assertTrue(until(lambda: ready(task), 10))
                        self.assertTrue(values['phase']['success'])
                        self.assertEqual(values['phase']['ready_pose'], 'pre_ini')
                        for actual, expected in zip(values['joints'], POST_GRASP):
                            self.assertAlmostEqual(actual, expected, places=5)
                        trace = values['plan_trace'][trace_start:]
                        first_pose = next(i for i, (is_pose, _) in enumerate(trace) if is_pose)
                        self.assertIn((False, [.05] + OBSERVE[1:]), trace[:first_pose])
                        self.assertIn((False, OBSERVE), trace[:first_pose],
                                      'formal grasp must still observe after reaching its initial pose')
                        self.assertEqual(trace[-1], (False, POST_GRASP))
                        # A return to the old observation pose must not be mistaken for
                        # physical arrival at the new post-grasp pose.
                        values['joints'] = list(OBSERVE)
                        self.assertTrue(until(lambda: not values['phase'].get('safe_to_drive'), 2))
                        values['joints'] = list(POST_GRASP)
                        self.assertTrue(until(lambda: ready(task), 2))
                    self.assertEqual(values['pose_goals'], 2)
                    values['command'] = dict(session='arm-test', task_id=4, command='grasp', expected_center=[50., 50., 50.])
                    self.assertTrue(until(lambda: ready(4)))
                    self.assertFalse(values['phase']['success'])
                    self.assertTrue(values['phase']['safe_to_drive'], 'failed grasp must recover for continued exploration')
                    self.assertEqual(values['phase']['ready_pose'], 'pre_ini')
                    self.assertEqual(values['joints'], POST_GRASP)
                    self.assertEqual(values['pose_goals'], 2, 'a different observed object must not be grasped')
                    values['command'] = dict(session='arm-test', task_id=5, command='prepare')
                    self.assertTrue(until(lambda: ready(5)))
                    self.assertEqual(values['phase']['ready_pose'], 'pre_ini')
                    values['feedback'] = False
                    self.assertTrue(until(lambda: not values['phase'].get('safe_to_drive'), 2))
                except Exception:
                    log.flush()
                    log.seek(0)
                    print(log.read()[-10000:])
                    raise
                finally:
                    process.send_signal(signal.SIGINT)
                    process.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
