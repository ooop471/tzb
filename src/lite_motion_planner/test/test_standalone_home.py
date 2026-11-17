"""Real tzb_catch with fake planning/joints; no serial or hardware driver."""
import signal
import subprocess
import tempfile
import unittest
from pathlib import Path
import rclpy
import yaml
from std_msgs.msg import Int8
# This fixture consumes executable/source arguments from sys.argv.
from test_mission_control import Harness, EXECUTABLE

class StandaloneHarness(Harness):
    def __init__(self, test, name):
        super().__init__(test,name,{'tzb.mission_controlled':False,'tzb.autostart':True,
            'tzb.prepare_only':True,'tzb.arm_motion_stable_duration_sec':.25})
        self.result_code=None
        self.node.create_subscription(Int8,'/tzb_catch/result',lambda m:setattr(self,'result_code',m.data),10)
    def __enter__(self):
        self.directory=tempfile.TemporaryDirectory()
        path=Path(self.directory.name)/'params.yaml';path.write_text(yaml.safe_dump(self.params))
        self.log=(Path(self.directory.name)/'arm.log').open('w+')
        self.process=subprocess.Popen([EXECUTABLE,'--ros-args','--params-file',str(path)],stdout=self.log,stderr=subprocess.STDOUT)
        self.test.assertTrue(self.until(lambda:self.result_code is not None,15),'prepare result')
        self.test.assertEqual(self.result_code,1)
        return self

class StandaloneHomeTest(unittest.TestCase):
    def setUp(self):rclpy.init()
    def tearDown(self):rclpy.shutdown()
    def test_completed_prepare_does_not_command_home_twice(self):
        with StandaloneHarness(self,'standalone_no_duplicate') as h:
            h.wait(.1);before=h.values['arm_commands']
            h.process.send_signal(signal.SIGINT)
            self.assertTrue(h.until(lambda:h.process.poll() is not None,10,allow_exit=True))
            self.assertEqual(h.process.returncode,0)
            self.assertEqual(h.values['arm_commands'],before)
            self.assertIn('reason=completed_and_stably_home',h.log_text())
            self.assertNotIn('step=shutdown_home',h.log_text())
    def test_joint_drift_still_requires_shutdown_recovery(self):
        with StandaloneHarness(self,'standalone_drift') as h:
            h.values['joints'][1]+=.25;h.wait(.3);before=h.values['arm_commands']
            h.process.send_signal(signal.SIGINT)
            self.assertTrue(h.until(lambda:h.process.poll() is not None,15,allow_exit=True))
            self.assertEqual(h.process.returncode,0)
            self.assertGreater(h.values['arm_commands'],before)
            self.assertNotIn('reason=completed_and_stably_home',h.log_text())
            self.assertIn('[SHUTDOWN][HOME][OK]',h.log_text())
if __name__=='__main__':unittest.main()
