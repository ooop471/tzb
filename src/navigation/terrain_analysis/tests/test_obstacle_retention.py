"""Real terrain executable with synthetic clouds/odometry; no sensor driver."""
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import create_cloud
from std_msgs.msg import Header, Float32
EXECUTABLE=sys.argv.pop(1)

class ObstacleRetentionTest(unittest.TestCase):
    def setUp(self):
        rclpy.init();self.node=rclpy.create_node('terrain_retention_fixture')
        self.cloud=self.node.create_publisher(PointCloud2,'/cloud_registered',10)
        self.odom=self.node.create_publisher(Odometry,'/odometry',10)
        self.clear=self.node.create_publisher(Float32,'/map_clearing',10)
        self.received=[]
        self.node.create_subscription(PointCloud2,'/terrain_map',self.received.append,10)
        self.log=tempfile.TemporaryFile(mode='w+')
        self.process=subprocess.Popen([EXECUTABLE,'--ros-args','--params-file',
            str(Path(__file__).resolve().parents[1]/'config/terrain_analysis_plus.yaml'),
            '-p','voxelPointUpdateThre:='+('100000' if 'between_voxel' in self._testMethodName else '1'),
            '-p','voxelTimeUpdateThre:='+('1000.0' if 'between_voxel' in self._testMethodName else '0.5'),
            '-p','minBlockPointNum:=5','-p','minObstacleHeight:=0.1',
            '-p','noDecayDis:=8.0','-p','decayTime:=2.0','-p','selfFilterEnabled:=false'],
            stdout=self.log,stderr=subprocess.STDOUT)
        self.wait(lambda:self.cloud.get_subscription_count()>0 and self.odom.get_subscription_count()>0)
        self.set_position(0.)
        self.points=[(1.01+i*.04,-.49+j*.04,0.) for i in range(26) for j in range(26)]
        self.points += [(1.51,-.45+j*.04,.35) for j in range(24)]
    def wait(self,ready,timeout=5.):
        end=time.monotonic()+timeout
        while not ready() and time.monotonic()<end:
            self.assertIsNone(self.process.poll())
            rclpy.spin_once(self.node,timeout_sec=.02)
        self.assertTrue(ready(),'terrain fixture timed out')
    def set_position(self,x):
        msg=Odometry();msg.header.frame_id='odom_init';msg.pose.pose.orientation.w=1.;msg.pose.pose.position.x=x
        self.odom.publish(msg)
        end=time.monotonic()+.1
        while time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.01)
    def send(self,points,t):
        header=Header();header.frame_id='odom_init';header.stamp.sec=int(t);header.stamp.nanosec=int(round((t-int(t))*1e9))
        fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z','intensity'))]
        msg=create_cloud(header,fields,[(*p,0.) for p in points]);self.cloud.publish(msg)
        same=lambda m:m.header.stamp==header.stamp
        self.wait(lambda:any(same(m) for m in self.received))
        return next(m.width*m.height for m in reversed(self.received) if same(m))
    def test_observed_obstacles_stay_and_unobserved_nearby_points_expire(self):
        first=self.send(self.points,100.);self.assertGreater(first,0)
        for i in range(1,25):
            count=self.send(self.points,100.+i*.1)
            self.assertGreater(count,0,'continuously observed obstacle disappeared')
            self.assertLessEqual(count,first*2)
        self.assertGreater(self.send([],103.),0,'observation expired too early')
        self.assertEqual(self.send([],104.5),0,'nearby old obstacle survived decayTime')

    def test_new_ground_does_not_renew_old_obstacle(self):
        self.assertGreater(self.send(self.points,100.),0)
        ground=[p for p in self.points if p[2]==0.]
        for i in range(1,26):count=self.send(ground,100.+i*.1)
        self.assertEqual(count,0,'fresh ground or snapshot accumulation resurrected old obstacle')
        self.assertGreater(self.send(self.points,103.),0,'new real obstacle was not marked again')

    def test_expiry_between_voxel_rebuilds(self):
        self.assertGreater(self.send(self.points,100.),0)
        self.assertGreater(self.send([],101.),0)
        self.assertEqual(self.send([],102.1),0,'expiry depended on voxel rebuild interval')

    def test_motion_preserves_visible_wall_and_expiry(self):
        for i in range(9):
            self.set_position(i*.05)
            self.assertGreater(self.send(self.points,100.+i*.1),0,'visible world-fixed wall lost during motion')
        self.set_position(.6)
        self.assertEqual(self.send([],102.9),0,'motion renewed an unobserved obstacle')

    def test_burst_reaches_latest_source_stamp(self):
        self.assertGreater(self.send(self.points,100.),0)
        self.received.clear()
        fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,n in enumerate(('x','y','z','intensity'))]
        msg=create_cloud(Header(frame_id='odom_init'),fields,[(*p,0.) for p in self.points])
        for i in range(30):
            msg.header.stamp.sec=101
            msg.header.stamp.nanosec=i*1000000
            self.cloud.publish(msg)
        # A short burst must drain to the newest observation with its original
        # timestamp, rather than restamping historical geometry as current.
        self.wait(lambda:any(m.header.stamp==msg.header.stamp for m in self.received),timeout=3.)
        self.assertGreater(self.received[-1].width,0)

    def test_explicit_clear_removes_accumulated_obstacles(self):
        for i in range(3):self.assertGreater(self.send(self.points,100.+i*.1),0)
        self.clear.publish(Float32(data=20.))
        end=time.monotonic()+.1
        while time.monotonic()<end:rclpy.spin_once(self.node,timeout_sec=.01)
        self.assertEqual(self.send([],102.),0,'cleared obstacle replayed from accumulator')
    def tearDown(self):
        self.process.send_signal(signal.SIGINT)
        try:self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.log.seek(0)
        if self._outcome.result and (self._outcome.result.failures or self._outcome.result.errors):print(self.log.read()[-3000:])
        self.log.close();self.node.destroy_node();rclpy.shutdown()
if __name__=='__main__':unittest.main()
