"""Exercise real crop/downsampling/publication with delayed RGB and depth input.

Requires the onboard Open3D environment. Registration is mocked so this checks
message timing and publication independently of an actual object's appearance.
"""
import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from yolo_node.msg import Detection, DetectionArray

SOURCE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOURCE))
from pose_estimator.pose_estimation_node import PoseEstimationNode, stamp_to_sec


def cloud_at(stamp, depth):
    u, v = np.meshgrid(np.arange(64), np.arange(48))
    points = np.stack(((u-32)/60*depth, (v-24)/60*depth, np.full_like(u, depth, dtype=float)), axis=-1)
    cloud = PointCloud2(height=48, width=64, is_bigendian=False, point_step=12,
                        row_step=64*12, is_dense=True,
                        fields=[PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                                for i, name in enumerate(['x','y','z'])],
                        data=points.astype(np.float32).tobytes())
    cloud.header.stamp = stamp
    cloud.header.frame_id = 'camera_color_optical_frame'
    return cloud


class NavigationPipelineTest(unittest.TestCase):
    def test_delayed_detection_produces_one_hint_and_center_from_its_own_depth(self):
        model = SOURCE / 'config/half_block.pcd'
        rclpy.init(args=['--ros-args', '--params-file', str(SOURCE / 'config/params.yaml'),
                        '-p', f'model_path:={model}',
                        '-p', 'navigation_observations:=true', '-p', 'navigation_confidence:=0.1',
                        '-p', 'target_class:=blue block', '-p', 'bbox_smoothing_enable:=false',
                        '-p', 'publish_roi_cloud:=false', '-p', 'publish_model_marker:=false'])
        node = PoseEstimationNode()
        try:
            info = CameraInfo(width=64, height=48, k=[60.,0.,32.,0.,60.,24.,0.,0.,1.])
            info.header.frame_id = 'camera_color_optical_frame'
            node._cb_cam_info(info)
            now_ns = node.get_clock().now().nanoseconds
            exposure_stamp = rclpy.time.Time(nanoseconds=now_ns-240_000_000).to_msg()
            current_stamp = rclpy.time.Time(nanoseconds=now_ns).to_msg()
            old_cloud, current_cloud = cloud_at(exposure_stamp, .75), cloud_at(current_stamp, .95)
            node._cb_cloud(old_cloud)  # Buffered before inference has finished.
            detection = DetectionArray(detections=[Detection(x=12., y=8., width=40., height=32.,
                                                               confidence=.22, class_name='blue block')])
            detection.header = copy.deepcopy(old_cloud.header)
            node._cb_detection(detection)
            outputs, registered_depths = [], []

            def registration(scene):
                registered_depths.append(float(np.median(np.asarray(scene.points)[:,2])))
                transform = np.eye(4)
                transform[2,3] = registered_depths[-1]
                return transform, 1., 0.

            def published(header, trans, quat, frame, publisher):
                outputs.append((publisher, stamp_to_sec(header.stamp), list(trans)))

            with patch.object(node, '_register', side_effect=registration), \
                    patch.object(node, '_publish_pose_msg', side_effect=published):
                node._cb_cloud(current_cloud)
                self.assertEqual(len(registered_depths), 1)
                self.assertAlmostEqual(registered_depths[0], .75)
                for publisher in [node.pub_navigation_hint, node.pub_navigation_center, node.pub_pose]:
                    matches = [row for row in outputs if row[0] is publisher]
                    self.assertEqual(len(matches), 1)
                    self.assertAlmostEqual(matches[0][1], stamp_to_sec(exposure_stamp))
                # More depth frames without a new detector frame cannot manufacture
                # the independent observations needed to interrupt exploration.
                future_stamp = rclpy.time.Time(nanoseconds=now_ns+33_000_000).to_msg()
                node._cb_cloud(cloud_at(future_stamp, .95))
                self.assertEqual(len(registered_depths), 1)
                # A new detection produces its own observation at its own exposure.
                detection.header.stamp = future_stamp
                node._cb_detection(detection)
                node._cb_cloud(cloud_at(rclpy.time.Time(nanoseconds=now_ns+66_000_000).to_msg(), .95))
                self.assertEqual(len(registered_depths), 2)
                self.assertAlmostEqual(registered_depths[-1], .95, places=5)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
