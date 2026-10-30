#!/usr/bin/env python3
"""把识别到的目标物体发布到底盘坐标系 base_frame。

输入：/pose_estimator/object_pose      （camera_color_optical_frame，物体在相机系下的位姿）
输出：/mission/object_pose_base        （geometry_msgs/PoseStamped，base_frame）
      /mission/target_in_base          （tzb_target_bridge/TargetInBase，带距离/方位角）

base_link 与 base_frame 视为同一坐标系，所以只做一次 TF：光学系 -> base_link，
然后按 base_frame 命名发布。
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from geometry_msgs.msg import PoseStamped
from tzb_target_bridge.msg import TargetInBase


class TargetBridge(Node):
    def __init__(self):
        super().__init__('tzb_target_bridge')

        self.pose_topic = self.declare_parameter(
            'pose_topic', '/pose_estimator/object_pose').value
        self.base_link_frame = self.declare_parameter(
            'base_link_frame', 'base_link').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_frame').value
        self.pose_out_topic = self.declare_parameter(
            'pose_out_topic', '/mission/object_pose_base').value
        self.target_out_topic = self.declare_parameter(
            'target_out_topic', '/mission/target_in_base').value
        self.log_throttle_sec = float(self.declare_parameter(
            'log_throttle_sec', 2.0).value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_out_topic, 10)
        self.target_pub = self.create_publisher(TargetInBase, self.target_out_topic, 10)
        self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, 10)

        self.get_logger().info(
            'tzb_target_bridge ready: %s -> %s (%s) + %s'
            % (self.pose_topic, self.pose_out_topic, self.base_frame, self.target_out_topic))

    def on_pose(self, msg: PoseStamped):
        source_frame = msg.header.frame_id
        if not source_frame:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_link_frame, source_frame, Time())
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(
                'TF %s <- %s 不可用: %s' % (self.base_link_frame, source_frame, exc),
                throttle_duration_sec=2.0)
            return

        pose = do_transform_pose(msg.pose, tf)

        stamped = PoseStamped()
        stamped.header.stamp = msg.header.stamp
        stamped.header.frame_id = self.base_frame
        stamped.pose = pose
        self.pose_pub.publish(stamped)

        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        target = TargetInBase()
        target.header = stamped.header
        target.x = x
        target.y = y
        target.z = z
        target.yaw_rad = yaw
        target.distance_m = math.hypot(x, y)
        target.bearing_rad = math.atan2(y, x)
        self.target_pub.publish(target)

        self.get_logger().info(
            '目标(%s): x=%.3f y=%.3f z=%.3f 距离=%.3f m 方位=%.1f° yaw=%.1f°'
            % (self.base_frame, x, y, z, target.distance_m,
               math.degrees(target.bearing_rad), math.degrees(yaw)),
            throttle_duration_sec=self.log_throttle_sec)


def main():
    rclpy.init()
    node = TargetBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
