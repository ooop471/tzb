#!/usr/bin/env python3
"""把识别到的目标物体发布到底盘坐标系 base_frame。

完整链路（与 tzb_catch 内部一致）：

    /pose_estimator/object_pose        相机光学系下的物体位姿
        ├─ TF: base_link <- tzb_observe_camera_link  （冻结相机帧 × 标定矩阵）
        ├─ 修正: v = R(target_correction_rpy) · v + target_correction_xyz
        ├─ 限幅: v.z = max(v.z, target_min_z)
        └─ 发布: /mission/object_pose_base (PoseStamped)
                 /mission/target_in_base   (TargetInBase)

修正参数默认从 lite_motion_planner 的 tzb_planner.yaml 读取，
保证和机械臂用同一份配置，不用改两处。
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

from geometry_msgs.msg import PoseStamped
from tzb_target_bridge.msg import TargetInBase

DEFAULT_PLANNER_CONFIG = (
    '/home/tzb/tzb_ws/src/lite_motion_planner/config/tzb_planner.yaml')


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s,
                (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return ((R[2, 1] - R[1, 2]) / s, 0.25 * s,
                (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                0.25 * s, (R[1, 2] + R[2, 1]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
    return ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s, 0.25 * s)


def rpy_to_mat(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class TargetBridge(Node):
    def __init__(self):
        super().__init__('tzb_target_bridge')

        self.pose_topic = self.declare_parameter(
            'pose_topic', '/pose_estimator/object_pose').value
        self.base_link_frame = self.declare_parameter(
            'base_link_frame', 'base_link').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_frame').value
        self.pose_frame = self.declare_parameter(
            'pose_frame', 'tzb_observe_camera_link').value
        self.pose_out_topic = self.declare_parameter(
            'pose_out_topic', '/mission/object_pose_base').value
        self.target_out_topic = self.declare_parameter(
            'target_out_topic', '/mission/target_in_base').value
        self.planner_config_file = self.declare_parameter(
            'planner_config_file', DEFAULT_PLANNER_CONFIG).value
        self.log_throttle_sec = float(self.declare_parameter(
            'log_throttle_sec', 2.0).value)

        corr_xyz, corr_rpy_deg, corr_min_z = self._load_correction()
        self.corr_R = rpy_to_mat(*[math.radians(a) for a in corr_rpy_deg])
        self.corr_t = np.array(corr_xyz, dtype=float)
        self.corr_min_z = corr_min_z

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_out_topic, 10)
        self.target_pub = self.create_publisher(TargetInBase, self.target_out_topic, 10)
        self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, 10)

        self.get_logger().info(
            'tzb_target_bridge ready: %s -> %s (%s) + %s   pose_frame=%s'
            % (self.pose_topic, self.pose_out_topic, self.base_frame,
               self.target_out_topic, self.pose_frame or '<message frame_id>'))

    def _load_correction(self):
        """从 planner 的 YAML 读修正参数，保证和机械臂一致。"""
        xyz = [0.0, 0.0, 0.0]
        rpy_deg = [0.0, 0.0, 0.0]
        min_z = -1.0
        try:
            import yaml
            with open(self.planner_config_file, encoding='utf-8') as handle:
                data = yaml.safe_load(handle)
            params = {}
            for value in (data or {}).values():
                if isinstance(value, dict) and 'ros__parameters' in value:
                    params.update(value['ros__parameters'])
            xyz = [float(v) for v in params.get('tzb.target_correction_xyz', xyz)]
            rpy_deg = [float(v) for v in
                       params.get('tzb.target_correction_rpy_deg', rpy_deg)]
            min_z = float(params.get('tzb.target_min_z', min_z))
            self.get_logger().info(
                '修正参数(来自 %s): xyz=%s rpy_deg=%s min_z=%.3f'
                % (self.planner_config_file, xyz, rpy_deg, min_z))
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().error(
                '读取 %s 失败(%s)，将使用零修正，结果会与机械臂不一致！'
                % (self.planner_config_file, exc))
        return xyz, rpy_deg, min_z

    def on_pose(self, msg: PoseStamped):
        source_frame = self.pose_frame or msg.header.frame_id
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

        # 与 tzb_catch::applyTargetCorrection 完全一致
        v = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
        v = self.corr_R @ v + self.corr_t
        if v[2] < self.corr_min_z:
            v[2] = self.corr_min_z

        robot_R = quat_to_mat(pose.orientation.x, pose.orientation.y,
                              pose.orientation.z, pose.orientation.w)
        out_R = self.corr_R @ robot_R
        qx, qy, qz, qw = mat_to_quat(out_R)

        stamped = PoseStamped()
        stamped.header.stamp = msg.header.stamp
        stamped.header.frame_id = self.base_frame
        stamped.pose.position.x = float(v[0])
        stamped.pose.position.y = float(v[1])
        stamped.pose.position.z = float(v[2])
        stamped.pose.orientation.x = qx
        stamped.pose.orientation.y = qy
        stamped.pose.orientation.z = qz
        stamped.pose.orientation.w = qw
        self.pose_pub.publish(stamped)

        yaw = math.atan2(out_R[1, 0], out_R[0, 0])
        distance = math.hypot(v[0], v[1])

        target = TargetInBase()
        target.header = stamped.header
        target.x = float(v[0])
        target.y = float(v[1])
        target.z = float(v[2])
        target.yaw_rad = yaw
        target.distance_m = distance
        target.bearing_rad = math.atan2(v[1], v[0])
        self.target_pub.publish(target)

        self.get_logger().info(
            '目标(%s): x=%.3f y=%.3f z=%.3f 距离=%.3f m 方位=%.1f° yaw=%.1f°'
            % (self.base_frame, v[0], v[1], v[2], distance,
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
