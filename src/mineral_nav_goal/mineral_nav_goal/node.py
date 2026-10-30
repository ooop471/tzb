import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, PointCloud2
from tf2_ros import Buffer, TransformListener
from yolo_node.msg import DetectionArray

from mineral_nav_goal.geometry import fits_wheel_corridor, grasp_goal


def transform_point(point, transform):
    rotation = transform.transform.rotation
    translation = transform.transform.translation
    norm = math.sqrt(sum(value * value for value in (
        rotation.x, rotation.y, rotation.z, rotation.w)))
    if norm < 1e-9:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = (rotation.x / norm, rotation.y / norm,
                  rotation.z / norm, rotation.w / norm)
    matrix = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return matrix @ point + np.array([translation.x, translation.y, translation.z])


class MineralNavGoal(Node):
    def __init__(self):
        super().__init__('mineral_nav_goal')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.target_class = param('target_class', 'white block')
        self.base_frame = param('base_frame', 'base_link')
        self.map_frame = param('map_frame', 'map')
        self.standoff_m = float(param('standoff_m', 0.30))
        self.min_standoff_m = float(param('min_standoff_m', 0.20))
        self.max_standoff_m = float(param('max_standoff_m', 0.40))
        self.inner_wheel_width_m = float(param('inner_wheel_width_m', 0.30))
        self.side_clearance_m = float(param('side_clearance_m', 0.03))
        self.min_confidence = float(param('min_confidence', 0.40))
        self.max_cloud_age_sec = float(param('max_cloud_age_sec', 0.50))
        self.max_sync_error_sec = float(param('max_sync_error_sec', 0.25))
        self.min_stable_detections = int(param('min_stable_detections', 3))
        self.send_nav_goal = bool(param('send_nav_goal', False))
        self.action_name = param('action_name', 'navigate_to_pose')
        if not (0 < self.min_standoff_m <= self.standoff_m <= self.max_standoff_m):
            raise ValueError('standoff_m must be within the configured positive range')
        if self.inner_wheel_width_m <= 2 * self.side_clearance_m:
            raise ValueError('Wheel corridor must exceed the side clearances')

        sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            DetectionArray, param('detection_topic', '/yolo/detections'),
            self.on_detection, 10)
        self.create_subscription(
            PointCloud2, param('pointcloud_topic', '/camera/depth/color/points'),
            self.on_cloud, sensor_qos)
        self.create_subscription(
            CameraInfo, param('camera_info_topic', '/camera/color/camera_info'),
            self.on_camera_info, 10)
        self.goal_pub = self.create_publisher(
            PoseStamped, param('goal_topic', '/mineral_nav_goal/goal_pose'), 10)
        self.target_pub = self.create_publisher(
            PointStamped, param('target_topic', '/mineral_nav_goal/target_point'), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cloud = None
        self.cloud_received = 0.0
        self.camera_info = None
        self.streak = 0
        self.last_stamp = None
        self.last_detection_time = 0.0
        self.goal_submitted = False
        self.action_client = None
        if self.send_nav_goal:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            self.nav_action_type = NavigateToPose
            self.action_client = ActionClient(self, NavigateToPose, self.action_name)
        self.get_logger().info(
            f'Publishing candidate grasp goals in {self.map_frame}; '
            f'base={self.base_frame}, Nav2 action enabled={self.send_nav_goal}')

    def on_cloud(self, msg):
        self.cloud = msg
        self.cloud_received = time.monotonic()

    def on_camera_info(self, msg):
        self.camera_info = msg

    @staticmethod
    def stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def on_detection(self, msg):
        now = time.monotonic()
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        matching = [item for item in msg.detections
                    if item.class_name == self.target_class
                    and item.confidence >= self.min_confidence]
        if not matching:
            self.streak = 0
            return
        self.streak = self.streak + 1 if now - self.last_detection_time < 0.6 else 1
        self.last_detection_time = now
        if self.streak < self.min_stable_detections or self.goal_submitted:
            return
        detection = max(matching, key=lambda item: item.confidence)
        goal = self.compute_goal(msg, detection)
        if goal is None:
            return
        pose, point = goal
        self.target_pub.publish(point)
        self.goal_pub.publish(pose)
        self.goal_submitted = True
        self.get_logger().info(
            f'Grasp stop candidate: x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}, frame={pose.header.frame_id}')
        if self.action_client is not None:
            self.submit_nav_goal(pose)

    def compute_goal(self, detection_msg, detection):
        cloud = self.cloud
        info = self.camera_info
        if cloud is None or info is None:
            return None
        if time.monotonic() - self.cloud_received > self.max_cloud_age_sec:
            return None
        if abs(self.stamp_seconds(detection_msg.header.stamp)
               - self.stamp_seconds(cloud.header.stamp)) > self.max_sync_error_sec:
            return None
        if (cloud.height <= 1 or cloud.width <= 1 or info.width <= 0
                or info.height <= 0 or info.k[0] <= 0):
            return None
        offsets = {field.name: field.offset for field in cloud.fields}
        if not all(axis in offsets and offsets[axis] + 4 <= cloud.point_step
                   for axis in ('x', 'y', 'z')):
            return None
        if not (math.isfinite(detection.x) and math.isfinite(detection.y)
                and math.isfinite(detection.width) and math.isfinite(detection.height)
                and detection.width > 0 and detection.height > 0):
            return None
        center_x = (detection.x + detection.width / 2) * cloud.width / info.width
        center_y = (detection.y + detection.height / 2) * cloud.height / info.height
        pixel_x = int(round(center_x))
        pixel_y = int(round(center_y))
        if not (5 <= pixel_x < cloud.width - 5 and 5 <= pixel_y < cloud.height - 5):
            return None
        raw = np.frombuffer(cloud.data, dtype=np.uint8)
        if raw.size < cloud.row_step * cloud.height or cloud.point_step < 12:
            return None
        dtype = np.dtype('>f4' if cloud.is_bigendian else '<f4')
        points = []
        for row in range(pixel_y - 5, pixel_y + 6):
            for col in range(pixel_x - 5, pixel_x + 6):
                start = row * cloud.row_step + col * cloud.point_step
                xyz = [np.frombuffer(raw[start + offsets[axis]:start + offsets[axis] + 4],
                                     dtype=dtype, count=1)[0]
                       for axis in ('x', 'y', 'z')]
                if np.isfinite(xyz).all() and xyz[2] > 0.05:
                    points.append(xyz)
        if len(points) < 10:
            return None
        points = np.asarray(points)
        depth = float(np.median(points[:, 2]))
        if float(np.median(np.abs(points[:, 2] - depth))) > 0.03:
            return None
        width_m = float(detection.width) * depth / float(info.k[0])
        if not fits_wheel_corridor(
                width_m, self.inner_wheel_width_m, self.side_clearance_m):
            self.get_logger().warn(f'Target too wide for wheel corridor: {width_m:.3f} m')
            return None
        try:
            sample_time = Time.from_msg(cloud.header.stamp)
            base_from_cloud = self.tf_buffer.lookup_transform(
                self.base_frame, cloud.header.frame_id, sample_time,
                timeout=Duration(seconds=0.05))
            map_from_base = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, sample_time,
                timeout=Duration(seconds=0.05))
            target_base = transform_point(np.median(points, axis=0), base_from_cloud)
            target_map = transform_point(target_base, map_from_base)
        except Exception as error:
            self.get_logger().warn(f'Cannot locate target in {self.map_frame}: {error}')
            return None
        if target_base[0] <= 0 or not np.isfinite(target_map).all():
            return None
        robot_x = float(map_from_base.transform.translation.x)
        robot_y = float(map_from_base.transform.translation.y)
        try:
            goal_x, goal_y, heading = grasp_goal(
                robot_x, robot_y, float(target_map[0]), float(target_map[1]),
                self.standoff_m)
        except ValueError:
            return None
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.z = math.sin(heading / 2)
        goal.pose.orientation.w = math.cos(heading / 2)
        target = PointStamped()
        target.header.frame_id = self.map_frame
        target.header.stamp = goal.header.stamp
        target.point.x = float(target_map[0])
        target.point.y = float(target_map[1])
        target.point.z = float(target_map[2])
        return goal, target

    def submit_nav_goal(self, pose):
        if not self.action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().error('Nav2 action server unavailable; candidate topic was published')
            return
        goal = self.nav_action_type.Goal()
        goal.pose = pose
        self.action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = MineralNavGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
