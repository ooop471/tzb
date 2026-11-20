#!/usr/bin/env python3
"""Prepare text-free RViz geometry and visible path overlays; no robot commands."""
import copy
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException


class NavigationVisualData(Node):
    def __init__(self):
        super().__init__('state_visual_data')
        self.grasp_region = None
        self.odom = None
        self.paths = {}
        self.local_received = None
        self.history = {}
        self.history_dirty = False
        self.history_last_update = None
        self.session = None
        self.history_frame = 'odom_init'
        self.voxel_size = self.declare_parameter('history_voxel_size', 0.05).value
        self.max_voxels = self.declare_parameter('history_max_voxels', 200000).value
        if not math.isfinite(self.voxel_size) or self.voxel_size <= 0 or self.max_voxels < 1:
            raise ValueError('Invalid history voxel size or capacity')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.history_pub = self.create_publisher(OccupancyGrid, '/visualization/obstacle_history_map', latched)
        self.history_cells_pub = self.create_publisher(Marker, '/visualization/obstacle_history_cells', latched)
        self.grasp_pub = self.create_publisher(Marker, '/visualization/grasp_region', latched)
        self.body_pub = self.create_publisher(Marker, '/visualization/robot_body', latched)
        self.body_frame = self.declare_parameter('body_frame', 'base_frame').value
        footprint = json.loads(self.declare_parameter('body_footprint',
            '[[0.21,0.31],[0.21,-0.31],[-0.59,-0.31],[-0.59,0.31]]').value)
        if len(footprint) < 3 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in footprint):
            raise ValueError('Invalid body footprint')
        body = Marker()
        body.header.frame_id = self.body_frame
        body.frame_locked = True
        body.ns, body.id, body.type = 'configured_robot_body', 0, Marker.LINE_STRIP
        body.pose.orientation.w = 1.0
        body.scale.x = 0.015
        body.color.r = body.color.g = body.color.b = body.color.a = 1.0
        body.points = [Point(x=float(x), y=float(y), z=0.025) for x, y in footprint]
        body.points.append(copy.deepcopy(body.points[0]))
        self.body_pub.publish(body)
        self.geometry = self.create_publisher(MarkerArray, '/visualization/corridor_geometry', latched)
        self.subscriptions_ = [
            self.create_subscription(Odometry, '/odometry', self.on_odometry, qos_profile_sensor_data),
            self.create_subscription(MarkerArray, '/explore/corridor_selection', self.on_markers, latched),
        ]
        self.path_publishers = {
            '/trajectories': self.create_publisher(Path, '/visualization/local_trajectory', latched),
        }
        self.subscriptions_ += [
            self.create_subscription(MarkerArray, '/trajectories', self.on_trajectories, 10),
            self.create_subscription(PointCloud2, '/terrain_map', self.on_obstacles, qos_profile_sensor_data),
            self.create_subscription(String, '/mission/state', self.on_mission, latched),
        ]
        self.clear_service = self.create_service(Trigger, '/visualization/clear_obstacle_history', self.clear_history)
        for topic, output, qos in [
            ('/plan', '/visualization/nav2_plan', 10),
            ('/exploration/executed_path', '/visualization/controller_path', latched),
        ]:
            self.path_publishers[topic] = self.create_publisher(Path, output, latched)
            self.subscriptions_.append(self.create_subscription(
                Path, topic, lambda msg, key=topic: self.on_path(key, msg), qos))
        self.timer = self.create_timer(0.5, self.refresh_paths)

    def on_odometry(self, msg):
        if self.odom is not None:
            old, new = self.odom.pose.pose.position, msg.pose.pose.position
            if (msg.header.frame_id != self.odom.header.frame_id or
                    math.hypot(new.x-old.x, new.y-old.y) > 2.0):
                self.reset_history()
        self.odom = msg

    def on_mission(self, msg):
        try:
            data = json.loads(msg.data)
            session = data.get('session')
            self.grasp_region = data.get('grasp_region')
            self.publish_grasp_region()
        except (ValueError, AttributeError):
            return
        if session and self.session is not None and session != self.session:
            self.reset_history()
        if session:
            self.session = session

    def publish_grasp_region(self):
        region = self.grasp_region
        if not region or region.get('shape') != 'annular_sector':
            return
        marker = Marker()
        # Body-relative geometry must follow the latest TF at every RViz frame,
        # rather than remain transformed at the last mission-status timestamp.
        marker.header = Header(frame_id=region['frame'])
        marker.frame_locked = True
        marker.ns = 'front_edge_grasp_sector'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.012
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.65, 0.05, 1.0
        angles = np.linspace(-region['half_angle'], region['half_angle'], 50)
        for radius, sequence in [(region['outer_radius'], angles),
                                 (region['inner_radius'], angles[::-1])]:
            for a in sequence:
                marker.points.append(Point(x=region['front_offset']+radius*math.cos(a),
                                           y=radius*math.sin(a), z=0.02))
        marker.points.append(copy.deepcopy(marker.points[0]))
        self.grasp_pub.publish(marker)

    def reset_history(self):
        self.history.clear()
        self.history_dirty = True
        self.history_last_update = None

    def clear_history(self, request, response):
        self.reset_history()
        self.publish_history()
        response.success = True
        response.message = 'Historical obstacle display cleared'
        return response

    def on_trajectories(self, msg):
        # MPPI publishes candidates and the optimized trajectory in one array.
        # Display only the optimizer output, ordered along its prediction horizon.
        optimal = sorted((m for m in msg.markers if m.ns == 'Optimal Trajectory' and
                          m.action == Marker.ADD), key=lambda m: m.id)
        if not optimal:
            return
        path = Path()
        path.header = copy.deepcopy(optimal[0].header)
        if path.header.stamp.sec == 0 and path.header.stamp.nanosec == 0:
            path.header.stamp = self.get_clock().now().to_msg()
        for marker in optimal:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose = copy.deepcopy(marker.pose)
            path.poses.append(pose)
        self.local_received = time.monotonic()
        self.on_path('/trajectories', path)

    def on_obstacles(self, msg):
        now = time.monotonic()
        if self.history_last_update is not None and now-self.history_last_update < 0.5:
            return
        self.history_last_update = now
        try:
            points = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
            points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
            points = points[np.isfinite(points).all(axis=1)]
            if msg.header.frame_id != self.history_frame:
                t = self.tf.lookup_transform(self.history_frame, msg.header.frame_id, Time()).transform
                q = t.rotation
                rotation = np.array([
                    [1-2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.z*q.w), 2*(q.x*q.z+q.y*q.w)],
                    [2*(q.x*q.y+q.z*q.w), 1-2*(q.x*q.x+q.z*q.z), 2*(q.y*q.z-q.x*q.w)],
                    [2*(q.x*q.z-q.y*q.w), 2*(q.y*q.z+q.x*q.w), 1-2*(q.x*q.x+q.y*q.y)]])
                points = points @ rotation.T + np.array([t.translation.x, t.translation.y, t.translation.z])
            # Project into world XY: observations at different heights occupy
            # the same historical 2D cell rather than forming a 3D cloud.
            keys = np.floor(points[:, :2]/self.voxel_size).astype(np.int64)
            keys = np.unique(keys, axis=0)
            for key in keys:
                self.history[tuple(key)] = True
            while len(self.history) > self.max_voxels:
                self.history.pop(next(iter(self.history)))
            self.history_dirty = self.history_dirty or bool(len(keys))
        except (TransformException, ValueError, AssertionError):
            return

    def publish_history(self):
        if not self.history_dirty:
            return
        header = Header(frame_id=self.history_frame, stamp=self.get_clock().now().to_msg())
        grid = OccupancyGrid()
        grid.header = header
        grid.info.origin.orientation.w = 1.0
        grid.info.map_load_time = header.stamp
        cells = Marker()
        cells.header = header
        cells.ns = 'historical_obstacles_2d'
        cells.id = 0
        cells.type = Marker.CUBE_LIST
        cells.pose.orientation.w = 1.0
        cells.pose.position.z = 0.03
        cells.scale.z = 0.01
        cells.color.r, cells.color.g, cells.color.b, cells.color.a = 0.10, 1.0, 0.20, 0.70
        resolution = self.voxel_size
        if self.history:
            keys = np.asarray(list(self.history), dtype=np.int64)
            stride = 1
            while True:
                projected = np.unique(np.floor_divide(keys, stride), axis=0)
                lower, upper = projected.min(axis=0), projected.max(axis=0)
                width, height = (upper-lower+1).tolist()
                if width*height <= 4000000:
                    break
                stride *= 2
            resolution *= stride
            grid.info.width, grid.info.height = width, height
            grid.info.origin.position.x = float(lower[0]*resolution)
            grid.info.origin.position.y = float(lower[1]*resolution)
            data = np.full((height, width), -1, dtype=np.int8)
            indices = projected-lower
            data[indices[:, 1], indices[:, 0]] = 100
            grid.data = data.reshape(-1).tolist()
            cells.action = Marker.ADD
            cells.points = [Point(x=float((x+.5)*resolution), y=float((y+.5)*resolution), z=0.)
                            for x, y in projected]
        else:
            grid.info.width = grid.info.height = 1
            grid.data = [-1]
            cells.action = Marker.DELETE
        grid.info.resolution = resolution
        cells.scale.x = cells.scale.y = resolution
        self.history_pub.publish(grid)
        self.history_cells_pub.publish(cells)
        self.history_dirty = False

    def on_markers(self, msg):
        msg.markers = [m for m in msg.markers if m.type != Marker.TEXT_VIEW_FACING]
        self.geometry.publish(msg)

    def on_path(self, topic, msg):
        self.paths[topic] = msg
        self.render_path(topic, msg)

    def robot_height(self, frame):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        if frame == self.odom.header.frame_id:
            return p.z
        try:
            transform = self.tf.lookup_transform(frame, self.odom.header.frame_id, Time())
            q = transform.transform.rotation
            return (2 * (q.x*q.z - q.y*q.w) * p.x +
                    2 * (q.y*q.z + q.x*q.w) * p.y +
                    (1 - 2 * (q.x*q.x + q.y*q.y)) * p.z + transform.transform.translation.z)
        except TransformException:
            return None

    def render_path(self, topic, msg):
        display = copy.deepcopy(msg)
        height = self.robot_height(msg.header.frame_id)
        for pose in display.poses:
            pose.pose.position.z = (height if height is not None else pose.pose.position.z) + 0.10
        self.path_publishers[topic].publish(display)

    def refresh_paths(self):
        if self.local_received is not None and time.monotonic()-self.local_received > 1.0:
            stale = self.paths.pop('/trajectories', None)
            if stale is not None:
                empty = Path()
                empty.header = stale.header
                self.path_publishers['/trajectories'].publish(empty)
            self.local_received = None
        for topic, msg in self.paths.items():
            self.render_path(topic, msg)
        self.publish_history()


def main():
    rclpy.init()
    node = NavigationVisualData()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
