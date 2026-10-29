"""Live capture node for eye-in-hand calibration.

Shows the camera stream, detects the calibration board and stores one sample
(T_base_ee, T_cam_target) per SPACE press.  Every node start throws the previous
capture away, so moving the camera only requires replaying this node and then
running ``offline_calibrate``.
"""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from hand_eye_calib.board import BoardConfig, BoardDetection, BoardDetector
from hand_eye_calib.geometry import make_transform, pose_delta_deg_mm, quat_to_matrix
from hand_eye_calib.session import CalibrationSession

WINDOW_NAME = 'hand_eye_calib'
CAPTURE_KEYS = (ord(' '), 13)          # SPACE or ENTER
QUIT_KEYS = (ord('q'), 27)             # q or ESC
KEY_WAIT_MS = 1
DETECTION_PERIOD_S = 0.1


class HandEyeCalibNode(Node):
    def __init__(self) -> None:
        super().__init__('hand_eye_calib')

        self.image_topic = self.declare_parameter(
            'image_topic', '/camera/color/image_raw').value
        self.camera_info_topic = self.declare_parameter(
            'camera_info_topic', '/camera/color/camera_info').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.ee_frame = self.declare_parameter('ee_frame', 'tcp_link').value
        self.session_root = str(self.declare_parameter('session_root', '').value)
        self.min_pose_delta_deg = float(
            self.declare_parameter('min_pose_delta_deg', 5.0).value)
        self.min_pose_delta_m = float(
            self.declare_parameter('min_pose_delta_m', 0.01).value)

        self.board_config = BoardConfig(
            board_type=self.declare_parameter('board_type', 'chessboard').value,
            squares_x=int(self.declare_parameter('squares_x', 8).value),
            squares_y=int(self.declare_parameter('squares_y', 11).value),
            square_length=float(self.declare_parameter('square_length', 0.03).value),
            marker_length=float(self.declare_parameter('marker_length', 0.0225).value),
            dictionary=str(self.declare_parameter(
                'dictionary', 'DICT_5X5_100').value),
            min_corners=int(self.declare_parameter('min_corners', 6).value),
            legacy_pattern=bool(self.declare_parameter(
                'legacy_pattern', False).value),
        )
        self.detector = BoardDetector(self.board_config)
        self._layout_warned = False

        self.bridge = CvBridge()
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.camera_frame = ''
        self._frame: np.ndarray | None = None
        self._frame_stamp = None
        self._frame_token = 0
        self._detection: BoardDetection | None = None
        self._detection_token = -1
        self._last_detect_time = 0.0
        self._last_tf_check = 0.0
        self._ee_ready = False
        self._status = ''
        self._quit = False

        self.session = CalibrationSession(
            root=self.session_root or None,
            board=self.board_config,
            frames={'base': self.base_frame, 'ee': self.ee_frame, 'camera': ''},
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, self.image_topic, self._on_image, 1)
        self.create_subscription(CameraInfo, self.camera_info_topic,
                                 self._on_camera_info, 1)

        self.get_logger().info(
            f'board: {self.board_config.board_type} '
            f'{self.board_config.squares_x}x{self.board_config.squares_y}'
            + (' squares' if self.board_config.is_charuco else ' inner corners')
            + f', square={self.board_config.square_length * 1000.0:.1f} mm')
        self.get_logger().info(
            f'images: {self.image_topic}   intrinsics: {self.camera_info_topic}')
        self.get_logger().info(
            f'poses: {self.base_frame} -> {self.ee_frame} (tf2)')
        self.get_logger().info(
            f'session: {self.session.directory} (previous data cleared)')
        self.get_logger().info(
            'SPACE capture sample   q/ESC quit')

    # -- subscriptions -----------------------------------------------------
    def _on_image(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # cv_bridge raises various conversion errors
            self.get_logger().warn(f'cannot convert image: {exc}')
            return
        self._frame = frame
        self._frame_stamp = msg.header.stamp
        self._frame_token += 1
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id
            self.session.frames['camera'] = self.camera_frame

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if not any(value != 0.0 for value in msg.k):
            return
        self.camera_matrix = np.asarray(msg.k, dtype=float).reshape(3, 3)
        self.dist_coeffs = np.asarray(msg.d, dtype=float).reshape(-1)
        if msg.header.frame_id:
            self.camera_frame = msg.header.frame_id
            self.session.frames['camera'] = self.camera_frame
        camera_info = {
            'frame_id': msg.header.frame_id,
            'width': int(msg.width),
            'height': int(msg.height),
            'distortion_model': msg.distortion_model,
            'k': [float(v) for v in msg.k],
            'd': [float(v) for v in msg.d],
        }
        if camera_info != self.session.camera_info:
            self.session.set_camera_info(camera_info)

    # -- tf ----------------------------------------------------------------
    def _lookup(self, parent: str, child: str, stamp, timeout: float = 0.1):
        attempts = []
        if stamp is not None:
            attempts.append(Time.from_msg(stamp))
        attempts.append(Time())
        last_error = None
        for target in attempts:
            try:
                message = self.tf_buffer.lookup_transform(parent, child, target,
                                                          Duration(seconds=timeout))
            except tf2_ros.TransformException as exc:
                last_error = exc
                continue
            rotation = message.transform.rotation
            translation = message.transform.translation
            transform = make_transform(
                quat_to_matrix([rotation.x, rotation.y, rotation.z, rotation.w]),
                [translation.x, translation.y, translation.z])
            age = None
            if stamp is not None:
                age = (Time.from_msg(stamp)
                       - Time.from_msg(message.header.stamp)).nanoseconds * 1e-9
            return transform, age
        if last_error is not None:
            self.get_logger().debug(
                f'TF {parent}->{child} unavailable: {last_error}')
        return None, None

    # -- capture -----------------------------------------------------------
    def _detect_current(self, force: bool = False) -> BoardDetection | None:
        if self._frame is None:
            return None
        if not force and self._detection_token == self._frame_token:
            return self._detection
        if self.camera_matrix is None:
            return None
        try:
            detection = self.detector.detect(self._frame, self.camera_matrix,
                                             self.dist_coeffs)
        except cv2.error as exc:
            self.get_logger().warn(f'board detection failed: {exc}')
            detection = None
        self._detection = detection
        self._detection_token = self._frame_token
        self._last_detect_time = time.monotonic()
        if (detection is not None and not self._layout_warned
                and not self.board_config.is_charuco
                and self.detector.uses_swapped_pattern):
            self._layout_warned = True
            self.get_logger().warn(
                f'chessboard detected as {self.detector.active_pattern} inner corners, '
                f'configured {self.board_config.squares_x}x'
                f'{self.board_config.squares_y}; using the detected layout')
        return detection

    def _capture(self) -> None:
        if self._frame is None:
            self._status = 'no image received yet'
            self.get_logger().warn(self._status)
            return
        if self.camera_matrix is None:
            self._status = 'waiting for camera_info (intrinsics)'
            self.get_logger().warn(self._status)
            return

        detection = self._detect_current(force=True)
        if detection is None:
            self._status = 'board not detected - adjust the view'
            self.get_logger().warn(self._status)
            return

        base_ee, age = self._lookup(self.base_frame, self.ee_frame, self._frame_stamp)
        if base_ee is None:
            self._status = (f'TF {self.base_frame}->{self.ee_frame} missing '
                            '(is robot_state_publisher running?)')
            self.get_logger().warn(self._status)
            return
        if age is not None and abs(age) > 0.5:
            self.get_logger().warn(
                f'end-effector TF is {age:.2f} s away from the image stamp; '
                'capture with the arm standing still')

        previous = self.session.last_base_ee()
        if previous is not None:
            angle_deg, distance_mm = pose_delta_deg_mm(previous, base_ee)
            if (angle_deg < self.min_pose_delta_deg
                    and distance_mm < self.min_pose_delta_m * 1000.0):
                self._status = (f'pose too close to the last sample '
                                f'({angle_deg:.1f} deg, {distance_mm:.1f} mm)')
                self.get_logger().warn(self._status + ' - move the arm further')
                return

        base_camera = None
        if self.camera_frame:
            self.session.frames['camera'] = self.camera_frame
            base_camera, _ = self._lookup(self.base_frame, self.camera_frame,
                                          self._frame_stamp)

        index = self.session.add_sample(
            image_bgr=self._frame,
            stamp=self._frame_stamp,
            base_ee=base_ee,
            detection=detection,
            base_camera=base_camera,
        )
        self._status = (f'sample {index} saved '
                        f'({detection.corner_count} corners, '
                        f'{detection.reproj_error_px:.2f} px)')
        self.get_logger().info(self._status)

    # -- ui ----------------------------------------------------------------
    def _render(self) -> bool:
        if self._frame is None:
            canvas = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(canvas, 'waiting for camera images...', (30, 180),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        else:
            if (time.monotonic() - self._last_detect_time > DETECTION_PERIOD_S
                    or self._detection_token != self._frame_token):
                self._detect_current()

            detection = self._detection if self._detection_token == self._frame_token else None
            if detection is not None and self.camera_matrix is not None:
                canvas = self.detector.draw(self._frame, detection,
                                            self.camera_matrix, self.dist_coeffs)
            else:
                canvas = self._frame.copy()

            if detection is not None:
                board_state = (f'board OK  corners={detection.corner_count}  '
                               f'reproj={detection.reproj_error_px:.2f}px')
                board_color = (0, 220, 0)
            else:
                board_state = 'board NOT detected'
                board_color = (0, 0, 255)

            if time.monotonic() - self._last_tf_check > 0.5:
                ee_pose, _ = self._lookup(self.base_frame, self.ee_frame, None,
                                          timeout=0.0)
                self._ee_ready = ee_pose is not None
                self._last_tf_check = time.monotonic()
            ee_state = (f'tf {self.base_frame}->{self.ee_frame} OK'
                        if self._ee_ready else
                        f'tf {self.base_frame}->{self.ee_frame} MISSING')
            ee_color = (0, 220, 0) if self._ee_ready else (0, 0, 255)

            lines = [
                ('HAND-EYE CALIBRATION  (eye-in-hand)', (255, 255, 255)),
                (f'{self.board_config.board_type} '
                 f'{self.detector.active_pattern[0]}x{self.detector.active_pattern[1]}'
                 + (' squares' if self.board_config.is_charuco else ' corners')
                 + f'  square={self.board_config.square_length * 1000.0:.1f}mm',
                 (200, 200, 200)),
                (board_state, board_color),
                (ee_state, ee_color),
                (f'samples: {len(self.session)}   '
                 f'SPACE capture   q/ESC quit', (0, 255, 255)),
            ]
            if self._status:
                lines.append((self._status, (0, 200, 255)))

            header = 22 * len(lines) + 12
            overlay = canvas.copy()
            cv2.rectangle(overlay, (0, 0), (canvas.shape[1], header), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, canvas, 0.4, 0, canvas)
            for row, (text, color) in enumerate(lines):
                cv2.putText(canvas, text, (8, 20 + 22 * row),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        try:
            cv2.imshow(WINDOW_NAME, canvas)
        except cv2.error as exc:
            self.get_logger().error(
                f'cannot open a display window ({exc}); '
                'run this node on a machine with a desktop session')
            return False

        key = cv2.waitKey(KEY_WAIT_MS) & 0xFF
        if key in QUIT_KEYS:
            return False
        if key in CAPTURE_KEYS:
            self._capture()

        try:
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                return False
        except cv2.error:
            return False
        return True

    # -- lifecycle ---------------------------------------------------------
    def run(self) -> None:
        while rclpy.ok() and not self._quit:
            rclpy.spin_once(self, timeout_sec=0.02)
            if not self._render():
                break
        self.finish()

    def finish(self) -> None:
        cv2.destroyAllWindows()
        if len(self.session) > 0:
            self.session.finalize()
            self.get_logger().info(
                f'{len(self.session)} samples kept in {self.session.directory}')
            if len(self.session) < 5:
                self.get_logger().warn(
                    'fewer than 5 samples captured; expect a poor calibration')
            self.get_logger().info(
                'next step: ros2 run hand_eye_calib offline_calibrate')
        else:
            self.session.discard_if_empty()
            self.get_logger().warn('no samples captured, nothing to calibrate')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = HandEyeCalibNode()
        node.run()
    except KeyboardInterrupt:
        if node is not None:
            node.finish()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
