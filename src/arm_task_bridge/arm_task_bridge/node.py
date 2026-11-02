#!/usr/bin/env python3
"""目标位置 -> 机械臂抓取任务 的决策桥。

订阅
    /mission/target_in_base   物体在 base_frame 下的位置（tzb_target_bridge 发布）
    /odometry                 用来判断车是否停稳（可选，require_odometry:=false 时忽略）
    /tzb_catch/result         机械臂抓取结果（Int8: 1 成功 / -1 失败）

发布
    /tzb_catch/start          Int8=1，触发一次抓取
    /arm_task/state           String，当前状态（IDLE/WAITING_SETTLE/READY/GRASPING/SUCCESS/FAILED）
    /arm_task/result          Int8，最终结果（1 成功 / -1 放弃），每个任务只发一次

测试时可以完全不用状态机，直接 echo 上面两个话题看行为。
"""

import math
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger
from tzb_target_bridge.msg import TargetInBase

IDLE = 'IDLE'
WAITING_SETTLE = 'WAITING_SETTLE'
READY = 'READY'
GRASPING = 'GRASPING'
SUCCESS = 'SUCCESS'
FAILED = 'FAILED'


class ArmTaskBridge(Node):
    def __init__(self):
        super().__init__('arm_task_bridge')

        # ---- 话题 ----
        self.target_topic = self.declare_parameter(
            'target_topic', '/mission/target_in_base').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odometry').value
        self.start_topic = self.declare_parameter(
            'start_topic', '/tzb_catch/start').value
        self.result_topic = self.declare_parameter(
            'result_topic', '/tzb_catch/result').value
        self.state_topic = self.declare_parameter(
            'state_topic', '/arm_task/state').value
        self.outcome_topic = self.declare_parameter(
            'outcome_topic', '/arm_task/result').value

        # ---- 可达性门限 ----
        self.min_distance_m = float(self.declare_parameter('min_distance_m', 0.25).value)
        self.max_distance_m = float(self.declare_parameter('max_distance_m', 0.55).value)
        self.max_bearing_deg = float(self.declare_parameter('max_bearing_deg', 50.0).value)
        self.min_z_m = float(self.declare_parameter('min_z_m', -0.26).value)
        self.max_z_m = float(self.declare_parameter('max_z_m', -0.05).value)

        # ---- 时序 ----
        self.stale_timeout_sec = float(self.declare_parameter('stale_timeout_sec', 0.5).value)
        self.settle_speed_mps = float(self.declare_parameter('settle_speed_mps', 0.02).value)
        self.settle_time_sec = float(self.declare_parameter('settle_time_sec', 0.5).value)
        self.stable_time_sec = float(self.declare_parameter('stable_time_sec', 0.5).value)
        self.max_spread_m = float(self.declare_parameter('max_spread_m', 0.03).value)
        self.grasp_timeout_sec = float(self.declare_parameter('grasp_timeout_sec', 90.0).value)
        self.retry_cooldown_sec = float(self.declare_parameter('retry_cooldown_sec', 2.0).value)

        # ---- 策略 ----
        self.max_retries = int(self.declare_parameter('max_retries', 2).value)
        self.require_odometry = bool(self.declare_parameter('require_odometry', True).value)
        self.enable_trigger = bool(self.declare_parameter('enable_trigger', True).value)
        self.log_throttle_sec = float(self.declare_parameter('log_throttle_sec', 2.0).value)

        state_qos = QoSProfile(depth=1,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, self.state_topic, state_qos)
        self.outcome_pub = self.create_publisher(Int8, self.outcome_topic, 10)
        self.start_pub = self.create_publisher(Int8, self.start_topic, 10)

        self.create_subscription(TargetInBase, self.target_topic, self.on_target, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(Int8, self.result_topic, self.on_result, 10)

        self.create_service(Trigger, '/arm_task/reset', self.on_reset)

        # ---- 运行状态 ----
        self._state = IDLE
        self._target = None
        self._target_time = 0.0
        self._samples = deque()
        self._speed = 0.0
        self._odom_seen = False
        self._slow_since = None
        self._attempt = 0
        self._trigger_time = 0.0
        self._retry_after = 0.0
        self._outcome_sent = False
        self._last_log = 0.0

        self.create_timer(0.05, self.tick)
        self._publish_state()
        self.get_logger().info(
            'arm_task_bridge ready: target=%s odom=%s start=%s result=%s '
            'enable_trigger=%s require_odometry=%s retries=%d'
            % (self.target_topic, self.odom_topic, self.start_topic,
               self.result_topic, self.enable_trigger, self.require_odometry,
               self.max_retries))

    # ------------------------------------------------------------------
    #  回调
    # ------------------------------------------------------------------
    def on_target(self, msg: TargetInBase):
        self._target = msg
        self._target_time = time.monotonic()
        self._samples.append((self._target_time, msg.x, msg.y))
        horizon = self._target_time - max(self.stable_time_sec, 1.0) - 0.5
        while self._samples and self._samples[0][0] < horizon:
            self._samples.popleft()

    def on_odom(self, msg: Odometry):
        v = msg.twist.twist.linear
        self._speed = math.hypot(v.x, v.y)
        self._odom_seen = True

    def on_result(self, msg: Int8):
        if self._state != GRASPING:
            self.get_logger().warn(
                '收到抓取结果 %d，但当前不在 GRASPING 状态（忽略）' % msg.data)
            return
        if msg.data > 0:
            self.get_logger().info('抓取成功')
            self._finish(SUCCESS, 1)
        else:
            self.get_logger().error('机械臂回报抓取失败')
            self._handle_failure()

    def on_reset(self, request, response):
        del request
        self._attempt = 0
        self._retry_after = 0.0
        self._outcome_sent = False
        self._samples.clear()
        self._set_state(IDLE, '手动 reset')
        response.success = True
        response.message = 'arm_task_bridge reset'
        return response

    # ------------------------------------------------------------------
    #  主循环
    # ------------------------------------------------------------------
    def tick(self):
        now = time.monotonic()

        if self._state in (SUCCESS, FAILED):
            return

        if self._state == GRASPING:
            if now - self._trigger_time > self.grasp_timeout_sec:
                self.get_logger().error(
                    '抓取超时：%.1f s 内没收到 /tzb_catch/result' % self.grasp_timeout_sec)
                self._handle_failure()
            return

        if self._retry_after > 0.0 and now < self._retry_after:
            self._set_state(IDLE, '失败后冷却中')
            return
        self._retry_after = 0.0

        ok, reason = self._target_ok(now)
        if not ok:
            self._samples.clear()
            self._set_state(IDLE, reason)
            return

        if self.require_odometry:
            if not self._odom_seen:
                self._set_state(WAITING_SETTLE, '还没收到 /odometry')
                return
            if not self._settled(now):
                self._samples.clear()
                self._set_state(WAITING_SETTLE, '车速 %.3f m/s，等待停稳' % self._speed)
                return

        if not self._target_stable(now):
            self._set_state(READY, '等待目标稳定')
            return

        self._set_state(READY, '目标有效且稳定')
        if self.enable_trigger:
            self._trigger(now)

    # ------------------------------------------------------------------
    #  判断
    # ------------------------------------------------------------------
    def _target_ok(self, now):
        if self._target is None:
            return False, '还没有目标'
        age = now - self._target_time
        if age > self.stale_timeout_sec:
            return False, '目标已过期 %.2f s' % age

        t = self._target
        if not (self.min_distance_m <= t.distance_m <= self.max_distance_m):
            return False, '距离 %.3f m 不在 [%.2f, %.2f]' % (
                t.distance_m, self.min_distance_m, self.max_distance_m)
        if abs(math.degrees(t.bearing_rad)) > self.max_bearing_deg:
            return False, '方位 %.1f° 超过 ±%.0f°' % (
                math.degrees(t.bearing_rad), self.max_bearing_deg)
        if not (self.min_z_m <= t.z <= self.max_z_m):
            return False, '高度 %.3f m 不在 [%.2f, %.2f]' % (
                t.z, self.min_z_m, self.max_z_m)
        return True, 'ok'

    def _settled(self, now):
        if self._speed > self.settle_speed_mps:
            self._slow_since = None
            return False
        if self._slow_since is None:
            self._slow_since = now
            return False
        return (now - self._slow_since) >= self.settle_time_sec

    def _target_stable(self, now):
        window = [s for s in self._samples if now - s[0] <= self.stable_time_sec]
        if len(window) < 5:
            return False
        xs = [s[1] for s in window]
        ys = [s[2] for s in window]
        spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        return spread <= self.max_spread_m

    # ------------------------------------------------------------------
    #  动作
    # ------------------------------------------------------------------
    def _trigger(self, now):
        self._attempt += 1
        self._trigger_time = now
        msg = Int8()
        msg.data = 1
        self.start_pub.publish(msg)
        self._set_state(GRASPING, '已触发第 %d 次抓取' % self._attempt)

    def _handle_failure(self):
        if self._attempt < self.max_retries + 1:
            self._retry_after = time.monotonic() + self.retry_cooldown_sec
            self._samples.clear()
            self._set_state(IDLE, '第 %d 次失败，准备重试' % self._attempt)
        else:
            self.get_logger().error('已达到最大尝试次数 %d，放弃抓取' % self._attempt)
            self._finish(FAILED, -1)

    def _finish(self, state, outcome):
        self._set_state(state, '任务结束')
        if not self._outcome_sent:
            msg = Int8()
            msg.data = outcome
            self.outcome_pub.publish(msg)
            self._outcome_sent = True

    # ------------------------------------------------------------------
    #  状态发布
    # ------------------------------------------------------------------
    def _set_state(self, state, reason=''):
        changed = state != self._state
        self._state = state
        if changed:
            self._publish_state()
        now = time.monotonic()
        if changed:
            self.get_logger().info('[%s] %s' % (state, reason))
            self._last_log = now
        elif reason and (now - self._last_log) >= self.log_throttle_sec:
            self.get_logger().info('[%s] %s' % (state, reason))
            self._last_log = now

    def _publish_state(self):
        msg = String()
        msg.data = self._state
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    node = ArmTaskBridge()
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
