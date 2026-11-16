#!/usr/bin/env python3
"""Move the arm to a target JointState and read back the measured pose.

This does NOT calibrate the zero by itself.  It only moves the arm and prints
target / measured / error.  Zero calibration still needs a physical reference
(mechanical marks, fixture, angle gauge, or the lower controller's zero mode).

SAFETY
  - Start only the arm serial driver, NOT tzb_catch/planning_node.
  - Support the arm, clear the workspace, keep the e-stop in hand.
  - The target is in ROS radians for J1_joint ... J6_joint.
  - There is a 3 s countdown before commands start; Ctrl+C aborts.

Examples
  python3 move_and_read.py --target 0 0 0 0 0 0
  python3 move_and_read.py --target 0.05 0 0 0 0 0 --duration 2
  python3 move_and_read.py --target 0 1.0 -2.61 0 1.3 0
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

NAMES = [
    "J1_joint",
    "J2_joint",
    "J3_joint",
    "J4_joint",
    "J5_joint",
    "J6_joint",
]


class MoveRead(Node):
    def __init__(self, target, cmd_topic, state_topic, rate_hz):
        super().__init__("move_and_read")
        self.target = [float(v) for v in target]
        self.enabled = False
        self.measured = None
        self.measured_mono = 0.0
        self.last_state_msg_mono = 0.0

        self.pub = self.create_publisher(JointState, cmd_topic,
                                         qos_profile_sensor_data)
        self.create_subscription(JointState, state_topic, self.on_state,
                                 qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / max(1.0, rate_hz), self.send)

    def send(self):
        if not self.enabled:
            return
        msg = JointState()
        msg.name = NAMES
        msg.position = self.target
        self.pub.publish(msg)

    def on_state(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(name in d for name in NAMES):
            self.measured = [float(d[name]) for name in NAMES]
            self.measured_mono = time.monotonic()
            self.last_state_msg_mono = self.measured_mono


def wait_for_state(node, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and node.measured is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.measured is not None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", nargs=6, type=float, required=True,
                        help="6 个 ROS 关节角，单位 rad")
    parser.add_argument("--cmd-topic", default="/tzb_catch/arm_joint_command")
    parser.add_argument("--state-topic", default="/serial/arm_joint_state")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="命令发布频率 Hz，默认 20")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="持续发目标的秒数")
    parser.add_argument("--settle", type=float, default=6.0,
                        help="停发后等待稳定的最长秒数")
    parser.add_argument("--out", default="/tmp/move_read.json")
    parser.add_argument("--no-countdown", action="store_true",
                        help="跳过 3 秒倒计时（危险，不建议）")
    args = parser.parse_args()

    if len(args.target) != 6:
        parser.error("--target 需要 6 个数")

    rclpy.init()
    node = MoveRead(args.target, args.cmd_topic, args.state_topic, args.rate)

    try:
        if not wait_for_state(node, 5.0):
            print("ERROR: 5 s 内没有收到 %s，检查串口驱动/设备/下位机" % args.state_topic)
            return 1

        before = list(node.measured)
        print("state topic :", args.state_topic)
        print("cmd topic   :", args.cmd_topic)
        print("before (rad):", [round(v, 5) for v in before])
        print("target (rad):", [round(v, 5) for v in args.target])
        print("target (deg):", [round(v * 57.2957795, 2) for v in args.target])
        print()
        print("WARNING: 这是直接给串口驱动发关节目标；tzb_catch/planner 必须关闭。")
        print("WARNING: 只有机械臂在物理零位时读到的 measured 才是零位 offset。")

        if not args.no_countdown:
            for remaining in (3, 2, 1):
                print("moving in %d s ... (Ctrl+C to abort)" % remaining)
                time.sleep(1.0)

        node.enabled = True
        print("publishing target for %.1f s ..." % args.duration)
        start = time.monotonic()
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.measured_mono and time.monotonic() - node.measured_mono > 0.5:
                print("WARNING: %s 超过 0.5 s 没更新" % args.state_topic)

        node.enabled = False
        print("target hold; waiting for settle (max %.1f s) ..." % args.settle)

        last = None
        stable = 0
        deadline = time.monotonic() + args.settle
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.measured is None:
                continue
            if last is not None:
                delta = max(abs(a - b) for a, b in zip(node.measured, last))
                stable = stable + 1 if delta < 0.002 else 0
            last = node.measured
            if stable >= 10:
                break

        if node.measured is None:
            print("ERROR: 没有实测值")
            return 1

        error = [m - t for m, t in zip(node.measured, args.target)]
        print()
        print("measured (rad):", [round(v, 5) for v in node.measured])
        print("measured (deg):", [round(v * 57.2957795, 2) for v in node.measured])
        print("error (rad)   :", [round(v, 5) for v in error])
        print("max |error|   : %.5f rad (%.2f deg)" % (
            max(abs(v) for v in error),
            max(abs(v) for v in error) * 57.2957795))

        result = {
            "state_topic": args.state_topic,
            "cmd_topic": args.cmd_topic,
            "before_rad": before,
            "target_rad": args.target,
            "measured_rad": node.measured,
            "error_rad": error,
            "timestamp": time.time(),
        }
        with open(args.out, "w") as handle:
            json.dump(result, handle, indent=2)
        print("saved:", args.out)
        return 0

    except KeyboardInterrupt:
        print("\naborted by user")
        return 130
    finally:
        node.enabled = False
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
