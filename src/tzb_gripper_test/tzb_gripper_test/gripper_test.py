#!/usr/bin/env python3
"""Standalone gripper command tester.

Only publishes std_msgs/Int8 on the gripper command topic, so the serial
driver / electronics can be exercised without the arm stack.

Protocol:
    -1 = open
     1 = closed mode 1 (blue_block)
     2 = closed mode 2 (big_rock)
     3 = closed mode 3 (small_rock)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8

VALID_COMMANDS = (-1, 1, 2, 3)
COMMAND_NAMES = {
    -1: 'open',
    1: 'closed mode 1 (blue_block)',
    2: 'closed mode 2 (big_rock)',
   
}


class GripperTestNode(Node):
    def __init__(self):
        super().__init__('gripper_test')
        self.declare_parameter('command', -1)
        self.declare_parameter('topic', '/tzb_catch/gripper_command')
        self.declare_parameter('command_topic', '/gripper_test/command')
        self.declare_parameter('publish_hz', 10.0)

        self.command = int(self.get_parameter('command').value)
        self.topic = str(self.get_parameter('topic').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        hz = float(self.get_parameter('publish_hz').value)
        if hz <= 0.0:
            self.get_logger().warning('publish_hz <= 0, using 10 Hz')
            hz = 10.0

        if self.command not in VALID_COMMANDS:
            self.get_logger().error(
                f'command {self.command} is not one of {VALID_COMMANDS}; falling back to -1 (open)')
            self.command = -1

        # The serial driver subscribes with transient_local + reliable, so the
        # publisher must match that QoS to be compatible.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Int8, self.topic, qos)
        self.subscription = self.create_subscription(
            Int8, self.command_topic, self.on_command, 10)
        self.timer = self.create_timer(1.0 / hz, self.publish_command)

        self.get_logger().info(
            f'gripper_test ready: topic={self.topic} command={self.command} '
            f'({COMMAND_NAMES.get(self.command, "?")}) at {hz:.1f} Hz; '
            f'send new values on {self.command_topic}')
        self.publish_command()

    def on_command(self, msg):
        value = int(msg.data)
        if value not in VALID_COMMANDS:
            self.get_logger().warning(
                f'ignoring {value}: only {VALID_COMMANDS} are allowed')
            return
        if value == self.command:
            return
        self.command = value
        self.get_logger().info(
            f'command -> {value} ({COMMAND_NAMES.get(value, "?")})')
        self.publish_command()

    def publish_command(self):
        self.publisher.publish(Int8(data=self.command))


def main(args=None):
    rclpy.init(args=args)
    node = GripperTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
