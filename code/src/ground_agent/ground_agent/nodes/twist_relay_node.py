#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from shared_infrastructure.utils import exit_gracefully


class TwistRelayNode(Node):
    def __init__(self):
        super().__init__('twist_relay_node')

        self.publisher = self.create_publisher(
            Twist, '/diff_drive_base_controller/cmd_vel_unstamped', 10)

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.relay_cmd, 10)

        self.get_logger().info('🔁 Relaying /cmd_vel → /diff_drive_base_controller/cmd_vel_unstamped')

    def relay_cmd(self, msg: Twist):
        try:
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish Twist: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TwistRelayNode()
    node.get_logger().info("✅ twist_relay_node main() started")
    exit_gracefully(node)


if __name__ == '__main__':
    main()
