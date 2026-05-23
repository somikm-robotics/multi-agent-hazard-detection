#!/usr/bin/env python3
import rclpy, sys, time
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class WaitForTFNode(Node):
    def __init__(self, timeout):
        super().__init__('wait_for_tf_node')
        self.buffer   = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timeout  = float(timeout)

    def spin_until_tf(self, parent, child):
        start = time.time()
        while rclpy.ok() and time.time() - start < self.timeout:
            try:
                self.buffer.lookup_transform(parent, child, rclpy.time.Time())
                self.get_logger().info('TF map→odom received – exiting')
                return 0                          # success
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error('Timed-out waiting for TF')
        return 1                                  # failure

def main(argv=sys.argv[1:]):
    parent, child, timeout = argv or ('map', 'odom', '15')
    rclpy.init()
    node = WaitForTFNode(timeout)
    rc = node.spin_until_tf(parent, child)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)

if __name__ == '__main__':
    main()
