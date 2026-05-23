# Placeholder for subscribing to TF frames and resolving transforms
# ground_agent/nodes/tf_listener_node.py

import rclpy
from rclpy.node import Node
from shared_infrastructure.utils import exit_gracefully

class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.get_logger().info("Tf Listener Node initialized")

def main(args=None):
    rclpy.init(args=args)
    node = TfListenerNode()
    node.get_logger().info("✅ tf_listener_node main() started")
    exit_gracefully(node)
    

if __name__ == "__main__":
    main()

