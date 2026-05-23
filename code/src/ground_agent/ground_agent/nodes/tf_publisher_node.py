# Placeholder for publishing ground robot pose as TF frames
# ground_agent/nodes/tf_publisher_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from shared_infrastructure.utils import exit_gracefully

import time


class TfPublisherNode(Node):
    def __init__(self):
        super().__init__('tf_publisher_node')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.5, self.broadcast_tf)
     
        # Publish initial pose after 2 seconds
        self.get_logger().info("✅ Ground Agent: tf_publisher_node main() started")

    def broadcast_tf(self):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'

            
            t.child_frame_id = 'agilex_diff_drive/base_link'     

            t.transform.translation.x = 2.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0  # Slight lift for visibility
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.br.sendTransform(t)

    
def main(args=None):
    rclpy.init(args=args)
    node = TfPublisherNode()
    node.get_logger().info("✅ tf_publisher_node main() started")
    exit_gracefully(node)
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()