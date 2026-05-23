# Placeholder for publishing hazard notification messages
# aerial_agent/nodes/notifier_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from shared_infrastructure.utils import exit_gracefully
from shared_interfaces.msg import HazardPose


class NotifierNode(Node):
    def __init__(self):
        super().__init__('notifier_node')
        self.get_logger().info("Notifier Node initialized")
        self.publisher_ = self.create_publisher(HazardPose, 'hazard_detected', 10)
        self.subscription = self.create_subscription(
            HazardPose,
            'internal_hazard',
            self.forward_hazard,
            10
        )
        # self.subscription  # prevent unused warning

    def forward_hazard(self, msg):
        self.publisher_.publish(msg)
        self.get_logger().info(f"📡 Forwarded: Hazard Pose")

def main(args=None):
    rclpy.init(args=args)
    node = NotifierNode()
    node.get_logger().info("✅ notifier_node main() started") 
    exit_gracefully(node)

if __name__ == "__main__":
    main()

