# trigger node to send shutdown message so as to shut down gracefullyy
# shared_infrastructure/nodes/shutdown_trigger_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from threading import Event
import os

# This flag will help us cleanly exit after shutdown
shutdown_triggered = Event()

class ShutdownTriggerNode(Node):
    def __init__(self):
        super().__init__('shutdown_trigger_node')
        self.get_logger().info("Shutdown ListenerNode initialized")
        self.subscriber = self.create_subscription(
            Bool,
            '/shutdown_system',
            self.shutdown_callback,
            10
        )
        self.get_logger().info("🛑 ShutdownTriggerNode started and listening...")

    def shutdown_callback(self, msg):
        self.get_logger().info("🔴 Shutdown command received! Triggering launch shutdown.")
        shutdown_triggered.set()  # Signal shutdown externally


def main(args=None):
    rclpy.init(args=args)
    node = ShutdownTriggerNode()
    node.get_logger().info("✅ Shutdown Trigger Node main() started") 
    
    # Spin in background
    while rclpy.ok() and not shutdown_triggered.is_set():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()