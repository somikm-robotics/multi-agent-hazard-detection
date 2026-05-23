import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from shared_infrastructure.utils import exit_gracefully

class GPSCovarianceInjectorNode(Node):
    def __init__(self):
        super().__init__('gps_covariance_injector_node')
        self.sub = self.create_subscription(NavSatFix, '/gps/fix', self.inject, 10)
        self.pub = self.create_publisher(NavSatFix, '/gps/fix/with_cov', 10)

    def inject(self, msg):
        var_xy = 3.0 ** 2      # 9
        var_z  = 6.0 ** 2      # 36

        msg.position_covariance = [
            var_xy, 0.0,   0.0,
            0.0,   var_xy, 0.0,
            0.0,   0.0,   var_z]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = GPSCovarianceInjectorNode()
    node.get_logger().info("✅ gps_covariance_injector_node main() started")
    exit_gracefully(node)

if __name__ == '__main__':
    main()
