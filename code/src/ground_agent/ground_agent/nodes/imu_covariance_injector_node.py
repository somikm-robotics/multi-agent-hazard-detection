import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from shared_infrastructure.utils import exit_gracefully

class ImuCovarianceInjectorNode(Node):
    def __init__(self):
        super().__init__('imu_covariance_injector_node')
        self.sub = self.create_subscription(
            Imu, '/agilex/imu/data', self.cb, 10)
        self.pub = self.create_publisher(
            Imu, '/agilex/imu/data/with_cov', 10)

    def cb(self, msg):
        # diag orientation covariance: 0.001 rad² ≈ 0.06°
        msg.orientation_covariance = [1e-3, 0, 0,
                                      0, 1e-3, 0,
                                      0, 0, 1e-3]
         # angular velocity – give Z a finite variance; X,Y small but non-zero
        msg.angular_velocity_covariance = [
            1e-6, 0.0,  0.0,
            0.0,  1e-6, 0.0,
            0.0,  0.0,  1e-4]          # ← we fuse ωz, so **must be >0**
        
        # linear acceleration (not fused right now, but safe to fill)
        msg.linear_acceleration_covariance = [
            1e-3, 0.0,  0.0,
            0.0,  1e-3, 0.0,
            0.0,  0.0,  1e-2]


        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ImuCovarianceInjectorNode()
    node.get_logger().info("✅ imu_covariance_injector_node main() started")
    exit_gracefully(node)

if __name__ == '__main__':
    main()
