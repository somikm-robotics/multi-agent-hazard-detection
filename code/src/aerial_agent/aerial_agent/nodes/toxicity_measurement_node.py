# aerial_agent/nodes/crazy_flie_patrol_node.py
import rclpy
from rclpy.node import Node
from shared_infrastructure.utils import exit_gracefully
from shared_interfaces.msg import ToxicityResult
from shared_interfaces.srv import DetectToxicGas


# ─────────  RISK / RTL tuning only ─────────
MAX_DANGER_COUNT = 3          # is unsafe after n “Dangerous” readings
HAZARD_TOPIC     = "/crazyflie/gas/ppm"

class ToxicityMeasurementNode(Node):
    def __init__(self):
        super().__init__('toxicity_measurement_node')

        self.create_subscription(ToxicityResult, HAZARD_TOPIC, self.gas_cb, 10)
        
        self.gas_pub = self.create_publisher(
            ToxicityResult,
            "/toxic_gas_reading",
            10)
        
        self.create_service(DetectToxicGas,
                            "detect_toxic_gas",
                            self.detect_toxicity)


        self.latest_gas: ToxicityResult | None = None

        self.danger_hits = 0

        self.get_logger().info('Toxicity Measurement node initialised.')

    def gas_cb(self, msg):
        # self.get_logger().info("In gas callback")
        # keep the most recent reading
        self.latest_gas = msg
        status = self.latest_gas.status
        # self.get_logger().info(f"status={status}")

    def detect_toxicity(self, request: DetectToxicGas.Request,
                    context) -> DetectToxicGas.Response:
        
        # ── Build response object ---------------------------------------------
        resp = DetectToxicGas.Response()
        if self.latest_gas is None:
            resp.is_unsafe       = False
            resp.toxicity_status = "Unknown"
            return resp
       
        
        is_unsafe = self._check_safety()
        status = self.latest_gas.status
        
        resp.is_unsafe = is_unsafe
        resp.toxicity_status = status

        if status != "Acceptable":
            self.gas_pub.publish(self.latest_gas)
            self.get_logger().info(
                f"🧪 Gas reading published: "
                f"CO={self.latest_gas.co_ppm:.1f} ppm, "
                f"NH3={self.latest_gas.nh3_ppm:.1f} ppm, "
                f"NO2={self.latest_gas.no2_ppm:.1f} ppm, "
                f"VOC={self.latest_gas.voc_ppb:.1f} ppb, "
                f"status={self.latest_gas.status}")
        else:
            self.get_logger().info(f"status={status}")
            
        return resp

    def _check_safety(self):
        status = self.latest_gas.status
        if status == "Emergency":
            self.get_logger().warn("🛑  Emergency gas detected!")
            return True

        if status == "Dangerous":
            self.danger_hits += 1
            self.get_logger().warn(
                f"⚠️  Dangerous reading #{self.danger_hits}/3")
            if self.danger_hits >= MAX_DANGER_COUNT:
                return True
        return False
            
 
def main(args=None):
    rclpy.init(args=args)
    node = ToxicityMeasurementNode()
    node.get_logger().info("✅ toxicity_measurement_node main() started") 
    exit_gracefully(node)

if __name__ == '__main__':
    main()


