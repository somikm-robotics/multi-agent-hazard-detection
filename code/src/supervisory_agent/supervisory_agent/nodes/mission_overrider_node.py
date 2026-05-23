# Publishes /approve_mission, /cancel_mission, and /scan_incident
# supervisory_agent/nodes/mission_overrider_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from shared_interfaces.srv import ScanIncident
from shared_infrastructure.utils import load_service_timeout_from_config, exit_gracefully
from shared_interfaces.srv import OverrideMission
from supervisory_agent.data_loader import get_location_coordinates
from shared_interfaces.msg import UIOverrideMission


class MissionOverriderNode(Node):
    def __init__(self):
        super().__init__('mission_overrider_node')
        self.get_logger().info("Mission Overrider Node initialized")
        
        # Service clients for override / scan logic
        self.override_client = self.create_client(OverrideMission, 'override_mission')
        self.scan_incident_client = self.create_client(ScanIncident, 'scan_incident')
        self.service_timeout = load_service_timeout_from_config()

        self.service_timeout = load_service_timeout_from_config()

        # Subscriptions
        self.create_subscription(UIOverrideMission, 'ui_override_command', self.ui_override_handler, 10)
        self.create_subscription(String, 'ui_scan_incident', self.ui_scan_incident_handler, 10)

        # Publisher
        self.ui_status_pub = self.create_publisher(String, 'mission_status', 10)
    
    def ui_override_handler(self, msg):
        self.send_overide_request(msg)

    def send_overide_request(self, msg):

        if msg.command not in ['approve', 'cancel']:
            self.get_logger().warn(f"Unknown override command: {command}")
            return

        if not self.override_client.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().error("❌ override_mission service not available.")
            return

        req = OverrideMission.Request()
        req.command = msg.command
        req.is_initial_inspection = msg.is_initial_inspection
        future = self.override_client.call_async(req)
        future.add_done_callback(self.handle_override_response)

    def handle_override_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"📬 Override Response: {response.status}")
            msg = String()
            msg.data = f"Override Response: {response.status}"   # ✅ wrap string properly
            self.ui_status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"❌ Service call failed: {e}")


    def ui_scan_incident_handler(self, msg):
        # location = self.location_entry.get().strip()  # Get text from the UI textbox
        location = msg.data.strip()
        if not location:
            self.get_logger().warn("⚠️ Location field is empty. Scan request not sent.")
            return

        location_coords = get_location_coordinates(location)
        if not self.scan_incident_client.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().error("❌ scan_incident service not available.")
            return


        req = ScanIncident.Request()
        pt = Point()
        pt.x = float(location_coords["x"])
        pt.y = float(location_coords["y"])
        pt.z = 0.0   # or set as needed

        req.incident_point = pt
        
        future = self.scan_incident_client.call_async(req)
        future.add_done_callback(self.handle_scan_response)

        self.get_logger().info(f"🛰️ Incident scan request sent to Aerial Agent. Location: {location}")

    def handle_scan_response(self, future):
        try:
            response = future.result()
            if response.accepted:
                self.get_logger().info(f"📡 Scan accepted.")
            else:
                self.get_logger().info(f"📡 Scan NOT accepted")
        except Exception as e:
            self.get_logger().error(f"🚨 Scan request failed: {str(e)}")
    


def main(args=None):
    rclpy.init(args=args)
    node = MissionOverriderNode()
    node.get_logger().info("✅ mission_overrider_node main() started")
    exit_gracefully(node)
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()