# Placeholder for path execution using navigate_to_pose
# ground_agent/nodes/navigation_node.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.action import ActionClient
from shared_interfaces.srv import NavigateRequest
from shared_interfaces.msg import  NavigationResult
from shared_infrastructure.utils import load_service_timeout_from_config, exit_gracefully, hazard_type_to_str
import time
from nav2_msgs.action import FollowWaypoints


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info("Navigation Node initialized")
        self.service_timeout = load_service_timeout_from_config()
        self.simulated_navigation_delay = 60.0

        self.srv = self.create_service(NavigateRequest, 'navigate_to_pose', self.handle_navigation)
        
        self._follow_ac = ActionClient(self, FollowWaypoints, "/follow_waypoints")

        # self.detection_client = self.create_client(PerformHazardDetection, 'perform_hazard_detection')
        self.reached_target_pub = self.create_publisher(NavigationResult, 'reached_target', 10)
        
        # self.rtb_timer = None
        # self.rtb_active = False  # ✅ Add guard flag

        
        # self.return_simulation_timer = self.create_timer(self.simulated_navigation_delay, self.publish_return_to_base_status)
        # Timeout simulation duratioself.create_timer(1.0, self.publish_internal)n (in seconds)
        
    

    def handle_navigation(self, request, response):
        self.get_logger().info(f"📍 Navigation request received: Hazard type = {request.hazard_type}")
        
        # Store request context
        self.pending_request = request
        self.pending_response = response

        try:
            if not (self._follow_ac.wait_for_server(timeout_sec=self.service_timeout)):
                self.get_logger().info("Waiting for Nav2 action servers …")
                return          # abort the request here

        # TODO: navigate according to plan received
        # Simulate navigation  (1 min)
        # self.get_logger().info("🕒 Simulating navigation...")
        # self.simulation_timer =  self.create_timer(self.simulated_navigation_delay, self.trigger_detection_service)

         # Build FollowWaypoints goal
        
            # self.get_logger().info("Sending goal")
            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = request.waypoints

            send_future = self._follow_ac.send_goal_async(goal_msg)
            send_future.add_done_callback(self._follow_goal_response)

            response.success = True
            response.message = f"🗺️ Navigation to hazard triggered for type: {request.hazard_type}"
        except Exception as ex:
            self.get_logger().error(f"Exception Occurred: {ex}")
        return response
    
    def handle_detection_response(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"✅ Hazard handling complete: {result.result}")

            # Update and log final result (if using extended response logic)
            self.pending_response.success = True
            self.pending_response.message = result.result

        except Exception as e:
            self.get_logger().error(f"❌ Detection service failed: {e}")

    def _follow_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("FollowWaypoints goal rejected")
            return

        self.get_logger().info("Waypoints goal accepted; waiting for completion …")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._follow_result)

    # ────────────────────────────────────────────────────────────────────────
    def _follow_result(self, future):
        
        result = future.result().result

        navigation_result = NavigationResult()

        if result.missed_waypoints:
            navigation_result.message = f"Reached destination but missed a few waypoints"
            self.get_logger().warn(f"Completed with missed indices: {result.missed_waypoints}")
        else:
            navigation_result.message  = "Reached destination successfully!"
            self.get_logger().info("All waypoints reached successfully!")
        navigation_result.hazard_type = self.pending_request.hazard_type
        

        self.reached_target_pub.publish(navigation_result)
        self.get_logger().info("Navigation result published successfully")
    
        

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    node.get_logger().info("✅ navigation_node main() started")
    exit_gracefully(node)
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()
