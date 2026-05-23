# Handles mission initiation, override, and status reporting
# ground_agent/nodes/mission_handler_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from shared_interfaces.srv import NavigateRequest, OverrideMission, RequestPathPlan, PathPlanResult
from shared_infrastructure.hazard_types import HazardType
from shared_infrastructure.utils import load_service_timeout_from_config, parse_hazard_type, get_agilex_base_pose
from shared_infrastructure.utils import hazard_type_to_str, exit_gracefully, load_navigation_override_timeout_from_config
from shared_interfaces.msg import HazardPose, NavigationResult, BaseReturn, OnArrivalTask, MissionStatus
from rclpy.action import ActionClient
import math
from shared_infrastructure.task_types import TaskType
from shared_infrastructure.utils import task_type_to_str, find_orbit_safety_radius, parse_task_type
from shared_infrastructure.utils import  compute_edge, is_orbit_clockwise, get_pose, load_agilex_camera_offset_from_config

class MissionHandlerNode(Node):
    def __init__(self):
        super().__init__('mission_handler_node')
        self.get_logger().info("Mission Handler Node initialized")

         # Internal state
        self.approval = None
        self.orbit_approval = None
        self.hazard_received = False
        self.hazard_type = HazardType.NONE
        self.task_type = TaskType.NONE
        self.hazard_location = None
        self.hazard_pose = None
        self.hazard_orbit_pose = None
        self.estimated_hazard_diameter = None
        self.hazard_center_point = None
        self.robot_alt = None
        self.agilex_pose = None
        self.service_timeout = load_service_timeout_from_config()
        self.navigation_override_timeout_in_minutes = load_navigation_override_timeout_from_config()
        
        # Subscriptions
        self.create_subscription(HazardPose, 'hazard_detected', self.hazard_callback, 10)
        self.create_subscription(NavigationResult, 'reached_target', self.reached_target_callback, 10)
        # self.create_subscription(bool, 'mission_complete', self.mission_complete, 10)
        # self.create_subscription(OverrideMission, 'override_mission', self.override_callback, 10)
        self.create_subscription(MissionStatus, 'on_arrival_mission_status', self.on_arrival_task_completion, 10)

        # self.create_subscription(String, 'test_set_orbit_goal', self.test_set_orbit_goal, 10)

        self.on_arrival_task_pub = self.create_publisher(OnArrivalTask, 'on_arrival_task', 10)
        self.base_return_pub = self.create_publisher(BaseReturn, 'return_to_base_status', 10)
        self.intermediary_mission_status_pub = self.create_publisher(MissionStatus, 'intermediary_mission_status', 10)

        # Service
        self.override_mission_srv = self.create_service(OverrideMission, 'override_mission', self.handle_override)

        self.path_plan_return_srv = self.create_service(PathPlanResult, 'receive_path_result', self.handle_path_result) 


        # Service client using  custom NavigateRequest
        self.path_plan_client = self.create_client(RequestPathPlan, 'request_path_plan')
        self.navigate_client = self.create_client(NavigateRequest, 'navigate_to_pose')
        
               
        self.tasks_done = False

        self.returned_to_base = True  # needs to be set to True to process hazards.
        self.is_return = False
        
        # self.srv = self.create_service(OverrideMission, 'override_mission', self.override_callback)
        self.get_logger().info("✅ Mission Handler ready for override commands")

    def hazard_callback(self, msg):
        self.get_logger().info(f"📥 Ground Agent received: {msg.hazard_type}")

        if not self.returned_to_base:
            self.get_logger().info("Hazard Processing in progress")
            return

        self.hazard_received = True
        self.approval = None
        self.hazard_pose = msg.pose
        self.returned_to_base = False
        self.estimated_hazard_diameter = msg.estimated_hazard_diameter
        self.hazard_center_point = msg.hazard_center_point
        self.agilex_pose = get_agilex_base_pose()
        
        #  Extract hazard type from message string
        try:
            self.hazard_type = parse_hazard_type(msg.hazard_type)
            self.task_type = (
                TaskType.INITIAL_INSPECTION if self.hazard_type == HazardType.FIBROUS_HAZARD
                else TaskType.DENSITY_ESTIMATION if self.hazard_type == HazardType.DUST_PLUME
                else TaskType.NONE
            )

            self.get_logger().info(f"📥 Waiting: {self.navigation_override_timeout_in_minutes} minute(s)")
            self.get_logger().info(f"Hazard Type: {self.hazard_type} Task: {self.task_type}")
            override_timeout_in_seconds = self.navigation_override_timeout_in_minutes * 60
            self.override_timer = self.create_timer(override_timeout_in_seconds, self.check_override_status)
        except Exception as e:
            self.get_logger().error(f"Failed to parse hazard type: {e}")
            return
        
    def handle_override(self, request, response):
        self.get_logger().info(f"🔁 Ground Agent Received Override command: {request.command}")

        message = "🟢 APPROVED. Dispatching mission." if request.command == "approve" \
                        else "🚫 CANCELLED! Supervisor aborted the mission."
        if request.command == "approve":
            response.status = "dispatched"
            self.publish_intermediary_status(message, True)
            self.get_logger().info(message)

            if self.hazard_received:
                if self.task_type == TaskType.INITIAL_INSPECTION:
                    self.approval = True
                    self.request_path_for_navigation()
                else:
                    self.orbit_approval = True
                    self.set_orbit_goal()
                    self.request_path_for_navigation()
                self.hazard_received = False

        elif request.command == "cancel":
            self.approval = False
            if self.task_type == TaskType.INITIAL_INSPECTION:
                    self.approval = False
            else:
                self.orbit_approval = True
            response.status = "cancelled"
            self.get_logger().info("🚫 CANCELLED! Supervisor aborted the mission.")
            self.hazard_received = False
            self.publish_intermediary_status(message, True)

        else:
            response.status = "invalid"
            self.get_logger().warn("❓ Unknown override command received.")

        return response


    def check_override_status(self):
        try:
            if self.task_type != TaskType.NONE:
                if not self.hazard_received:
                    return  # nothing to do
            self.get_logger().info(f"Task type: {self.task_type}")
            initial_task_type = (
                TaskType.INITIAL_INSPECTION if self.hazard_type == HazardType.FIBROUS_HAZARD
                else TaskType.DENSITY_ESTIMATION if self.hazard_type == HazardType.DUST_PLUME
                else TaskType.NONE
            )
            if (self.task_type == initial_task_type and self.approval is None) or \
                (self.task_type == TaskType.ASBESTOS_ANALYSIS and self.orbit_approval is None):
                message = f"⏱️ No override received. Timeout reached. Proceeding with mission."
                self.get_logger().info(message)

                if self.task_type == initial_task_type:
                    self.override_timer.cancel()  # stop repeated calls
                else:
                    self.override_timer_step_2.cancel()
                self.publish_intermediary_status(message, True)

                if self.task_type == TaskType.INITIAL_INSPECTION:
                    self.request_path_for_navigation()
                else:
                    self.set_orbit_goal()
                    self.request_path_for_navigation()

        except Exception as ex:
            self.get_logger().info(f"Some exception occurred: {ex}")
        # approval/cancel handled in override_callback already
        self.hazard_received = False
        self.approval = None
        self.orbit_approval = None
        

    def publish_intermediary_status(self, message, toggle_command):
        msg = MissionStatus()
        msg.hazard_type = hazard_type_to_str(self.hazard_type)
        msg.task_type = task_type_to_str(self.task_type)
        msg.message = message
        msg.success = True
        msg.toggle_command = toggle_command
        self.intermediary_mission_status_pub.publish(msg)
        self.get_logger().info(f"Intermediary Status: {message} published")


    def get_return_pose(self):
        
        x, y, z, yaw = self.agilex_pose

        self.robot_alt = z
        # Convert yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        
        pose = get_pose(x, y, z, qz, qw)
        return pose
    
    # def test_set_orbit_goal(self, msg):
    #     self.hazard_pose = Pose(
    #         position=Point(x=1.0, y=3.5, z=4.014917814708891),
    #         orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    #     )

    #     self.hazard_center_point = Point(x=1.5, y=4.0, z=0.0)
    #     self.hazard_type = HazardType.FIBROUS_HAZARD
    #     self.estimated_hazard_diameter = 1.2
    #     # self.is_return = True
    #     self.reached_target_callback()

    def set_orbit_goal(self):
        robot_pose_x = self.hazard_pose.position.x
        robot_pose_y = self.hazard_pose.position.y
        hazard_pose_x = self.hazard_center_point.x
        hazard_pose_y = self.hazard_center_point.y

        self.get_logger().info(f"Hazard pose: ({hazard_pose_x}, {hazard_pose_y}) Robot pose: ({robot_pose_x}, {robot_pose_y})")
        edge = compute_edge(robot_pose_x, robot_pose_y, hazard_pose_x, hazard_pose_y)
        self.get_logger().info(f"Edge: {edge}")
        orbit_safety_radius = find_orbit_safety_radius(self.hazard_type, self.estimated_hazard_diameter)
        self.get_logger().info(f"Orbit Safety Radius: {orbit_safety_radius}")

        camera_offset = load_agilex_camera_offset_from_config()
        # how far the *base_link* must be from the hazard centre
        r_base = orbit_safety_radius + camera_offset

        self.get_logger().info(f"R_base: {r_base}")

        xs, ys = {
        "west":  (hazard_pose_x - r_base, hazard_pose_y),
        "east":  (hazard_pose_x + r_base, hazard_pose_y),
        "south": (hazard_pose_x, hazard_pose_y - r_base),
        "north": (hazard_pose_x, hazard_pose_y + r_base),
        }[edge]

        is_clockwise = is_orbit_clockwise()
        self.get_logger().info(f"Is closkwise: {is_clockwise}")

        inward   = math.atan2(hazard_pose_y - ys, hazard_pose_x - xs)
        heading = inward + math.pi/2  if is_orbit_clockwise() else inward - math.pi/2
        qz = math.sin(heading/2)
        qw = math.cos(heading/2)
        _, _, z, _ = self.agilex_pose
        self.get_logger().info(f"Navigating to : x: {xs}, y: {ys}, qz: {qz}, qw: {qw} ")
        self.hazard_orbit_pose = get_pose(xs, ys, z, qz, qw)

    def request_path_for_navigation(self):
        if not self.path_plan_client.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().error("❌ path_plan service not available.")
            return
        
        try:
            self.get_logger().info("Getting path")
            req = RequestPathPlan.Request()
            req.goal = (
                self.return_pose if self.is_return else
                self.hazard_orbit_pose if self.task_type != TaskType.INITIAL_INSPECTION else
                self.hazard_pose
            )

            req.is_return = self.is_return
            # self.get_logger().info("Set request")
            future = self.path_plan_client.call_async(req)
            # self.get_logger().info("request sent")
            future.add_done_callback(self.handle_path_plan_response)
            # self.get_logger().info("added callback")
        except Exception as ex:
            self.get_logger().info(f"Exception raised - {ex} ")

    def send_navigation_command(self, waypoints):
        if not self.navigate_client.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().error("❌ navigate_to_pose service not available.")
            return

        req = NavigateRequest.Request()

        req.waypoints = waypoints
        req.hazard_type = hazard_type_to_str(self.hazard_type)

        future = self.navigate_client.call_async(req)
        future.add_done_callback(self.handle_navigation_response)


    def handle_navigation_response(self, future):
        try:
            response = future.result()
            if response.success:
                if self.is_return:
                    self.get_logger().info(f"✅ Return navigation successfully triggered: {response.message}")    
                else:
                    self.get_logger().info(f"✅ Navigation successfully triggered: {response.message}")
            else:
                self.get_logger().warn(f"⚠️ Navigation trigger failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"🚨 Service call failed: {str(e)}")


    def handle_path_plan_response(self, future):
        try:
            response = future.result()
            if response.accepted:
                self.get_logger().info(f"✅ Path Plan successfully triggered - {response.message}")
            else:
                self.get_logger().warn(f"⚠️ Path Plan trigger failed! - {response.message}")
                return None
        except Exception as e:
            self.get_logger().error(f"🚨 Service call failed: {str(e)}")

    def handle_path_result(self, request, response):
        self.get_logger().info(f"Received waypoints: {len(request.waypoints)} poses")
        response.success = True
        self.send_navigation_command(request.waypoints)
        return response

    def reached_target_callback(self, msg):
        try:
            # self.get_logger().info("In reached_target_callback")
            if self.is_return:
                self.set_next_task_type()    
                if self.tasks_done:
                    pub_msg = f"{msg.message} for {self.hazard_type}"
                    self.get_logger().info(f"Publishing message: {pub_msg}")
                    self.publish_return_to_base_status()
                else:
                    self.is_return = False
                    self.wait_for_approval_or_timeout()
                    # self.set_orbit_goal()
                    # self.request_path_for_navigation()
            else:
                #  first task
                self.publish_on_arrival_task()
        except Exception as ex:
            self.get_logger().info(f"Some Exception occurred: {ex}")

    def wait_for_approval_or_timeout(self):
        self.get_logger().info(f"Hazard Type: {self.hazard_type} Task: {self.task_type}")
        msg = f"📥 Waiting: {self.navigation_override_timeout_in_minutes} minute(s) for supervisor override after return to base"
        self.get_logger().info(msg)

        override_timeout_in_seconds = self.navigation_override_timeout_in_minutes * 60
        self.publish_intermediary_status(msg, True)
        self.hazard_received = True
        self.override_timer_step_2 = self.create_timer(override_timeout_in_seconds, self.check_override_status)
        

    def publish_on_arrival_task(self):
        try:
            on_arrival_msg = OnArrivalTask()
            on_arrival_msg.hazard_type = hazard_type_to_str(self.hazard_type)
            on_arrival_msg.task_type = task_type_to_str(self.task_type)
            on_arrival_msg.estimated_hazard_diameter = self.estimated_hazard_diameter
            on_arrival_msg.hazard_center_point = self.hazard_center_point
            self.on_arrival_task_pub.publish(on_arrival_msg)
            self.get_logger().info(f"Publishied message On Arrival for : {self.hazard_type} - {self.task_type}")
            msg = f"Arrived at the hazard and will start {task_type_to_str(self.task_type)}"
            self.publish_intermediary_status(msg, False)
        except Exception as ex:
            self.get_logger().info(f"Some Exception occurred: {ex}")
        
    def on_arrival_task_completion(self, msg):
        self.is_return = True
        if not msg.success:
            self.get_logger().error(f"Received error message: {msg.message} for {msg.hazard_type} performing {msg.task_type}")        
            return

        self.get_logger().info(f"Received message: {msg.message} for {msg.hazard_type} performing {msg.task_type}")
        self.hazard_type = parse_hazard_type(msg.hazard_type)
        # self.task_type == parse_task_type(msg.task_type)
        self.publish_intermediary_status(msg.message, False)

        self.start_return_to_base(msg)

    def set_next_task_type(self):
        if self.hazard_type == HazardType.DUST_PLUME:
            self.task_type = TaskType.NONE
            self.tasks_done = True
        elif self.hazard_type == HazardType.FIBROUS_HAZARD:
            if self.task_type == TaskType.INITIAL_INSPECTION:
                self.task_type = TaskType.ASBESTOS_ANALYSIS
            elif self.task_type == TaskType.ASBESTOS_ANALYSIS:
                self.task_type = TaskType.NONE
                self.tasks_done = True
            
        

    def start_return_to_base(self, msg):
        # if self.rtb_active:
        #     self.get_logger().warn("⚠️ Return-to-base already in progress. Ignoring duplicate call.")
        #     return
        if not msg.success:
            self.get_logger().error(f"Received message: {msg.message} for {msg.hazard_type} performing {msg.task_type}")        
            return
        
        self.get_logger().info(f"Received message: {msg.message} for {msg.hazard_type} performing {msg.task_type}")
        try:
        # self.rtb_active = True  # ✅ Set flag
            self.return_pose = self.get_return_pose()
            self.is_return = True
            self.get_logger().info(f"On Arrival Task completed for {msg.hazard_type}:  - Navigating back to base...")
            self.request_path_for_navigation()
        except Exception as ex:    
            self.get_logger().info(f"Some Exception occurred: {ex}")
        

    def publish_return_to_base_status(self):
        
        # if self.rtb_timer:
        #     self.rtb_timer.cancel()
        #     self.rtb_timer = None

        msg = BaseReturn()
        msg.message = f"Returned to base after completion for mission for {self.hazard_type}. Standing by for next mission."
        msg.success = True
        self.base_return_pub.publish(msg)
        self.get_logger().info("🏁 Return to base status published.")
        # self.rtb_active = False  # ✅ Reset flag

    


def main(args=None):
    rclpy.init(args=args)
    node = MissionHandlerNode()
    node.get_logger().info("✅ mission_handler_node main() started") 
    exit_gracefully(node)
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()