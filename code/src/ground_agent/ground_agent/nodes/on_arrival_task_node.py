import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from shared_infrastructure.utils import load_service_timeout_from_config, parse_hazard_type, hazard_type_to_str, exit_gracefully
from shared_infrastructure.utils import parse_task_type, task_type_to_str
from shared_interfaces.msg import  OnArrivalTask, MissionStatus, DensityEstimationResult
from shared_infrastructure.hazard_types import HazardType
from shared_infrastructure.task_types import TaskType
from shared_interfaces.srv import OrbitHazard, InitialInspection, PlumeDensity


class OnArrivalHazardNode(Node):
    def __init__(self):
        super().__init__('on_arrival_task_node')
        

        self.create_subscription(OnArrivalTask, 'on_arrival_task', self.on_arrival, 10)
        self.create_subscription(Bool, 'orbit_complete', self.on_orbit_complete, 10)
        self.create_subscription(Bool, 'inspection_complete', self.on_initial_inspection_complete, 10)
        self.create_subscription(DensityEstimationResult, 'estimation_complete', self.on_estimation_complete, 10)

        self.status_pub = self.create_publisher(MissionStatus, 'on_arrival_mission_status', 10)
        
        self.initial_insp_client = self.create_client(InitialInspection, "initial_inspection")
        self.fibrous_hazard_orbit_client = self.create_client(OrbitHazard, 'orbit_fibrous_hazard')
        self.dust_plume_estimation_client = self.create_client(PlumeDensity, 'estimate_plume_density')
        
        self.hazard_type = HazardType.NONE
        self.task_type = TaskType.NONE
        self.service_timeout = load_service_timeout_from_config()

        self.get_logger().info("🧪 On Arrival Task Node ready.")

    def on_arrival(self, msg):
        try:
            # self.get_logger().info(f"On arrival: {msg.hazard_type}")
            self.hazard_type = parse_hazard_type(msg.hazard_type)
            self.task_type = parse_task_type(msg.task_type)
            self.get_logger().info(f"On arrival: {self.hazard_type} - {self.task_type}")


            if self.hazard_type == HazardType.FIBROUS_HAZARD:
                if self.task_type == TaskType.INITIAL_INSPECTION:
                    if not self.initial_insp_client.wait_for_service(timeout_sec=self.service_timeout):
                        self.get_logger().error("❌ initial_inspection service not available.")
                        return    

                    self.get_logger().info("Requesting initial inspection")  
                    req = InitialInspection.Request()
                    req.hazard_type = msg.hazard_type
                    self.initial_insp_client.call_async(req)
                    # future.add_done_callback(self.on_initial_inspection_complete)
                    return
                elif self.task_type == TaskType.ASBESTOS_ANALYSIS:
                    if not self.fibrous_hazard_orbit_client.wait_for_service(timeout_sec=self.service_timeout):
                        self.get_logger().error("❌ Fibrous hazard Orbit Client service not available.")
                        return    

                    self.get_logger().info("Requesting asbestos orbit for analysis")  
                    request = OrbitHazard.Request()
                    request.hazard_type = msg.hazard_type
                    request.estimated_hazard_diameter = msg.estimated_hazard_diameter
                    request.hazard_center_point = msg.hazard_center_point

                    self.fibrous_hazard_orbit_client.call_async(request)

            elif self.hazard_type == HazardType.DUST_PLUME:
                if not self.dust_plume_estimation_client.wait_for_service(timeout_sec=self.service_timeout):
                        self.get_logger().error("❌ Dust Pume densty estimation service not available.")
                        return    

                self.get_logger().info("Requesting dust plume densoty estimation")
                req = PlumeDensity.Request()
                self.dust_plume_estimation_client.call_async(req)    
        except Exception as ex:
            self.get_logger().info(f"Some exception occurred - {ex}")

    def on_estimation_complete(self, msg):
        message = f"Dust Plume density estimate - {msg.density} Level: {msg.level}"
        self.publish_task_complete_message(message)


    def on_orbit_complete(self, msg):
        # self.status_pub.publish(f"Orbit of Fibrous hazard completed, ros2 bags collected for further analysis")
        message = "Orbit of Fibrous hazard completed, ros2 bags collected for further analysis"
        self.publish_task_complete_message(message, msg.data)
        
    def on_initial_inspection_complete(self, msg):
        try:
            self.publish_task_complete_message("Inspection Completed", msg.data)
        except Exception as ex:
            self.get_logger().info(f"Some exception occurred - {ex}")            

    def publish_task_complete_message(self, message, success = True):
        msg = MissionStatus()
        msg.hazard_type = hazard_type_to_str(self.hazard_type)
        msg.task_type = task_type_to_str(self.task_type)
        msg.message = message
        msg.success = success
        self.status_pub.publish(msg)
        self.get_logger().info(f"Published Mission Status message for {msg.hazard_type} - {msg.task_type}")


def main(args=None):
    rclpy.init(args=args)
    node = OnArrivalHazardNode()
    node.get_logger().info("✅ on_arrival_task_node main() started")
    exit_gracefully(node)
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()