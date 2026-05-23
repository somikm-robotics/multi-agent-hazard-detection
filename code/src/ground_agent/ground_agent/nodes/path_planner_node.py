# path_planner/nodes/path_planner_node.py
import rclpy
from rclpy.node import Node
from shared_infrastructure.utils import exit_gracefully, get_agilex_base_pose, load_service_timeout_from_config
from rclpy.action import ActionClient, ActionServer

from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
from nav2_msgs.action  import ComputePathToPose
import math
from typing import List, Optional
from shared_interfaces.srv import RequestPathPlan, PathPlanResult


# ── helpers ────────────────────────────────────────────────────────────────────
def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))

def pose_dist(a: PoseStamped, b: PoseStamped) -> float:
    dx = a.pose.position.x - b.pose.position.x
    dy = a.pose.position.y - b.pose.position.y
    return math.hypot(dx, dy)

def euclid(a: PoseStamped, b: PoseStamped) -> float:
    dx = a.pose.position.x - b.pose.position.x
    dy = a.pose.position.y - b.pose.position.y
    return math.hypot(dx, dy)


def decimate(path: Path, spacing: float) -> List[PoseStamped]:
    if not path.poses:
        return []

    pruned = [path.poses[0]]
    for p in path.poses[1:]:
        if euclid(pruned[-1], p) >= spacing:
            pruned.append(p)

    if euclid(pruned[-1], path.poses[-1]) > 1e-3:
        pruned.append(path.poses[-1])
    return pruned

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        # Parameters
        self.declare_parameter("spacing", 2.0)

        # service
        self.path_plan_srv = self.create_service(RequestPathPlan, 'request_path_plan', self.plan)

        self.path_plan_client = self.create_client(PathPlanResult, 'receive_path_result')

        # Nav2 clients
        self._compute_cli = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose"
        )
        
        self.service_timeout = load_service_timeout_from_config()

        self.get_logger().info("Path Planner Node initialized")

    def plan(self, request, response):
        try:
            # self.get_logger().info("In Path plan node...planning")
            response.accepted = False
            self.is_path_return = request.is_return
            if not (self._compute_cli.wait_for_server(timeout_sec=self.service_timeout)):
                self.get_logger().info("Waiting for Nav2 action server …")
                return                    

            # self.get_logger().info("In Path plan node...planning after service available")
            goal_msg = ComputePathToPose.Goal()
            goal_msg.goal = self._make_pose(request.goal)
            goal_msg.use_start = False


            future = self._compute_cli.send_goal_async(goal_msg)

            future.add_done_callback(self._on_compute_goal_response)
            # self.get_logger().info("add callback …")
            response.accepted =  True
        except Exception as ex:
            self.get_logger().info(f"Exception raised - {ex} ")
        return response

    def _on_compute_goal_response(self, future):
        try:
            self.get_logger().info("In Compute goal response")
            goal_handle = future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error("ComputePathToPose goal rejected")
                return

            self.get_logger().info("ComputePathToPose goal accepted; waiting for result …")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_path_return)
        except Exception as ex:
            self.get_logger().info(f"Exception raised - {ex} ")
        

    def _make_pose(self, goal_pose):
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(goal_pose.position.x)
        pose.pose.position.y = float(goal_pose.position.y)
        pose.pose.position.z = float(goal_pose.position.z)
        pose.pose.orientation.x = float(goal_pose.orientation.x)
        pose.pose.orientation.y = float(goal_pose.orientation.y)
        pose.pose.orientation.z = float(goal_pose.orientation.z)
        pose.pose.orientation.w = float(goal_pose.orientation.w)

        return pose

    def _on_path_return(self, fut):
        
        try:
            if fut.exception():
                self.get_logger().error(f"Path service failed: {fut.exception()}")
                return 
            
            path: Path = fut.result().result.path
            self.get_logger().info(f"Nav2 returned {len(path.poses)} poses")

            spacing = self.get_parameter("spacing").get_parameter_value().double_value
            waypoints = decimate(path, spacing)

            # ── skip the first pose if we’re already there ──────────────
            if waypoints and pose_dist(waypoints[0], path.poses[0]) < 0.05:
                waypoints = waypoints[1:]

            self.get_logger().info(f"Down-sampled to {len(waypoints)} waypoints (≥{spacing} m)") 
        

            frame_id = path.header.frame_id or "map"
            now      = self.get_clock().now().to_msg()
            for p in waypoints:
                p.header.frame_id = frame_id   # ensure TF knows the frame
                p.header.stamp    = now        # optional but nice to have

            self.send_path_result(waypoints)
        except Exception as ex:
            self.get_logger().error(f"Some exception: {ex}")
        

    def send_path_result(self, waypoints):
        if not self.path_plan_client.wait_for_service(timeout_sec=self.service_timeout):
            self.get_logger().error("❌ path_plan result service not available.")
            return
        req = PathPlanResult.Request()
        req.waypoints = waypoints
        self.path_plan_client.call_async(req)
        return
        
        



def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    node.get_logger().info("✅ path_planner_node main() started") 
    exit_gracefully(node)

if __name__ == "__main__":
    main()

