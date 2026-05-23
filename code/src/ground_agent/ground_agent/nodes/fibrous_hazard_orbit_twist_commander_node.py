import math, rclpy, os, subprocess, datetime, signal, time
from rclpy.node import Node
from nav_msgs.msg     import Odometry
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from shared_interfaces.srv import OrbitHazard
from shared_infrastructure.utils import start_rosbag, stop_rosbag, exit_gracefully, parse_hazard_type
from ground_agent.orbit_params_loader import OrbitParamsLoader


LOG_EVERY_N = 5        # publish log every N timer ticks

BAG_DIR           = "/tmp/hazard_orbit"
BAG_TOPICS = [
    "/tf",
    "/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/image",
    "/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/camera_info",
    "/tf_static"
]
QOS_FILE = "/root/ros2_ws/src/ground_agent/config/tf_qos.yaml"
# ----------------------------------------------------------------


class FibrousHazardOrbitTwistCommanderNode(Node):
    def __init__(self):
        super().__init__("fibrous_hazard_orbit_twist_commander_node")
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sub = self.create_subscription(Odometry, '/odom',
                                            self._odom_cb, 10)
         # tf buffers
        self.tf_buf = tf2_ros.Buffer(cache_time = rclpy.duration.Duration(seconds=5))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.timer = None

        self.srv = self.create_service(OrbitHazard, 'orbit_fibrous_hazard', self.perform_orbit)
        self.orbit_complete_pub = self.create_publisher(Bool, 'orbit_complete', 1)

        self.pos  = None
        self.yaw  = None
        self.bear_prev = None
        self.accum = 0.0
        self.mode = "RADIAL_IN"          # RADIAL_IN → TANGENT_ALIGN → ORBIT
        self.tick = 0
        self.yaw_des = 0.0
        self.hazard_cx = None
        self.hazard_cy = None
        self.radius = None
        self.orbit_params = None
        self.bag_proc  = None    # handle returned by _start_rosba
        self.hazard_type = None

        self.get_logger().info("🧪 Fibrous Hazard Orbit Twist Commander Node ready.")

        

    def perform_orbit(self, request, response):
        try:
            if self.timer is None:
                self.hazard_cx = request.hazard_center_point.x
                self.hazard_cy = request.hazard_center_point.y
                self.hazard_type = parse_hazard_type(request.hazard_type)
                estimated_hazard_diameter = request.estimated_hazard_diameter
                self.orbit_params = OrbitParamsLoader.calc_and_get_orbit_params(self.hazard_type, estimated_hazard_diameter)
                # self.orbit_params = self.orbit_params.orbit_params
                cmd_rate_hz = self.orbit_params["cmd_rate"]
                self.set_radius(estimated_hazard_diameter)
                self.timer = self.create_timer(1.0/cmd_rate_hz, self._loop)
                self.get_logger().info(
                f"⚠  hazard @ ({self.hazard_cx:.2f},{self.hazard_cy:.2f}) m – radius {self.radius:.2f} m\n"
                "   starting in RADIAL_IN" )
            response.success = True

            return response
        except Exception as ex:
            self.get_logger().info(f"Some exception occured: {ex}")

    def set_radius(self, estimated_hazard_diameter):

        patch_half_diag = math.hypot(estimated_hazard_diameter/2, estimated_hazard_diameter/2)

        # what the *base_link* must keep:
        self.radius = patch_half_diag + self.orbit_params["camera_clear"] + self.orbit_params["camera_offset"]

    
     # ---------- callbacks & helpers ---------------------------------
    def _odom_cb(self, msg): 
        
        ps = PoseStamped()
        ps.header    = msg.header
        ps.pose      = msg.pose.pose
        
        try:
            tfm = self.tf_buf.lookup_transform(
                      'map',                           # target frame
                      msg.child_frame_id,              # source frame = base_link
                      rclpy.time.Time())
            
            
        except (LookupException, ConnectivityException, ExtrapolationException):
            # self.get_logger().info("in odom_cb failed")
            return         

        # self.get_logger().info("in odom_cb - obtained tfm")
        self.pos = (tfm.transform.translation.x, tfm.transform.translation.y)
        self.yaw = self._quat_to_yaw(tfm.transform.rotation)                          

    # ---------------- main loop -----------------------------------
    def _loop(self):
        if self.pos is None:
            return
        self.tick += 1

        # only intially
        if self.tick == 1:
            initial_r = math.hypot(self.pos[0] - self.hazard_cx,
                        self.pos[1] - self.hazard_cy)
            self.get_logger().info(f"Initial radial distance = {initial_r:.3f} m")

        # geometry
        dx = self.pos[0] - self.hazard_cx
        dy = self.pos[1] - self.hazard_cy
        r  = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)          # CCW from +X
        heading_in  = self._wrap(bearing + math.pi)   # R → C  (inwards) - towards centre
        heading_out = self._wrap(bearing)             # away from centre

        # --------- desired yaw by mode ----------
        if self.mode == 'RADIAL_IN':
            self.yaw_des = heading_in
        elif self.mode == 'RADIAL_OUT':
            self.yaw_des = heading_out
        else:                                      # ORBIT / ALIGN
            self.yaw_des = bearing - math.pi/2

        yaw_err = self._wrap(self.yaw_des - self.yaw) 

        if self.tick < 10:          # once per second
            self.get_logger().info(
                f"RAW  pos=({self.pos[0]:+.2f},{self.pos[1]:+.2f})  "
                f"r={r:4.2f}  err={r - self.radius:+4.2f}"    )

        
        # sign flip because CW is positive for us
        radial_error = r - self.radius          # + outside, – inside
         # If we were told to go RADIAL_IN but we are already inside,
        # flip to RADIAL_OUT automatically (and vice‑versa).
        if self.mode == 'RADIAL_IN' and radial_error < 0:
            self.mode = 'RADIAL_OUT'
        elif self.mode == 'RADIAL_OUT' and radial_error > 0:
            self.mode = 'RADIAL_IN'

          # ---------------- MODE : ALIGN --------------------------
        if self.mode == "RADIAL_IN":
           return self.do_radial_in(radial_error, yaw_err)
        
         # --------------- MODE : APPROACH (from INSIDE) ---------------
        if self.mode == "RADIAL_OUT":
           return  self.do_radial_out(radial_error, yaw_err)

        # ---------------- MODE : TANGENT_ALIGN -----------------------
        if self.mode == "TANGENT_ALIGN":
          return  self.do_tangent_align(yaw_err, bearing)
            
        # ---------------- MODE : ORBIT --------------------------
        # integrate clockwise rotation
        self.orbit(yaw_err, bearing, r)


    def do_radial_in(self, radial_error, yaw_err):
        if abs(radial_error) < self.orbit_params["approach_tol"]:
            self.mode = "TANGENT_ALIGN"
            self.get_logger().info("RADIAL‑IN complete → TANGENT_ALIGN")
            return

        fwd_gain = max(0.0, math.cos(yaw_err)) * max(0.0, radial_error)       # 1 when on‑beam, 0 when ±90 °
        twist = Twist()
        twist.linear.x  = self.orbit_params["approach_speed"] * fwd_gain
        twist.angular.z = self.orbit_params["k_head_p"] * yaw_err
        self.pub.publish(twist)

        if self.tick % LOG_EVERY_N == 0:
            self.get_logger().info(
                f"{self.mode:13s} | r_err {radial_error:+.3f} m | "
                f"yaw_err {math.degrees(yaw_err):+5.1f}° | "
                f"v {twist.linear.x:+.2f} | ω {twist.angular.z:+.2f}")

        return
    
    def do_radial_out(self, radial_error, yaw_err):
        if abs(radial_error) < self.orbit_params["approach_tol"]:
            self.mode = "TANGENT_ALIGN"
            self.get_logger().info("RADIAL-OUT complete → TANGENT_ALIGN")
            return

        out_error = -radial_error                     # make it positive
        fwd_gain = max(0.0, math.cos(yaw_err)) * max(0.0, out_error)
        twist = Twist()
        twist.linear.x  = self.orbit_params["approach_speed"] * fwd_gain
        twist.angular.z = self.orbit_params["k_head_p"] * yaw_err
        self.pub.publish(twist)

        if self.tick % LOG_EVERY_N == 0:
            self.get_logger().info(
                f"{self.mode:13s} | r_err {radial_error:+.3f} m | "
                f"yaw_err {math.degrees(yaw_err):+5.1f}° | "
                f"v {twist.linear.x:+.2f} | ω {twist.angular.z:+.2f}")
        return

    def do_tangent_align(self, yaw_err, bearing):
        if abs(yaw_err) < math.radians(4):
              # >>>>>>>  START BAG JUST BEFORE ORBIT  <<<<<<<
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_dir = os.path.join(BAG_DIR, f"bag_{stamp}")
            self.get_logger().info(f"TANGENT_ALIGN → Starting Ros Bag...{bag_dir}")
            self.bag_proc = start_rosbag(BAG_TOPICS, outdir=bag_dir, qos_file = QOS_FILE)   
            self.get_logger().info("TANGENT_ALIGN → Ros Bag started...")
            self.mode = "ORBIT"
            self.accum = 0.0
            self.bear_prev = bearing
            self.get_logger().info("TANGENT_ALIGN complete → ORBIT")
            return

        twist = Twist()
        twist.angular.z = 0.8 * math.copysign(1, yaw_err)   # pure spin
        self.pub.publish(twist)

        if self.tick % LOG_EVERY_N == 0:
            self.get_logger().info(
            f"{self.mode:13s} |  "
            f"yaw_err {math.degrees(yaw_err):+5.1f}° | "
            f"v {twist.linear.x:+.2f} | ω {twist.angular.z:+.2f}")
        return

    def orbit(self, yaw_err, bearing, r):
        if hasattr(self, 'bear_prev'):
            dtheta = self._wrap(bearing - self.bear_prev)
            self.accum += -dtheta              # CW positive
        self.bear_prev = bearing

        if self.accum >= 2*math.pi:
            self.pub.publish(Twist())          # stop once
            self.get_logger().info("✔  Orbit complete – exiting.")
            stop_rosbag(self.bag_proc)
            self.timer.cancel()
            msg = Bool()
            msg.data = True
            self.orbit_complete_pub.publish(msg)
            self.get_logger().info("Orbit completed & result published")
            return

        # --------------------------------------------------------
        # FEED‑FORWARD + RADIAL‑P CONTROLLER  (CW orbit, ROS +z is CCW)
        # --------------------------------------------------------
        
        radial_error = r - self.radius           # + if we’re outside, – if inside
        
        # 1. SCALE SPEED  (slow if |r_err| > SLOW_ZONE)
        base_v = self.orbit_params["base_v"] 
        scale = max(0.4, 1.0 - abs(radial_error)/self.orbit_params["slow_zone"])
        v_lin = base_v * scale

        # 2. FEED‑FORWARD + RADIAL‑P   (compute *curvature*, then ω)
        curv_ff = 1.0 / self.radius                     # ideal curvature
        curv_p  = self.orbit_params["k_radial"] * radial_error / self.radius # P correction
        
        # *** keep full steering authority even when creeping ***
        w_cmd = -(curv_ff + curv_p) * base_v      # CW negative → minus
        max_w = self.orbit_params["max_w"]
        w_cmd = max(min(w_cmd,  max_w), -max_w)  # saturate safely

        twist = Twist()
        twist.linear.x  = v_lin
        twist.angular.z = w_cmd
        self.pub.publish(twist)
        

        if self.tick % LOG_EVERY_N == 0:
            self.get_logger().info(
                f"{self.mode:13s} | r_err {radial_error:+.3f} m | "
                f"yaw_err {math.degrees(yaw_err):+5.1f}° | "
                f"v {twist.linear.x:+.2f} | ω {twist.angular.z:+.2f}")


    

    @staticmethod
    def _wrap(angle):
        """Wrap to (-pi, pi]"""
        while angle >  math.pi: angle -= 2*math.pi
        while angle <= -math.pi: angle += 2*math.pi
        return angle
    
    @staticmethod
    def _quat_to_yaw(q):
        # convert quaternion to yaw
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        return math.atan2(siny, cosy)


# ── entry point ───────────────────────────────────────────────────
def main():
    rclpy.init()
    node = FibrousHazardOrbitTwistCommanderNode()
    node.get_logger().info("✅ fibrous_hazard_orbit_twist_commander_node main() started")
    exit_gracefully(node)

if __name__ == "__main__":
    main()
