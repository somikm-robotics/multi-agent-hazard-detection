# aerial_agent/nodes/crazy_flie_patrol_node.py
import rclpy
from rclpy.node import Node
from shared_infrastructure.utils import exit_gracefully
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import yaml, math, time, traceback
from pathlib import Path
from shared_interfaces.msg import BaseReturn
from tf_transformations import euler_from_quaternion     # pip install tf‑transformations if needed
from shared_interfaces.srv import ClassifyHazard, DetectHazard, DetectToxicGas, ScanIncident
from shared_interfaces.msg import ToxicityResult
from rclpy.task import Future

from sensor_msgs.msg   import Image
from cv_bridge         import CvBridge
import os, math, cv2, numpy as np
# ------------------ CAPTURE CONFIG --------------------

LOW_SCAN_ALT_DROP  = 3.0         # metres to descend from cruise h
HAZARD_RADIUS      = 0.6          # m  (1.2 m diameter / 2)
ORBIT_CLEARANCE    = 0.4          # m  (tweak as you like)
ORBIT_RADIUS       = HAZARD_RADIUS + ORBIT_CLEARANCE   # 1.0 m


WAYPOINT_FILE = '/root/ros2_ws/src/aerial_agent/config/patrol_waypoints.yaml'  

WP_NUMBER = 7
# 
CRUISE_ALT = 4.0 # [m]
LIN_VEL_MAX     = 0.60    # [m/s] horizontal speed limit
Z_VEL_GAIN      = 0.80    # [%]   simple P-controller gain for altitude
HOVER_STEPS     = 40      # ×0.1 s timer ticks ≈ 2 s
MAX_Z_VEL  = 0.3         # vertical speed clamp (m/s)

# ─────────  RISK / RTL tuning only ─────────
RTL_TOPIC        = "/unsafe_return_to_base"

# ─────────  RTL gains & landing  ─────────
KP_XY = 0.6                   # m s‑1 per metre of horizontal error
KP_Z  = 0.8                   # m s‑1 per metre of vertical error
KP_YAW  = 1.0
BASE_HOVER_Z = 0.4            # height (m) to aim for over the pad before flare

VEL_MAX   = 1.5         # absolute velocity limit (m s‑1)
HOVER_Z   = 0.40        # hold this height while translating home
XY_TOL    = 0.25        # switch‑to‑descent band (metres)
Z_TOL     = 0.10        # landing‑complete band  (metres)
LAND_STEPS = 50         # at 20 Hz ≈ 2.5 s
RETURN_ALT      = 6.0     # metres – same as patrol
CLOSE_DIST_XY   = 0.6     # when XY error < 60 cm we can start to drop

HOVER_DIST_TOL = 0.40     # used during take‑off settle
LAND_DIST_TOL  = 0.30     # tighter band for the final landing

HOVER_ALT_BAND = 0.15     # z‑band around target after take‑off
LAND_ALT_BAND  = 0.20     # z‑band considered “on the ground”

BASE_LAND_Z      = 0.15      # final touchdown height (m)
DESCEND_VEL      = 0.30      # steady vertical descent speed (m/s)

Z_VEL_GAIN       = 1.0
MAX_Z_VEL        = 0.6

# DESCENT_Z_GAIN    = 0.2    # P‑gain for vertical speed during descent
# TOUCHDOWN_BAND    = 0.05    # m  – when |dz| < band we’re on the pad
EPS_DIST          = 1e-3    # small value to avoid div‑by‑zero in XY normalisation

MAX_DESCENT_VEL  = 0.30          # stays the same above 0.25 m
# SOFT_DESCENT_VEL = 0.15          # m/s – cap vertical speed near ground
FINAL_HOVER_Z    = 0.40          # pause height
HOVER_TICKS      = 30            # 3 s pause
RTB_XY_TOL       = 0.25          # consider “over base” at 25 cm radius
RTB_SLOW_RADIUS  = 1.00          # start slowing XY when this close
BASE_Z_FLOOR       = 0.02     # m   never descend below this
# SOFT_TOUCH_CLEAR   = 0.015    # m   stop a little above ground

BASE_YAW        = 0.0          # rad   (point “north”, or whatever you prefer)
KP_YAW          = 1.2          # gain  (rad s⁻¹ per rad error, keep small)

SAFE_LIFT_Z      = 0.25   # m  – target height for the pre‑lift
LIFT_Z_VEL       = 0.4    # m/s up
LIFT_ALT_BAND    = 0.05   # m  – we’re “high enough” if |dz| < band

# landing-specific gains / limits
DESCENT_Z_GAIN   = 0.12     # gentler proportional term
SOFT_DESCENT_VEL = 0.08     # cap vertical speed to 8 cm/s
SOFT_TOUCH_CLEAR = 0.12     # start flare 12 cm above pad
TOUCHDOWN_BAND   = 0.03     # within ±3 cm → consider landed

PIX_ERR_TOL   = 8          # px, stop when bbox centre is this close to image centre
XY_P_GAIN     = 0.04       # m/s per pixel of error  (tune ≤ 0.05 to stay gentle)

LOW_SCAN_XY_GAIN = 0.4          # much gentler than cruise gain
LOW_SCAN_MAX_VXY = 0.15         # m/s cap while centring
LOW_SCAN_DROP_M = 3.0           # descend 3 m below current cruise height
LOW_SCAN_DZ_BAND = 0.05         # ±5 cm dead-band for “at target_z”
# LOW_SCAN_Z_GAIN  = 0.4          # vertical P-gain (gentler than cruise gain)
# LOW_SCAN_MAX_VZ  = 0.4          # cap descent to 0.4 m/s

# constants used below (tune if needed)
# LOW_SCAN_Z_GAIN   = 0.5        #    : P-gain for z–control
# LOW_SCAN_MAX_VZ   = 0.25       # m/s: cap descent speed

XY_DEADBAND       = 0.05       # m  : “close enough” horiz distance
# XY_P_GAIN         = 0.4        #    : P-gain for xy centring
# XY_VEL_CAP        = 0.15       # m/s: cap xy speed

LOW_SCAN_SAFETY_Z  = 0.20      # m  – never descend below this clearance
# LOW_SCAN_DZ_BAND   = 0.05      # m  – when |dz| < band we are “at altitude”

XY_CENTER_BAND     = 0.05      # m  – when |dx,dy| < band we are centred
XY_P_GAIN          = 1.2       # proportional gain for centring
XY_VEL_CAP         = 0.4       # m s-1  – limit horizontal speed

LOW_SCAN_Z_GAIN    = 0.6       # P-gain for vertical descent
LOW_SCAN_MAX_VZ    = 0.3       # m s-1  – cap vertical speed

MAX_CENTER_TRIES   = 10            # 120 × ~0.1 s ≈ 12 s wall-clock
MIN_IMPROVE_EPS    = 1e-3           # need at least 1 mm improvement to “count”
# … keep the rest of your gains (XY_CENTER_BAND, LOW_SCAN_Z_GAIN, …)
# altitude-hold & climb
# ───────── post-low-scan stabiliser ────────────────────────────────
POST_SCAN_HOLD_STEPS = 20        # 20 × 0.1 s  ≈ 2 s
POST_SCAN_MAX_VEL    = 0.05      # m/s  cap on XY drift during hold
# at the top of your node (together with "TAKEOFF", "PATROL", …)
RESTORE_BAND     = 0.10      # m   – done when |dz| < band
RESTORE_MAX_VZ   = 0.5       # m/s – climb no faster than this
RESTORE_Z_GAIN   = 0.6       # simple P gain

# patrol heights (metres, above world frame “map”)
# constants – tune to taste
MAX_ROUNDS          = 3          # stop lowering after this many laps
LOWER_CRUISE_ALT    = 4.0  

DETECT_STABILITY_TICKS = 5   # must be calm for 5 timer ticks (~0.5 s)
XY_VEL_STILL_EPS       = 0.02  # m/s regarded as “still”
Z_VEL_STILL_EPS        = 0.02  # m/s regarded as “st

# --- centring refinement -------------------------------------------------
PX_TO_M_LIMIT = 0.30          # never shift inspect-XY by more than this [m] per step
REFINE_GAIN   = 0.80          # <1 ⇒ move most of the way, but not the whole way

LOW_DESCENT_ALT   = 3.0       # m   target altitude for low‐level scan
LOW_DESC_Z_GAIN   = 0.12      # P‐gain for gentle descent
LOW_DESC_VEL_MAX  = 0.20      # m/s hard cap while dropping
DET_ALT_BAND      = 3.00      # m   fire detector once inside ±3 cm



# CAPTURE_DIR = "/root/ros2_ws/src/aerial_agent/data/fibrous_detection_captures"

# ---- low-alt capture yaw sweep ----
YAW_SWEEP_RATE          = 0.12   # rad/s  (gentle)
YAW_SWEEP_SWITCH_TICKS  = 10     # at 10 Hz → flip direction every 1 s
CAPTURE_WARMUP_TICKS    = 5      # wait a short moment before saving first frame

CAPTURE_TARGET_COUNT   = 50      # images per batch
CAPTURE_EVERY_N_TICKS  = 1       # save every timer tick
CAPTURE_YAW_RATE       = 0.06    # rad/s; set 0.0 to disable tiny yaw sweep
CAPTURE_YAW_EVERY_N    = 12      # apply yaw every N ticks during capture

RGB_CAMERA_TOPIC   = "/world/mining_world/model/crazyflie/model/vga_camera/link/vga_camera/sensor/down_cam/image"

FLARE_START_ALT        = 0.40   # m above pad where we switch to "near" behavior
DESCENT_Z_GAIN_FAR     = 0.35   # P-gain when higher than FLARE_START_ALT
DESCENT_Z_GAIN_NEAR    = 0.18   # softer P-gain near the pad
DESCENT_VEL_FAR        = 0.35   # m/s cap when high
DESCENT_VEL_NEAR       = 0.12   # m/s cap very close to pad
TOUCHDOWN_SETTLE_TICKS = 12     # ticks to hold zero before disarm (≈1.2 s @10 Hz)
VZ_SLEW = 0.05   # m/s per tick (≈0.5 m/s² at 10 Hz). Lower = softer.


# ---- background capture params ----
BG_CAPTURE_DIR     = "/root/ros2_ws/src/aerial_agent/data/backgrounds"   # or wherever you want
BG_TARGET_COUNT    = 500
BG_PER_WP_COUNT         = 15          # ← per-waypoint limit
BG_EVERY_N_TICKS   = 5              # sample every N patrol ticks
BG_CAPTURE_TARGET_COUNT  = 500              # frames you want in total
BG_CAPTURE_EVERY_N_TICKS = 5                # save 1 frame every N ticks
BG_YAW_RATE              = 0.05             # rad / s (≈ 3 °/s)   0 → no dither
BG_YAW_EVERY_N           = 50               # give a tiny nudge every ~5 s
HAZARD_WP_NUMBERS = (6, 8)

DET_MAX_ATTEMPTS      = 5     # try up to N times at low-alt before giving up
DET_RETRY_WAIT_TICKS  = 5     # ~0.5 s if your loop is 10 Hz

WARMUP_HOVER_TICKS = 2

class CrazyfliePatrolNode(Node):
    def __init__(self):
        super().__init__('crazyflie_patrol_node')
        
        self.declare_parameter('waypoint_file', WAYPOINT_FILE)
        wp_file   = Path(
            self.get_parameter("waypoint_file").get_parameter_value().string_value
        )
        self.h    = CRUISE_ALT

        self.waypoints = self._load_waypoints(wp_file)
        if not self.waypoints:
            self.get_logger().fatal(f"❌ No waypoints found in {wp_file}")
            rclpy.shutdown()
            return
        self.get_logger().info(f"📄 Loaded {len(self.waypoints)} waypoints")


         # ── Publishers / Subscribers ─────────────────────────────────────────
        self.arm_pub = self.create_publisher(Bool, '/crazyflie/enable', 10)
        self.cmd_pub    = self.create_publisher(Twist, "/crazyflie/gazebo/command/twist", 1)
        self.odom_sub   = self.create_subscription(
            Odometry, "/model/crazyflie/odometry", self._odom_cb, 10
        )
        self.timer      = self.create_timer(0.10, self._loop_safe)   # 10 Hz
        
        
        self.return_pub  = self.create_publisher(String, '/unsafe_returning_to_base', 10)

        self.scan_now_return_pub = self.create_publisher(String, 'scan_now_response', 10)

        self.cls_cli  = self.create_client(ClassifyHazard, "classify_hazard")
        self.det_cli  = self.create_client(DetectHazard , "detect_hazard")

        self.gas_cli  = self.create_client(DetectToxicGas , "/detect_toxic_gas")

        self.create_service(ScanIncident, 'scan_incident', self.scan_incident_handler)

        # self.create_subscription(
        #     ToxicityResult,
        #     "/crazyflie/gas/ppm",
        #     self._gas_cb,
        #     10)

        self.create_subscription(BaseReturn, 'return_to_base_status', self._mission_status,  10)

        self.pose = None                    # latest Odometry pose
        self.i = 0                # current waypoint index
        self.state       = "INITIAL"        # INITIAL -> TAKEOFF → PATROL
        self.hover_count = 0                # ticks to pause at waypoint
        self.hovering = False
        # self.bridge = CvBridge()
        # self.image_frame  = None
        self.base_xy = None          # will be set from first odom
        self.base_z = None
        self.yaw = 0.0
        
         # ───────── gas + safety ─────────
        self.land_count = 0
        self.q = Quaternion()   # geometry_msgs.msg.Quaternion()

        self.original_h   = self.h      # remember nominal cruise alt
        self.low_scan_in_progress = False
        self.suspicious_hazard    = False
        self.target_z = None
        self.mission_in_progress = False
        self.checking_for_hazard = False
        # ── state helpers (set once, outside the method, e.g. in __init__) ───────
        self._center_tries      = 0         # how many refine steps we’ve attempted
        self._last_xy_error     = None      # last distance to inspect point
        self.z_last = 0.0
        self._still_ticks = 0


        # self.level_idx     = 0              # index in CRUISE_LEVELS
        # self.h             = CRUISE_LEVELS[self.level_idx]
        # self.hazard_found  = False          # set True in _on_det_done()
        self.hazard_published = False
        self.round_counter = 0

        # self.bridge   = CvBridge()
        # self.latest_cv = None
        self.latest_header  = None   
        # self.create_subscription(Image, RGB_CAMERA_TOPIC, self.image_cb, 10)

        self.bg_count     = 0
        self.bg_tick      = 0
        self._bg_last_xy  = None
        self._bg_last_hash = None
        self.is_unsafe = False

        self._arm(True)

        self.get_logger().info('🚁 Crazyflie patrol node started.')

    # def _gas_cb(self, msg: ToxicityResult):
    #     self.get_logger().info("In gas callback")
    #     # keep the most recent reading
    #     self.latest_gas = msg
    #     status = self.latest_gas.status
    #     self.get_logger().info(f"status={status}")

    def _arm(self, enable: bool):
        msg = Bool()
        msg.data = enable
        state = "armed" if enable else "disarmed"
        self.arm_pub.publish(msg)
        self.get_logger().info(f"Crazyflie {state}")
        time.sleep(0.5)  # Sleep to ensure message delivery before movement starts

    def _load_waypoints(self, path):
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)
            return [(float(w["x"]), float(w["y"])) for w in data["waypoints"]]
        except Exception as err:
            self.get_logger().error(f"Failed to load {path}: {err}")
            return []


    def _odom_cb(self, msg):
        self.pose = msg.pose.pose
         # orientation → roll, pitch, yaw
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def _mission_status(self, msg):
        if msg.success:
            self.mission_in_progress = False

    def scan_incident_handler(self, request, response):
        """
        Insert the requested (x,y) into the waypoint list if we are in a
        safe state to accept it.
        """
        try:
            response_msg = String()
            # 1. Only accept while patrolling and not already landing / scanning
            if self.state not in ("PATROL", "HOVER") \
            or getattr(self, "mission_in_progress", False) \
            or getattr(self, "low_scan_in_progress", False):
                response_msg.data = "🚫 Busy – scan request rejected"
                self.get_logger().warn(response_msg.data)
                self.scan_now_return_pub.publish(response_msg)
                response.accepted = False
                return response

            location = request.incident_point

            self.get_logger().info(f"🛰️ Scan incident request received for: {location.x} {location.y}")

            new_wp = (round(location.x, 2), round(location.y, 2))     # tidy small decimals


            # 2. Skip if that point is already in the plan (tolerance ≈ 5 cm)
            if any(math.hypot(wx - new_wp[0], wy - new_wp[1]) < 0.05
                for wx, wy in self.waypoints):
                response_msg.data = f"ℹ️ Waypoint {new_wp} already present"
                self.get_logger().info(response_msg.data)
                self.scan_now_return_pub.publish(response_msg)
                response.accepted = False
                return response

            # 3. Insert the point *after* the current one for minimal detour
            insert_at = (self.i + 1) % len(self.waypoints)
            self.waypoints.insert(insert_at, new_wp)
            response_msg.data = f"✅ Queued on-demand scan @ {new_wp}, index={insert_at}"
            self.get_logger().info(response_msg.data)
            self.scan_now_return_pub.publish(response_msg)

            response.accepted = True
            return response
        except Exception as ex:
            self.get_logger().info(f"Some exception occurred: {ex}")
    
        
    # helper that swallows all exceptions so the timer never dies
    def _loop_safe(self):
        try:
            self._loop()
        except Exception as err:
            self.get_logger().error(f'💥 Exception in _loop: {err}')
            traceback.print_exc()

    def _loop(self):
        # ── Wait for first odometry ───────────────────────────────────────────
        if self.pose is None:
            self.get_logger().info('⏳ Waiting for odometry…')
            return

        x, y, z = self.pose.position.x, self.pose.position.y, self.pose.position.z
        cmd = Twist()
        self.z_last = z
        # self.get_logger().info(f"Obtained {x}, {y}, {z}")
        
        if self.state == "INITIAL":
            self._lift_off_ground(x, y, z)    
            if self.base_xy is None:
                self.base_xy = (x, y)
                self.base_z = max(z, BASE_Z_FLOOR)
                self.get_logger().info(f"Base XY: {self.base_xy} Base Z: {self.base_z}" )

            self.state = "TAKEOFF"
            self.get_logger().info('Taking Off…')

         # ── TAKE‑OFF ───────────────────────────────────────────────────────────
        if self.state == "TAKEOFF":
            self._takeoff(cmd, z)
            return
        
        if self.state == "LANDING":
            return self._land(x, y, z)

        if self.state == "LOW_SCAN":
            if not self.low_scan_in_progress:
                self._start_low_descent(z)
            self._low_descent_for_detection(x, y, z)
            return
        
        if self.state == "HOVER":
            self._hover()                    
            return
        
        # if self.state == "CAPTURE_PIC":
        #     self._capture_background()
        #     return
                
        if self.state == "RESTORE_ALT":
            self._restore_alt(z)
            return

        self._patrol(cmd, x, y, z)
        


    def _lift_off_ground(self, x, y, z):
        """
        Pre‑lift: nudge the quad to SAFE_LIFT_Z before the real take‑off.
        """
        dz = SAFE_LIFT_Z - z             # how far we still need to climb
        cmd = Twist()

        if abs(dz) < LIFT_ALT_BAND:      # we’re high enough – switch state
            self.get_logger().info("🟢 Lift phase complete – starting take‑off")
            self.state = "TAKEOFF"
            return                       # don’t publish, let TAKEOFF do it

        # ascend gently, no XY motion
        cmd.linear.z = LIFT_Z_VEL
        self.cmd_pub.publish(cmd)

        
    def _takeoff(self, cmd, z):
        dz = self.h - z
        if abs(dz) < HOVER_DIST_TOL:                          # reached cruise alt
            self.state = "PATROL"
            self.get_logger().info("🟢 Take‑off complete – starting patrol")
            return
        cmd.linear.z = max(min(Z_VEL_GAIN * dz, MAX_Z_VEL), -MAX_Z_VEL)
        self.cmd_pub.publish(cmd)
        return

    def _patrol(self, cmd, x, y, z):
        # ── PATROL ─────────────────────────────────────────────────────────────
        gx, gy  = self.waypoints[self.i]
        dx, dy  = gx - x, gy - y
        dz      = self.h - z
        dist_xy = math.hypot(dx, dy)

        # Arrived at waypoint?  , log once, then advance
        if dist_xy < HOVER_DIST_TOL and abs(dz) < HOVER_ALT_BAND:
             # ---------- HAZARD WAY-POINT? ----------
            if self.hover_count == 0: 
                self.get_logger().info(
                f"✅ Reached wp {self.i + 1}/{len(self.waypoints)} "
                f"at ({x:.2f}, {y:.2f}, {z:.2f})"
                )

            # start a single hazard check and switch to HOVER state
            if not self.checking_for_hazard: 
                self.state = "HOVER"            # <- next loop tick goes to _hover
            return                            # no motion command this tick
  

        # Horizontal velocity command
        if dist_xy > 1e-3:
            cmd.linear.x = LIN_VEL_MAX * dx / dist_xy
            cmd.linear.y = LIN_VEL_MAX * dy / dist_xy

        # Altitude hold with dead‑band
        # cmd.linear.z = 0.0        # hold altitude: no vertical velocity command

        if abs(dz) > HOVER_ALT_BAND:
            cmd.linear.z = max(min(Z_VEL_GAIN * dz, MAX_Z_VEL), -MAX_Z_VEL)
        else:
            cmd.linear.z = 0.0                              # inside band → neutral

        self.cmd_pub.publish(cmd)

   #  ------------ warm-up & classification ---------------------
    def _hover(self):
         # hold still
        self.cmd_pub.publish(Twist())
        
        if self.hover_count >= WARMUP_HOVER_TICKS and self.is_unsafe != True:
            self._check_for_toxicity()
        
        # advance the tick counter
        self.hover_count += 1


    def _hover_done(self):
        # self.get_logger().info(f"In hover done for wp {self.i + 1}...Is suspicious {self.suspicious_hazard} ")
        self.hover_count = 0
        self.checking_for_hazard = False
        if self.suspicious_hazard:
            self.state = "LOW_SCAN"            # begin the descent
            # self._start_low_descent()
        else:
            # advance to next waypoint and resume patrol
            # self.i = (self.i + 1) % len(self.waypoints)
            self._next_wp()
            self.state = "PATROL"
    
    def _next_wp(self) -> None:
        """Advance waypoint index. Detect lap completion → maybe lower altitude."""
        prev_i = self.i
        self.i = (self.i + 1) % len(self.waypoints)

        # 1) normal housekeeping for every hop
        self.hover_count = 0
        self.hovering    = False

        # 2) did we just wrap from last wp back to wp-0?
        if self.i == 0 and prev_i == len(self.waypoints) - 1:
            # increment lap counter (create the attribute on first use)
            self.round_counter = getattr(self, "round_counter", 0) + 1

            self.get_logger().info(
                f"🔄 Completed lap {self.round_counter}"
            )

            # Only lower cruise height if *no* hazard published yet
            if (not self.hazard_published
                    and self.round_counter == 1          # after first lap
                    and self.h > LOWER_CRUISE_ALT         # already low? skip
            ):
                self.h = LOWER_CRUISE_ALT
                self.get_logger().info(
                    f"⬇️  No hazards found – lowering cruise height to {self.h:.1f} m"
                )

            # Optional: stop mission after MAX_ROUNDS
            if self.round_counter >= MAX_ROUNDS and not self.hazard_published:
                self.get_logger().warn(
                    "🚧 Max laps reached with no detections – returning to base"
                )
                self.state = "LANDING"            # or any finish behaviour you use

    

    # during hover – classification first
    def _check_for_hazard(self):
        if not self.cls_cli.service_is_ready():
            self.get_logger().info("Classify Hazard service not available")
            return
        self.checking_for_hazard = True
        future = self.cls_cli.call_async(ClassifyHazard.Request())
        future.add_done_callback(self._on_cls_done)

    def _on_cls_done(self, fut):
        msg = fut.result()
        if msg and msg.hazard_type in ("dust_plume", "fibrous_hazard"):
            self.get_logger().info(f"⚠️  {msg.hazard_type} suspected – descending")
            self.suspicious_hazard = True   
        self.checking_for_hazard = False  
        self._hover_done()
    
    def _start_low_descent(self, z):
        self.low_scan_in_progress = True
        self.state = "LOW_SCAN" 
        # where you already run low-scan
        if not hasattr(self, "det_attempts"): self.det_attempts = 0
        if not hasattr(self, "det_cooldown"): self.det_cooldown = 0

        self.target_z   = max(z - LOW_SCAN_DROP_M, BASE_Z_FLOOR)
        self.original_h = self.h                   # remember cruise height
        self.get_logger().info("Descending...")

   
    def _low_descent_for_detection(self, x, y, z):
    

        cmd = Twist()
        # if hasattr(self, "_det_req_sent"):    
        #     self.cmd_pub.publish(cmd)
        #     return

        # ---------------- vertical control -----------------
        dz = self.target_z - z
        vz_cmd = LOW_DESC_Z_GAIN * dz
        cmd.linear.z = max(min(vz_cmd, LOW_DESC_VEL_MAX), -LOW_DESC_VEL_MAX)

        # freeze XY so we stay centred above the plume/fibres
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        self.cmd_pub.publish(cmd)

        # NEW: simple cooldown between attempts (camera settle / AE settle)
        if self.det_cooldown > 0:
            self.det_cooldown -= 1
            return


        # ---------------- fire detector once ---------------
        if abs(dz) < DET_ALT_BAND and not hasattr(self, "_det_fut")  and \
                self.det_attempts < DET_MAX_ATTEMPTS:   
            if self.det_cli.service_is_ready():
                req = DetectHazard.Request()
                req.x = x        # current drone XY (approx centre of FOV)
                req.y = y
                req.z = z
                self._det_fut = self.det_cli.call_async(req)
                self._det_fut.add_done_callback(self._on_det_done)
                self._det_req_sent = True     # sentinel
                self.get_logger().info("📸 Low-alt image captured → detector called")
    

    def _on_det_done(self, fut):
        if hasattr(self, '_det_fut'):
            delattr(self, '_det_fut')

        try:
            res = fut.result()
            self.get_logger().info(
            f"Got result: detected={res.detected} centred={res.centred} "
                f"haz='{res.hazard_type}' has published = '{res.has_published}'")
        except Exception as ex:
            self.get_logger().error(f"Detector crashed: {ex}")
            self._reset_low_scan()
            return
        
        # ------------------------------------------------------------------
        if not res or not res.detected:
            self.det_attempts += 1
            if self.det_attempts < DET_MAX_ATTEMPTS:
                self.det_cooldown = DET_RETRY_WAIT_TICKS
                self.get_logger().info(
                    f"❌ No hazard – retry {self.det_attempts}/{DET_MAX_ATTEMPTS}")
                return  # stay at low-alt, try again
            self.get_logger().info(f"❌ No hazard after {DET_MAX_ATTEMPTS} tries – resume patrol")
            self.det_attempts = 0
            self.det_cooldown = 0
            self._reset_low_scan()
            self.state = "RESTORE_ALT"
            # self.i = (self.i + 1) % len(self.waypoints)
            # self._next_wp()
            return

        # ---------- centred (or forced)  → finish -------------------------
        if res.has_published:
            self.get_logger().info("✅ Hazard pose published – landing")
            self.hazard_published = res.has_published
            self._reset_low_scan()
            self.state = "LANDING"
            return

        # ── first detector hit: store bbox offsets for centring ────────────
        if not self.have_bbox:
            self.have_bbox = True


    def _reset_low_scan(self):
        self.low_scan_in_progress = False
        self.suspicious_hazard    = False
        
        # if hasattr(self, 'inspect_x'):
        #     delattr(self, 'inspect_x');  delattr(self, 'inspect_y')

        # if hasattr(self, 'have_bbox'):
        #     delattr(self, 'have_bbox')

        if hasattr(self, '_det_req_sent'):
            delattr(self, '_det_req_sent')

        self.target_z  = None
        self.h         = self.original_h
        self._center_tries = 0

    def _restore_alt(self, z):
        dz = self.original_h - z                # need to climb if > 0
        if abs(dz) < RESTORE_BAND:
            # reached cruise height – resume patrol
            self.state = "PATROL"
            # self.i = (self.i + 1) % len(self.waypoints)   # advance to next wp
            self._next_wp()
            self.get_logger().info("↗️  Back at cruise – resuming patrol")
            return

        vz_cmd = min(max(RESTORE_Z_GAIN * dz, -RESTORE_MAX_VZ), RESTORE_MAX_VZ)
        cmd = Twist();  cmd.linear.z = vz_cmd
        self.cmd_pub.publish(cmd)


    # -------------------------------------------------------------------
        
    def _check_for_toxicity(self):
        if not self.gas_cli.service_is_ready():
            self.get_logger().info("Gas service not available")
            return
        # self.get_logger().info("Checking for toxicity")
        future = self.gas_cli.call_async(DetectToxicGas.Request())
        future.add_done_callback(self._on_gas_done)

    def _on_gas_done(self, fut):
        msg = fut.result()
        
        if msg.is_unsafe:
            self.is_unsafe = True
            self.initiate_return_land(msg.toxicity_status) 
        elif msg.toxicity_status == "Dangerous":
            self.get_logger().info(f"Toxicity Status: {msg.toxicity_status}. So not checking for any other hazards presently")
            self._hover_done()
        elif self.checking_for_hazard != True:
            # only now do we look for hazards
            self.checking_for_hazard = True
            self.get_logger().info(f"Checking for hazard at wp {self.i + 1} after warm up...")
            self._check_for_hazard()           # (unchanged helper)

    def initiate_return_land(self, reason: str):
        """
        Abort patrol, descend, and disarm.  Safe to call more than once.
        """
        if self.state == "LANDING" :
            return          # already in progress

        
        self.hovering  = False         # stop waypoint logic immediately
        self.state  = "LANDING" 
        

        # Inform supervisory UI
        self.return_pub.publish(String(
            data=f"Unsafe gas levels – {reason}. Returning to base."))

        self.get_logger().warn("🔻 Returning to base / landing – " + reason)
    
    def _land(self, x, y, z):
        gx, gy  = self.base_xy                  # set when take-off completed
        # gz       = self.base_z 
        dx, dy  = gx - x, gy - y
        dist_xy = math.hypot(dx, dy)


        # ── keep heading home while nose-forward ──────────────────
        yaw_err        = self._wrap_to_pi(BASE_YAW - self.yaw)      # uses latest odom yaw
        cmd = Twist()
        cmd.angular.z  = KP_YAW * yaw_err

        # ── PHASE 1 – horizontal return at cruise height (stay level, cruise speed) ────────────────────
        if dist_xy > RTB_XY_TOL:
            # stay level: cmd.linear.z = 0   (controller will hold current z)
            if dist_xy > EPS_DIST:
                cmd.linear.x = LIN_VEL_MAX * dx / dist_xy
                cmd.linear.y = LIN_VEL_MAX * dy / dist_xy
            # stay level; position controller holds z
            self.cmd_pub.publish(cmd)
            # self.get_logger().info(
            #     f"[RTB] → ({gx:.1f}, {gy:.1f})  dist={dist_xy:.2f} m"
            # )
            # actively hold current cruise height (prevents sag/fall)
            z_err = self.h - z
            cmd.linear.z = max(min(Z_VEL_GAIN * z_err, MAX_Z_VEL), -MAX_Z_VEL)

            # reset smoother so Phase 2 starts clean
            self.vz_smooth = 0.0
            self.cmd_pub.publish(cmd)
            return                     
        # ── PHASE 2 – flare / gentle descent ─────────────────────────────────────
        # 1. Target height = captured base‑z (never below pad floor) + clearance
        target_z = max(self.base_z, BASE_Z_FLOOR) + SOFT_TOUCH_CLEAR
        dz = target_z - z                             # < 0  → need to descend

        
        # two-stage flare: softer gain and lower cap near the pad
        alt_err  = abs(dz)
        if alt_err > FLARE_START_ALT:
            gain = DESCENT_Z_GAIN_FAR
            vcap = DESCENT_VEL_FAR
        else:
            gain = DESCENT_Z_GAIN_NEAR
            vcap = DESCENT_VEL_NEAR


        vz_cmd = gain * dz                 # proportional term
        # clamp vertical speed
        vz_cmd = max(min(vz_cmd, vcap), -vcap)

        # NEW: slew-limit the vertical command so it eases in (no “drop”)
        if not hasattr(self, "vz_smooth"):
            self.vz_smooth = 0.0
        dv = max(min(vz_cmd - self.vz_smooth, VZ_SLEW), -VZ_SLEW)
        self.vz_smooth += dv
        cmd.linear.z = self.vz_smooth

        # Slow any residual XY drift as we get really close
        cmd.linear.x = self._scaled_vel(cmd.linear.x, dist_xy, RTB_SLOW_RADIUS)
        cmd.linear.y = self._scaled_vel(cmd.linear.y, dist_xy, RTB_SLOW_RADIUS)

        # ── Touch-down settle ─────────────────────────────────────────────────
        if not hasattr(self, "land_settle_ticks"):
            self.land_settle_ticks = 0

        # ── Touch‑down check ───────────────────────────────────────────────
        if dist_xy < RTB_XY_TOL and alt_err < TOUCHDOWN_BAND:
            self.cmd_pub.publish(Twist())   # full stop
            self.land_settle_ticks += 1
            if self.land_settle_ticks >= TOUCHDOWN_SETTLE_TICKS:
                self._arm(False)                # disarm
                self.get_logger().info("🛬 Landed at base. Patrol stopped.")
                self.vz_smooth = 0.0
            # rclpy.shutdown()
            return
        else:
            # self.get_logger().info(f"x: {cmd.linear.x} y: {cmd.linear.y} z: {cmd.linear.z}")
            self.land_settle_ticks = 0
            self.cmd_pub.publish(cmd)       # keep settling        

    def _scaled_vel(self, v_cmd, dist_xy, radius) -> float:
        """
        Linearly scale XY velocity down inside `radius` so we don't
        skid sideways while descending.
        """
        if dist_xy < radius:
            return v_cmd * (dist_xy / radius)
        return v_cmd
    
    def _wrap_to_pi(self, a):
        """Wrap angle to [-π, π]."""
        return (a + math.pi) % (2*math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = CrazyfliePatrolNode()
    node.get_logger().info("✅ crazyflie_patrol_node main() started") 
    exit_gracefully(node)

if __name__ == '__main__':
    main()


