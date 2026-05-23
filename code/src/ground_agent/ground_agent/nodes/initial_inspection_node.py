#!/usr/bin/env python3
import math, shlex, subprocess, rclpy
from rclpy.node  import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg  import Image            
from shared_infrastructure.utils import exit_gracefully, parse_hazard_type
from shared_interfaces.srv import  InitialInspection
from shared_infrastructure.hazard_types import HazardType
import os, time, cv2
from rclpy.qos  import qos_profile_sensor_data
from cv_bridge  import CvBridge
from nav_msgs.msg  import Odometry
from std_msgs.msg import Bool


CAMERA_TOPIC = "/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/image"
CMD_VEL_TOPIC = '/cmd_vel'
N_IMAGES = 10
SPIN_SPEED = 0.5
ANGLE_PER_IMAGE    = math.radians(30)    # 30 degrees per image
PHOTO_RATE = 2.0
OUT_DIR    = '/root/ros2_ws/src/ground_agent/data/initial_inspection'
PUB_HZ     = 10
IMG_FMT    = 'png'

class InitialInspectionNode(Node):
    # ──────────────────────────────────────────────────────────────────────────
    def __init__(self):
        super().__init__("initial_inspection_node")
        
        # ── Comms ────────────────────────────────────────────────────────────
        self.bridge  = CvBridge()
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.completion_pub = self.create_publisher(Bool, "inspection_complete", 10)
        
        self._srv = self.create_service(InitialInspection, "initial_inspection", self.start_spin_and_capture)
        self.create_subscription(Image, CAMERA_TOPIC, self._img_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)


        self.hazard_type = HazardType.NONE
        self._busy      = False
        self._img_msg = None
        self.spin_active = False
        self.out_dir = None
        self.images_captured = 0
        self.last_yaw = None
        self.spin_timer = None

        self.get_logger().info("🧪 Initial Inspection Node ready.")


    def _img_cb(self, msg):
        # self.get_logger().info("In img cb 0")
        # if self.spin_active:
        #     self.get_logger().info("In img cb 0")
        if not self.spin_active or self.images_captured >= N_IMAGES:
            return
        # self._img_msg = msg   # just cache the latest frame
        # Convert image
        # self.get_logger().info("In img cb")
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.current_yaw is None:
            return

        if self.last_yaw is None:
            self.last_yaw = self.current_yaw
            # self.get_logger().info("saving image 1")
            self._save_image(img)
            return
        
        # Compute angle difference (wrap-around safe)
        yaw_diff = abs((self.current_yaw - self.last_yaw + math.pi) % (2 * math.pi) - math.pi)
        if yaw_diff >= ANGLE_PER_IMAGE:
            self.last_yaw = self.current_yaw
            # self.get_logger().info("saving image 2")
            self._save_image(img)

    def odom_cb(self, msg):
        # msg: nav_msgs.msg.Odometry
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw (Euler Z)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def start_spin_and_capture(self, req, res):
        # Initialize state
        self.images_captured = 0
        self.last_yaw = None
        self.spin_active = True
        self.out_dir = os.path.expanduser(f'{OUT_DIR}/{req.hazard_type}/captured_images_{time.strftime("%Y%m%d_%H%M%S")}')
        os.makedirs(self.out_dir, exist_ok=True)
        self.get_logger().info(f'Capturing {N_IMAGES} images → {self.out_dir} for {req.hazard_type}')
        self.hazard_type = req.hazard_type

        # Start spinning
        self.twist = Twist()
        self.twist.angular.z = SPIN_SPEED
        if not hasattr(self, "spin_pub"):
            self.spin_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # spin_pub.publish(twist)
        

        if self.spin_timer is not None:
            self.spin_timer.cancel()
        self.spin_timer = self.create_timer(0.1, self._spin_publish)
        self.get_logger().info(f"Spin started. Images will be saved as robot rotates.")

        res.success = True
        res.message = f"Spin started. Images will be saved."
        return res
        # # Wait for self.current_yaw to be set by odom_cb
        # while not hasattr(self, "current_yaw"):
        #     rclpy.spin_once(self, timeout_sec=0.1)
        
        # while rclpy.ok() and self.images_captured < N_IMAGES:
        #     spin_pub.publish(twist)
        #     rclpy.spin_once(self, timeout_sec=0.05)

        # twist.angular.z = 0.0
        # spin_pub.publish(twist)
        # self.spin_active = False
        # self.get_logger().info("Spin complete.")

        # message = f'Captured {self.images_captured} image(s) for {req.hazard_type}'
        # res.success = True
        # res.message = message
        # self.get_logger().info(message)
        # return res

    def _spin_publish(self):
        if self.spin_active:
            self.spin_pub.publish(self.twist)
        else:
            self.spin_pub.publish(Twist())  # send zero velocity
            self.spin_timer.cancel() 

    def _save_image(self, img):
        fname = os.path.join(self.out_dir, f"frame_{self.images_captured+1:03d}.png")
        cv2.imwrite(fname, img)
        # self.get_logger().info(f"Saved {fname}")
        self.images_captured += 1
        if self.images_captured >= N_IMAGES:
            self.stop_spin()
            msg = Bool()
            msg.data = True
            self.completion_pub.publish(msg)
        
        # if self.images_captured >= N_IMAGES:
        #     self.stop_spin()

    def stop_spin(self):
        self.spin_active = False
        twist = Twist()
        twist.angular.z = 0.0
        if hasattr(self, "spin_pub"):
            self.spin_pub.publish(twist)
        if self.spin_timer is not None:
            self.spin_timer.cancel()
        message = f'Captured {self.images_captured} image(s) for {self.hazard_type}'
        self.get_logger().info(f"Spin complete: {message}")

    # ────────────────────────────────────────────────────────────────────────
    def _on_request(self, req, res):
        self.get_logger().info("In intial inspection")
        if self._busy:
            res.success = False
            res.message = "Burst already running"
            return res
        
        try:
            
            self._busy = True
            hazard_type = parse_hazard_type(req.hazard_type)

            out_dir = os.path.expanduser(f'{OUT_DIR}/{req.hazard_type}/captured_images_{time.strftime("%Y%m%d_%H%M%S")}')
            os.makedirs(out_dir, exist_ok=True)
            self.get_logger().info(f'Capturing {N_IMAGES} images → {OUT_DIR} for {hazard_type}')

            # start spinning
            twist = Twist()
            twist.angular.z = float(SPIN_SPEED)
            
            dt    = 1.0 / PUB_HZ
            captured = 0

            while rclpy.ok() and captured < N_IMAGES:
                # publish spin command
                self._cmd_pub.publish(twist)

                # allow exactly one incoming message, wait up to dt seconds
                rclpy.spin_once(self, timeout_sec=dt)

                # save current frame
                img_path = os.path.join(out_dir, f'frame_{captured:03d}.{IMG_FMT}')
                cv_img   = self.bridge.imgmsg_to_cv2(self._img_msg, 'bgr8')
                if not cv2.imwrite(img_path, cv_img):
                    self.get_logger().error(f'⚠ could not write {img_path}')
                    break

                captured += 1
                time.sleep(dt) 
                # self.get_logger().info(f'saved {captured}/{N_IMAGES}')
            
            self.get_logger().info(f'saved {captured} images')
            self._cmd_pub.publish(Twist())
            self._busy = False

            res.success = True
            res.message = f'captured {captured} image(s) for {hazard_type}'
            return res
        except Exception as ex:
            self.get_logger().info(f"Some exception occurred - {ex}")

    
# ──────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init(); 
    node = InitialInspectionNode()
    node.get_logger().info("✅ initial_inspection_node main() started")
    exit_gracefully(node)

if __name__ == "__main__":
    main()
