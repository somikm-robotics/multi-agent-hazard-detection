# aerial_agent/nodes/detection_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from shared_interfaces.srv import ScanIncident, ClassifyHazard, DetectHazard
from shared_interfaces.msg import HazardPose
from shared_infrastructure.hazard_types import HazardType
from shared_infrastructure.utils import exit_gracefully, load_stopping_distance_from_hazard_from_config
from ament_index_python.packages import get_package_share_directory
from cv_bridge         import CvBridge
from ultralytics import YOLO
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg   import Image
import numpy as np
from rclpy.duration import Duration
from sensor_msgs.msg import CameraInfo

from tf2_ros import Buffer, TransformListener, LookupException, \
                    ConnectivityException, ExtrapolationException
import tf_transformations                                


import random, os


RGB_CAMERA_TOPIC   = "/world/mining_world/model/crazyflie/model/vga_camera/link/vga_camera/sensor/down_cam/image"
CAMERA_INFO_TOPIC = "/world/mining_world/model/crazyflie/model/vga_camera/link/vga_camera/sensor/down_cam/camera_info"
CAMERA_FRAME = "crazyflie/vga_camera/vga_camera/down_cam"
GROUND_Z      = 0.0         # world-frame Z of the mine floor (or use a DEM lookup)
PLANE_WAIT    = 0.5         # seconds to wait for TF transform
IMG_FRAME_ID  = "camera_link"
WORLD_FRAME   = "world"     # or "map"
CENTER_ONLY   = True        # use bbox centre; set False to project four corners
DEPTH_SCALE   = 1.0                          # metres per stored unit
PATCH_R       = 2                            # 5×5 window for median
BBOX_CENTER_TOL = 0.04        # ±4 % of image width / height
CLS_THR = {                 # class-specific confidence thresholds
    "dust_plume":     0.50,
    "fibrous_hazard": 0.30,     # ← lowered only for this class
}
DEFAULT_THR = 0.60              # for any unexpected class-id
BBOX_CENTER_PX = 15            # pixels

def get_point_to_publish(pt):
    # Standard rounding to 1 decimal place
    stoping_distance = load_stopping_distance_from_hazard_from_config()
    x_rounded = round(pt.x, 1) - stoping_distance
    y_rounded = round(pt.y, 1) - stoping_distance

    pt.x = x_rounded
    pt.y = y_rounded
    return pt

def get_bbox_diameter_m(xmin_px, xmax_px, altitude_m, fx_px):
        """
        Estimate hazard diameter assuming the bounding box fully encloses the object
        and the camera is looking straight down.

        Returns: diameter [m]
        """
        px_width = xmax_px - xmin_px          # bounding box width in pixels
        diameter_m = (px_width / fx_px) * altitude_m
        return diameter_m


# Example usage:
# pose = geometry_msgs.msg.Pose()
# pose.position.x = 1.5551926571719494
# pose.position.y = 3.978836384740041
# rounded_pose = round_pose_xy_subtract(pose)  # y will be rounded to 4.0, then -0.5 → 3.5



class HazardDetectionNode(Node):
    def __init__(self):
        super().__init__('hazard_detection_node')
        self.get_logger().info("Hazard Detection Node initialized")
        
        # I/O
        self.bridge   = CvBridge()
        self.latest_cv = None
        self.latest_header  = None      # <-- cache the ROS Header here
        self.create_subscription(Image, RGB_CAMERA_TOPIC, self.image_cb, 10)

        # pub / subs
        self.det_pub = self.create_publisher(HazardPose, 'internal_hazard', 10)
        # self.publish_timer = self.create_timer(1.0, self.publish_internal)
        

        # services
        self.create_service(ClassifyHazard,
                            "classify_hazard",
                            self.handle_classify)
        self.create_service(DetectHazard,
                            "detect_hazard",
                            self.handle_detect)

        # ───── models ──────────────────────────────────────────────
        pkg_share = get_package_share_directory('aerial_agent')
        ai_model_path = os.path.join(pkg_share, "ai_models", "yolo")
        cls_model_path = os.path.join(ai_model_path, "classification", "best.pt")
        det_model_path = os.path.join(ai_model_path, "detection", "yolov8x_best.pt")
        self.cls_model = YOLO(cls_model_path)
        self.det_model = YOLO(det_model_path)

        self.get_logger().info(f"DET classes: {self.det_model.names}")

         # thresholds
        self.cls_thr        = 0.50
        self.det_thr        = 0.75   # one-shot detection threshold

        # TF listener
        self.tf_buf = Buffer(cache_time=Duration(seconds=10))
        self.tf_lst = TransformListener(self.tf_buf, self)

        # # camera intrinsics will arrive on /camera/camera_info
        self.K = None           # 3×3 numpy array
        self.fx = self.fy = self.cx = self.cy = None
        self.create_subscription(
                CameraInfo, CAMERA_INFO_TOPIC, self._caminfo_cb, 10)
        
        self.already_published = False

        self.get_logger().info("✅ hazard_detect_node ready")


    def image_cb(self, msg: Image):
        self.latest_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.latest_header = msg.header


    def _caminfo_cb(self, msg: CameraInfo):
        """Cache intrinsics once – they never change in sim."""
        if self.K is None:
            self.K  = np.array(msg.k).reshape(3, 3)
            self.fx = self.K[0, 0]  
            self.fy = self.K[1, 1]
            self.cx = self.K[0, 2]
            self.cy = self.K[1, 2]
            self.get_logger().info("📷 Camera intrinsics received")

 
    def handle_classify(self, req: ClassifyHazard.Request, res: ClassifyHazard.Response):
        if self.latest_cv is None:
            res.hazard_type = "background"
            return res
        
        pred = self.cls_model(self.latest_cv, verbose=False)[0]

        cls_id, conf = int(pred.probs.top1), float(pred.probs.top1conf)
        
        self.get_logger().info(f"Classifying... id: {cls_id} conf: {conf}")
        
        cls_name = self.cls_model.names[cls_id]
        self.get_logger().info(f"Classifying... class name: {cls_name}")
        # pick the threshold for this specific class
        thr = CLS_THR.get(cls_name, DEFAULT_THR)

        if conf >= thr:
            res.hazard_type = cls_name
        else:
            res.hazard_type = "background"
            
        return res
        
    # one-shot detector  ───────────────────────────────────────────
    def handle_detect(self, req:DetectHazard.Request, res: DetectHazard.Response):
        
        res.detected = False
        res.centred  = False
        res.hazard_type = ""
        res.has_published = False

        if self.latest_cv is None:
            self.get_logger().info("No image, so returning")
            return res

        pred = self.det_model(
            self.latest_cv,
            verbose=False,
            conf=0.05,        # << lower than default 0.25 so NMS keeps weak boxes
            iou=0.5,
            imgsz=640         # << match your training imgsz
        )[0]

        self.get_logger().info(f"DET raw box count: {len(pred.boxes)}")
        if len(pred.boxes) == 0:
            self.get_logger().info("No Boxes, so returning")
            return res

        result = self.det_model(self.latest_cv, verbose=False)[0]
        if not result.boxes:
            self.get_logger().info("No Boxes, so returning")
            return res

        best = result.boxes[0]
        conf = float(best.conf)
        cls_id  = int(best.cls)

        class_name = self.det_model.names[cls_id]
        
        self.get_logger().info(f"Detected: Class Name: {cls_id} - {class_name} - {conf}")

        if conf < self.det_thr or cls_id not in (0, 1):      # only two classes
            return res                       # nothing confident enough

        # cls_id  = int(best.cls)
        
        try:
        
            x_d, y_d, z_agl = req.x, req.y, req.z    # drone XY in map + altitude above ground
            # camera intrinsics (cached once from /camera_info)
            fx, fy = self.fx, self.fy                 # focal lengths in pixels
            xmin, ymin, xmax, ymax = best.xyxy[0].tolist()       # tensor → list
            #  project the box centre to world and publish the pose
            
            # 1. pixel offsets (signed) 
            u_c = 0.5 * (xmin + xmax)
            v_c = 0.5 * (ymin + ymax)

            W, H = self.latest_cv.shape[1], self.latest_cv.shape[0]
            cx, cy         = W*0.5, H*0.5
            du_px  = u_c - cx          # +ve ⇒ object right of centre
            dv_px  = v_c - cy          # +ve ⇒ object below centre

            # 2. offsets in the *camera* frame (metres) 
            dx_cam =  (du_px / fx) * z_agl        # +X_cam points right in the image
            dy_cam =  (dv_px / fy) * z_agl        # +Y_cam points down

            # --- 3. rotate cam-offset into the world frame --------------------
            tf = self.tf_buf.lookup_transform(         # camera → world rotation
                    target_frame='map',
                    source_frame=CAMERA_FRAME,
                    time=rclpy.time.Time())
            q   = tf.transform.rotation                # geometry_msgs/Quaternion
            R   = tf_transformations.quaternion_matrix(
                    [q.x, q.y, q.z, q.w])[:3, :3]

            offset_world = R @ np.array([dx_cam, dy_cam, 0.0])   # Z=0 on ground
            haz_x = x_d + offset_world[0]
            haz_y = y_d + offset_world[1]

            self.get_logger().info(f"Hazard center point determined:  x, y: ({haz_x}, {haz_y}")
            center_pt = Point()
            center_pt.x = round(haz_x, 1)
            center_pt.y = round(haz_y, 1)
            center_pt.z = 0.0                    # ground-plane
        
            res.hazard_type = class_name
            self.get_logger().info(f"Requesting bbox diameter: ({xmin}, {xmax}) z: {req.z}  {self.fx}")

            diameter_m = get_bbox_diameter_m(xmin, xmax, req.z, self.fx)
            estimated_diameter = round(diameter_m, 1)
            self.get_logger().info(f"Class name: {class_name}")
            self.get_logger().info(f"x: {req.x}, y: {req.y}")
            pt = Point();  
            pt.x, pt.y, pt.z = req.x, req.y, req.z
            
            self.publish_internal(class_name, conf, pt, estimated_diameter, center_point=center_pt)
            self.already_published = True
            res.detected     = True
            res.hazard_type  = class_name
            res.has_published = True

            return res
        except Exception as ex:
            self.get_logger().info(f"Exception in detect hazard: {ex}")
        
    

        
    def publish_internal(self, name, conf, pt, estimated_hazard_diameter, center_point):
        
        try:
        
            # publish pose (placeholder pose – replace with real geo-loc)
            msg = HazardPose()
            msg.hazard_type = name
            
            publish_pt = get_point_to_publish(pt)
            ps = Pose()
            ps.position = publish_pt                  

            ps.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

            msg.pose = ps
            msg.hazard_center_point = center_point
            msg.estimated_hazard_diameter = estimated_hazard_diameter
            
            self.get_logger().info(
                f"📣 detect_hazard → {name} (conf={conf:.2f}) published")
            self.get_logger().info(
                f"📣 Hazard Center point → ({center_point.x}, {center_point.y}, Estimated: {estimated_hazard_diameter} published")
            self.det_pub.publish(msg)
            self.get_logger().info(f"🚨 Published internal hazard: {msg}")
        except Exception as ex:
            self.get_logger().info(f"Exception: {ex}")    
    
    



def main(args=None):
    rclpy.init(args=args)
    node = HazardDetectionNode()
    node.get_logger().info("✅ hazad_detection_node main() started") 
    exit_gracefully(node)

if __name__ == "__main__":
    main()
