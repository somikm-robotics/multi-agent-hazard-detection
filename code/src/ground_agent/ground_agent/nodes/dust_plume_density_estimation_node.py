#!/usr/bin/env python3
"""
DustPlumeDensityNode
====================

• Subscribes to a monocular RGB camera topic (param: **camera_topic**).
• Caches the *latest* frame in BGR OpenCV format.
• Exposes the service **estimate_plume_density** (dust_msgs/srv/PlumeDensity):

    request : (empty - only a header stamp is useful for QoS if desired)  
    response: float32 density

If no frame has arrived yet, the service returns `NaN`.

––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

"""

import os, time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import cv2
import torch
from torch import nn
from torchvision import transforms as T

from cv_bridge          import CvBridge
from sensor_msgs.msg    import Image

from shared_interfaces.srv import PlumeDensity
from shared_infrastructure.utils import exit_gracefully
from shared_interfaces.msg import DensityEstimationResult

import torch
import torch.nn as nn
from torchvision import transforms, models


CAMERA_TOPIC = "/world/mining_world/model/agilex_diff_drive/link/base_link/sensor/rgb_cam/image"

class DustPlumeDensityEstimationNode(Node):
    def __init__(self):
        super().__init__("dust_plume_density_estimation_node")
        
        #Model 
        pkg_share = get_package_share_directory('ground_agent')
        ai_model_path = os.path.join(pkg_share, "ai_models", "density_regressor.pt")
        
        if not os.path.exists(ai_model_path):
            self.get_logger().fatal(f"Density model not found: {ai_model_path}")
            raise SystemExit(1)

        # device
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        #  model
        backbone = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)  # same as training
        backbone.fc = nn.Sequential(                                        # same head
            nn.Linear(backbone.fc.in_features, 1)
        )

        state = torch.load(ai_model_path, map_location=device)              # state_dict
        backbone.load_state_dict(state, strict=True)                        # keys match
        backbone.to(device).eval()

        self.model: nn.Module = backbone
        self.device            = device

        self.get_logger().info(f"✅ Density regressor loaded: {ai_model_path}")

        self.completion_pub = self.create_publisher(DensityEstimationResult, "estimation_complete", 10)
        

        #  Pre-processing 
        self.bridge  = CvBridge()
        self.pre_tf  = T.Compose([
            T.ToPILImage(),
            T.Resize((224, 224)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406],
                        std=[0.229, 0.224, 0.225]),
        ])

        self.latest_cv = None          # BGR image cache
        self.last_stamp_ns = 0

        # Subscribers / Service 
        self.create_subscription(Image, CAMERA_TOPIC,
                                 self._img_cb, 10)

        self.srv = self.create_service(
            PlumeDensity, "estimate_plume_density", self.handle_req)

        self.get_logger().info(f"📷 Subscribed to {CAMERA_TOPIC}")
        self.get_logger().info("🧪 Dust Plume density estimate node ready")

    
    #  Callbacks
    
    def _img_cb(self, msg: Image):
        """Store the most recent image (BGR CV-Mat)."""
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_cv = cv_img
            # self.last_stamp_ns = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
        except Exception as ex:
            self.get_logger().error(f"cv_bridge conversion failed: {ex}")

    # ------------------------------------------------------------------
    def handle_req(self,
                    req: PlumeDensity.Request,
                    res: PlumeDensity.Response) -> PlumeDensity.Response:
        """
        Return the predicted dust-plume density for the *latest* cached frame.
        """

        if self.latest_cv is None:
            self.get_logger().warn("No camera frame received yet – returning NaN")
            res.success = False
            return res

        try:
        # BGR ➜ RGB
            rgb_img = cv2.cvtColor(self.latest_cv, cv2.COLOR_BGR2RGB)
            tensor = self.pre_tf(rgb_img).unsqueeze(0)          # 1 × 3 × 224 × 224
            tensor   = tensor.to(self.device)      

            with torch.no_grad():
                pred = self.model(tensor).squeeze().item()
                
            res.success = True

            msg = DensityEstimationResult()
            msg.density = float(pred)
            msg.level = self.compute_severity_level(msg.density)
            self.completion_pub.publish(msg)
            self.get_logger().info(f"Estimated Densty: {msg.density} - Severity Level: {msg.level}")
            return res
        except Exception as ex:
            self.get_logger().info(f"Some exception occurrd: {ex}")


    def compute_severity_level(self, density):  
        if density < 0.2:
            level = "🟢 Low"
        elif density < 0.4:
            level = "🟡 Moderate"
        elif density < 0.6:
            level = "🟠 High"
        elif density < 0.8:
            level = "🔴 Very High"
        else:
            level = "☠️ Severe"
        return level

# ======================================================================
def main(args=None):
    rclpy.init(args=args)
    node = DustPlumeDensityEstimationNode()
    node.get_logger().info("✅ dust plume density estimation node main() started")
    exit_gracefully(node)


if __name__ == "__main__":
    main()
