#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import torch.nn as nn
import torchvision.transforms as T
from ament_index_python.packages import get_package_share_directory
from PIL import Image as PILImage
import json

# --- CONFIG ---
PKG_NAME = "material_handling_agent"
PKG_SHARE = get_package_share_directory(PKG_NAME)

VIDEO_PATH = os.path.join(PKG_SHARE, "video", "scrolling_conveyor.mp4")
MODEL_DIR = os.path.join(PKG_SHARE, "ai_models", "resnet18")
MODEL_PATH = os.path.join(MODEL_DIR, "best_loss.pth")
CLASSES_PATH = os.path.join(MODEL_DIR, "classes.json")

PAUSE_SECONDS = 30   # slideshow delay

class ConveyorClassifierSlideshow(Node):
    def __init__(self):
        super().__init__("conveyor_classifier_slideshow_node")
        self.pred_pub = self.create_publisher(String, "/conveyor/classifier_output", 10)
        self.image_pub = self.create_publisher(Image, "/conveyor/slideshow_image", 10)
        self.bridge = CvBridge()

        # Load class names
        with open(CLASSES_PATH, "r") as f:
            self.class_names = json.load(f)
        self.get_logger().info(f"✅ Loaded class names: {self.class_names}")

        # Load model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self._load_model()
        self.model.eval()

        # Transforms
        self.transform = T.Compose([
            T.Resize((224, 224)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406],
                        std=[0.229, 0.224, 0.225]),
        ])

        # Open video
        self.cap = cv2.VideoCapture(VIDEO_PATH)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video {VIDEO_PATH}")
            return

        self.frame_id = 0
        self.get_logger().info(f"🎞️ Slideshow started. Pausing {PAUSE_SECONDS}s per frame...")

        # Timer callback
        self.timer = self.create_timer(PAUSE_SECONDS, self.timer_callback)

    def _load_model(self):
        model = torch.hub.load("pytorch/vision:v0.14.0", "resnet18", pretrained=False)
        model.fc = nn.Linear(model.fc.in_features, len(self.class_names))
        model.load_state_dict(torch.load(MODEL_PATH, map_location=self.device))
        model.to(self.device)
        return model

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("✅ End of video reached")
            rclpy.shutdown()
            return

        self.frame_id += 1

        # Preprocess
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_pil = PILImage.fromarray(frame_rgb)
        inp = self.transform(img_pil).unsqueeze(0).to(self.device)

        # Inference
        with torch.no_grad():
            logits = self.model(inp)
            probs = torch.softmax(logits, dim=1)[0]
            pred_idx = torch.argmax(probs).item()
            pred_class = self.class_names[pred_idx]
            conf = probs[pred_idx].item()

        # Publish text message
        msg = String()
        msg.data = f"frame_{self.frame_id:04d} | prediction={pred_class} | confidence={conf:.2f}"
        self.pred_pub.publish(msg)
        self.get_logger().info(msg.data)

        # Annotate frame
        overlay_text = f"{pred_class} ({conf:.2f})"
        cv2.putText(frame, overlay_text, (30, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)

        # Publish annotated frame
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorClassifierSlideshow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
