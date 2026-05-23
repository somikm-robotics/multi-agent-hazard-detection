#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import yaml
from pathlib import Path

# --- CONFIG ---
PKG_NAME = "material_handling_agent"
ROOT = Path(os.path.expanduser("~/ros2_ws/src/material_handling_agent"))

YAML_PATH = ROOT / "config" / "combined_ore_sequence.yaml"
ANNOTATED_DIR = Path("/tmp/annotated_images")   # annotated images location
PAUSE_SECONDS = 10   # slideshow delay between frames


class ConveyorClassifierSlowScrollNode(Node):
    def __init__(self):
        super().__init__("conveyor_classifier_slow_scroll")
        self.pred_pub = self.create_publisher(String, "/conveyor/classifier_output", 10)
        self.image_pub = self.create_publisher(Image, "/conveyor/slideshow_image", 10)
        self.bridge = CvBridge()

        # Load YAML sequence
        with open(YAML_PATH, "r") as f:
            yaml_data = yaml.safe_load(f)

        self.sequence = []
        for entry in yaml_data:
            for hazard, ores in entry.items():
                for ore in ores:
                    img_path = ANNOTATED_DIR / f"{ore}.png"
                    if img_path.exists():
                        self.sequence.append((ore, hazard, str(img_path)))
                    else:
                        self.get_logger().warn(f"⚠️ Missing image for {ore}")

        self.index = 0
        self.get_logger().info(f"✅ Loaded YAML slideshow with {len(self.sequence)} images")

        # Show first frame immediately
        self.show_next()

        # Timer for slow scroll (subsequent frames)
        self.timer = self.create_timer(PAUSE_SECONDS, self.show_next)

    def show_next(self):
        if self.index >= len(self.sequence):
            self.get_logger().info("✅ End of YAML slideshow reached")
            cv2.destroyAllWindows()
            self.destroy_node()   # end node cleanly
            return

        ore_name, hazard, img_path = self.sequence[self.index]
        frame = cv2.imread(img_path)
        if frame is None:
            self.get_logger().error(f"Failed to load {img_path}")
            self.index += 1
            return

        # Publish message
        msg = String()
        msg.data = f"{ore_name} | ground_truth={hazard}"
        self.pred_pub.publish(msg)
        self.get_logger().info(msg.data)

        # Show in local OpenCV window
        cv2.imshow("Conveyor Slow Scroll", frame)
        cv2.waitKey(1)

        # Publish to ROS2 topic
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(img_msg)

        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorClassifierSlowScrollNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
