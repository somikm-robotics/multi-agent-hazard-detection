import cv2
import os
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# --- CONFIG ---
PKG_NAME = "material_handling_agent"
PKG_SHARE = get_package_share_directory(PKG_NAME)

# Resolve package *source* tree (not install/share)
SRC_ROOT = os.path.dirname(os.path.dirname(PKG_SHARE))

ANNOTATED_DIR = os.path.join(SRC_ROOT, "video", "annotated_images")
YAML_PATH     = os.path.join(PKG_SHARE, "config", "combined_ore_sequence.yaml")
OUT_PATH      = os.path.join("/tmp", "annotated_summary.mp4")

FPS = 1  # 1 frame per second (slower!)
SIZE = (1280, 720)

# --- Load YAML order ---
with open(YAML_PATH, "r") as f:
    yaml_data = yaml.safe_load(f)

sequence = []
for entry in yaml_data:
    for hazard, ores in entry.items():
        for ore in ores:
            sequence.append(ore)

# --- Make video ---
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
writer = cv2.VideoWriter(OUT_PATH, fourcc, FPS, SIZE)

for ore_name in sequence:
    img_path = os.path.join(ANNOTATED_DIR, f"{ore_name}.png")
    if not os.path.exists(img_path):
        print(f"⚠️ Missing {img_path}, skipping")
        continue

    img = cv2.imread(img_path)
    img = cv2.resize(img, SIZE)
    writer.write(img)
    print(f"Added {ore_name} to video")

writer.release()
print(f"\n🎬 Annotated summary video saved to {OUT_PATH}")
