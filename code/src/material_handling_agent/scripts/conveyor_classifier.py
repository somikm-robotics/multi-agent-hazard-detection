#!/usr/bin/env python3
import os
import cv2
import yaml
import torch
import torch.nn as nn
import torchvision.transforms as T
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# --- CONFIG ---
PKG_NAME = "material_handling_agent"
PKG_SHARE = get_package_share_directory(PKG_NAME)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_SRC = os.path.dirname(SCRIPT_DIR)  # goes up one level to src/material_handling_agent

IMG_DIR   = os.path.join(PKG_SHARE, "video", "images")
OUT_DIR   = os.path.join("/tmp", "annotated_images")
YAML_PATH = os.path.join(PKG_SHARE, "config", "combined_ore_sequence.yaml")
MODEL_PATH = os.path.join(PKG_SHARE, "ai_models", "resnet18", "best_loss.pth")

CLASS_NAMES = ["blocked", "misalignment", "normal", "overloaded", "spillage"]

# --- Load model ---
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = torch.hub.load("pytorch/vision:v0.14.0", "resnet18", pretrained=False)
model.fc = nn.Linear(model.fc.in_features, len(CLASS_NAMES))
model.load_state_dict(torch.load(MODEL_PATH, map_location=device))
model.to(device)
model.eval()

transform = T.Compose([
    T.ToPILImage(),
    T.Resize((224, 224)),
    T.ToTensor(),
    T.Normalize(mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]),
])

# --- Load YAML order ---
with open(YAML_PATH, "r") as f:
    yaml_data = yaml.safe_load(f)

sequence = []
for entry in yaml_data:
    for hazard, ores in entry.items():
        for ore in ores:
            sequence.append(ore)

os.makedirs(OUT_DIR, exist_ok=True)

# --- Process each ore once ---
for ore_name in sequence:
    img_path = os.path.join(IMG_DIR, f"{ore_name}.png")
    if not os.path.exists(img_path):
        print(f"⚠️ Missing {img_path}, skipping")
        continue

    img = cv2.imread(img_path)
    inp = transform(img).unsqueeze(0).to(device)

    with torch.no_grad():
        logits = model(inp)
        probs = torch.softmax(logits, dim=1)[0]
        pred_idx = torch.argmax(probs).item()
        pred_class = CLASS_NAMES[pred_idx]
        conf = probs[pred_idx].item()

    overlay_text = f"{pred_class} ({conf:.2f})"
    cv2.putText(img, overlay_text, (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)

    out_path = os.path.join(OUT_DIR, f"{ore_name}.png")
    cv2.imwrite(out_path, img)
    print(f"✅ Annotated {ore_name} → {out_path} - {pred_class} ({conf:.2f})")
