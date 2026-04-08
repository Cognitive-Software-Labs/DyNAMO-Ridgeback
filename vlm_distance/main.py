"""
Unitree G1 Robot Detection - Local VLM Version (No API Key Required)

What this script does:
1. Loads a local VLM (Llava-Onevision) using the transformers library.
2. Uses camera settings from config.json.
3. Estimates top-down robot position (x, z) using camera formulas.
4. Hardcodes the image path so no terminal input is needed.
5. Shows the result on screen with bounding boxes and coordinates.

Requirements:
    pip install transformers torch torchvision torchaudio Pillow matplotlib numpy accelerate
"""

import numpy as np
import json
import os
import re
import sys
import math
import torch
from PIL import Image
from transformers import pipeline
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# --- CONFIGURATION ---
# Hardcoded image path
DEFAULT_IMAGE = "g1_photo.jpg"
# Local VLM Model (Small and efficient)
VLM_MODEL = "llava-hf/llava-onevision-qwen2-0.5b-ov-hf"
# Configuration file
CONFIG_FILE = "config.json"

def load_config(config_path: str = CONFIG_FILE) -> dict:
    """Load camera constants from a JSON file."""
    if not os.path.exists(config_path):
        print(f"Warning: {config_path} not found. Using default camera specs.")
        return {
            "camera": {
                "depth_hfov_deg": 87.0,
                "depth_vfov_deg": 58.0,
                "pitch_deg": 0.0,
                "height_m": 0.50476
            }
        }
    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)

def add_topdown_xz_from_floor(result: dict, config: dict) -> dict:
    """
    Estimate top-down (x, z) using camera geometry.
    """
    if not result.get("detected"):
        return result

    camera_cfg = config["camera"]
    depth_hfov_deg = camera_cfg["depth_hfov_deg"]
    depth_vfov_deg = camera_cfg["depth_vfov_deg"]
    camera_pitch_deg = camera_cfg["pitch_deg"]
    camera_height_m = camera_cfg["height_m"]

    img_w = result["img_width"]
    img_h = result["img_height"]

    # Focal length in pixels
    fx = img_w / (2.0 * math.tan(math.radians(depth_hfov_deg / 2.0)))
    fy = img_h / (2.0 * math.tan(math.radians(depth_vfov_deg / 2.0)))

    # Principal point (center of image)
    cx0 = img_w / 2.0
    cy0 = img_h / 2.0

    pitch = math.radians(camera_pitch_deg)

    for robot in result["robots"]:
        x1, y1, x2, y2 = robot["bbox_pixel"]
        cx, cy = robot["center_px"]

        # Assume bottom-center is the contact point with the floor
        u = cx
        v = y2

        # Ray-casting from camera
        x_cam = (u - cx0) / fx
        y_cam = (v - cy0) / fy
        z_cam = 1.0

        # Apply pitch rotation
        y_world = math.cos(pitch) * y_cam - math.sin(pitch) * z_cam
        z_world = math.sin(pitch) * y_cam + math.cos(pitch) * z_cam
        x_world = x_cam

        if y_world <= 0:
            robot["x_m"] = None
            robot["z_m"] = None
            continue

        # Intersection with floor plane (y = height)
        t = camera_height_m / y_world
        x_m = x_world * t
        z_m = z_world * t

        robot["x_m"] = float(x_m)
        robot["z_m"] = float(z_m)
        robot["distance_m"] = float(math.sqrt(x_m ** 2 + z_m ** 2))

    return result

def parse_detection_result(text: str, img_width: int, img_height: int) -> dict:
    """
    Extract bounding boxes from the model's text output.
    Llava often outputs boxes as [ymin, xmin, ymax, xmax] or [x1, y1, x2, y2].
    We'll try to catch common patterns.
    """
    result = {
        "detected": False,
        "count": 0,
        "robots": [],
        "img_width": img_width,
        "img_height": img_height,
    }

    # Look for patterns like [y1, x1, y2, x2] or [x1, y1, x2, y2]
    # Normalizing to 0-1000 range if they look like percentages/normalized
    box_pattern = r"\[(\d+),\s*(\d+),\s*(\d+),\s*(\d+)\]"
    matches = re.findall(box_pattern, text)

    for match in matches:
        # Llava-Onevision often uses normalized coordinates (0-1000)
        # Note: Llava format is often [ymin, xmin, ymax, xmax]
        v1, v2, v3, v4 = [int(v) for v in match]
        
        # Heuristic to guess order. We'll assume [y1, x1, y2, x2] for Llava
        # but let's try to be robust. If v1 > v3, it's definitely not [y1, x1, y2, x2]
        # Most VLMs use [y1, x1, y2, x2] normalized to 1000.
        
        y1_n, x1_n, y2_n, x2_n = v1, v2, v3, v4
        
        # Sanity check: if it's [x1, y1, x2, y2] instead
        if x1_n > x2_n or y1_n > y2_n:
            x1_n, y1_n, x2_n, y2_n = v1, v2, v3, v4

        # Convert normalized to pixels
        x1_px = int(x1_n / 1000 * img_width)
        y1_px = int(y1_n / 1000 * img_height)
        x2_px = int(x2_n / 1000 * img_width)
        y2_px = int(y2_n / 1000 * img_height)

        result["robots"].append({
            "bbox_pixel": [x1_px, y1_px, x2_px, y2_px],
            "center_px": [(x1_px + x2_px) // 2, (y1_px + y2_px) // 2],
            "width_px": x2_px - x1_px,
            "height_px": y2_px - y1_px,
        })

    result["count"] = len(result["robots"])
    result["detected"] = result["count"] > 0
    return result

def display_results(image_path: str, result: dict):
    """Visualize detections and top-down coordinates."""
    img = Image.open(image_path).convert("RGB")
    fig, ax = plt.subplots(1, figsize=(12, 8))
    ax.imshow(img)

    if not result["detected"]:
        ax.set_title("No Robots Detected", color="red")
    else:
        for i, robot in enumerate(result["robots"]):
            x1, y1, x2, y2 = robot["bbox_pixel"]
            rect = patches.Rectangle((x1, y1), x2-x1, y2-y1, linewidth=2, edgecolor='lime', facecolor='none')
            ax.add_patch(rect)
            
            label = f"Robot #{i+1}"
            if "x_m" in robot and robot["x_m"] is not None:
                label += f"\nX:{robot['x_m']:.2f}m Z:{robot['z_m']:.2f}m"
            
            ax.text(x1, y1-10, label, color='white', weight='bold', bbox=dict(facecolor='black', alpha=0.5))

    ax.axis("off")
    plt.show()

def main():
    print(f"--- Loading Local Model: {VLM_MODEL} ---")
    print("This may take a minute on the first run...")
    
    # Initialize the pipeline
    device = "cuda" if torch.cuda.is_available() else "cpu"
    pipe = pipeline(
        task="image-text-to-text",
        model=VLM_MODEL,
        device_map="auto",
        torch_dtype=torch.float16 if torch.cuda.is_available() else torch.float32,
    )

    # Load image
    if not os.path.exists(DEFAULT_IMAGE):
        print(f"Error: {DEFAULT_IMAGE} not found in directory.")
        return
    
    image = Image.open(DEFAULT_IMAGE).convert("RGB")
    width, height = image.size

    # Prepare prompt
    # Llava-Onevision likes specific formatting for detection
    prompt = "Locate the Unitree G1 humanoid robot in the image. Return the bounding box in the format [ymin, xmin, ymax, xmax] using normalized coordinates (0-1000)."
    
    messages = [
        {
            "role": "user",
            "content": [
                {"type": "image", "image": image},
                {"type": "text", "text": prompt},
            ],
        }
    ]

    print(f"--- Analyzing Image: {DEFAULT_IMAGE} ---")
    outputs = pipe(text=messages, max_new_tokens=100)
    
    # Extract the generated text
    # The structure depends on the pipeline version, usually:
    # result[0]["generated_text"][-1]["content"]
    try:
        response_text = outputs[0]["generated_text"][-1]["content"]
    except (KeyError, IndexError):
        response_text = str(outputs)

    print("Model Response:", response_text)

    # Parse and apply formulas
    detection_data = parse_detection_result(response_text, width, height)
    config = load_config()
    final_result = add_topdown_xz_from_floor(detection_data, config)

    # Output Summary
    if final_result["detected"]:
        for i, r in enumerate(final_result["robots"]):
            print(f"Robot {i+1}:")
            if r.get("x_m") is not None:
                print(f"  Position: X={r['x_m']:.2f}m, Z={r['z_m']:.2f}m")
                print(f"  Distance: {r['distance_m']:.2f}m")
    else:
        print("No robots detected.")

    # Visualize
    display_results(DEFAULT_IMAGE, final_result)

if __name__ == "__main__":
    main()
