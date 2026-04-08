"""
inspect_depth.py — Interactive Relative Depth Inspector for Depth-Anything-V2

Purpose:
    Load a Depth-Anything-V2 model (relative depth, ViT backbone) and run inference
    on a single image. Two OpenCV windows are displayed side-by-side:
      - "RGB Image"  : the original input image
      - "Depth Map"  : a false-colour (INFERNO colormap) visualisation of predicted depth

    Click anywhere in either window to query the relative depth value at that pixel.
    The clicked point is highlighted with a green circle and the coordinates + depth
    value are printed to the terminal and overlaid on both windows.
    Press 'q' to quit.

Where to run:
    Run this script from the ROOT of the Depth-Anything-V2 repository, i.e. the
    directory that contains depth_anything_v2/ and checkpoints/.

    Example:
        cd /path/to/Depth-Anything-V2
        python inspect_depth.py

    Make sure the checkpoint file exists at:
        checkpoints/depth_anything_v2_<encoder>.pth
    and that the target image (e.g. 1.jpeg) is present in the same directory.
"""

import cv2
import torch
import numpy as np

from depth_anything_v2.dpt import DepthAnythingV2

# ── Device selection ────────────────────────────────────────────────────────────
DEVICE = (
    'cuda' if torch.cuda.is_available()
    else 'mps' if torch.backends.mps.is_available()
    else 'cpu'
)

# ── Model configuration table ───────────────────────────────────────────────────
model_configs = {
    'vits': {'encoder': 'vits', 'features': 64,  'out_channels': [48,   96,   192,  384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96,   192,  384,  768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256,  512,  1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]},
}

# ── User settings ───────────────────────────────────────────────────────────────
encoder  = 'vits'     # Choose encoder: 'vits' | 'vitb' | 'vitl' | 'vitg'
img_path = '1.jpeg'   # Path to the input image (relative to the repo root)

# ── Step 1: Load model ──────────────────────────────────────────────────────────
model = DepthAnythingV2(**model_configs[encoder])
model.load_state_dict(
    torch.load(f'checkpoints/depth_anything_v2_{encoder}.pth', map_location='cpu')
)
model = model.to(DEVICE).eval()

# ── Step 2: Read input image ────────────────────────────────────────────────────
raw_img = cv2.imread(img_path)
if raw_img is None:
    raise FileNotFoundError(f'Cannot read image: {img_path}')

# ── Step 3: Run depth inference ─────────────────────────────────────────────────
depth = model.infer_image(raw_img)   # Returns a (H, W) float array of relative depth
depth = depth.astype(np.float32)

# ── Step 4: Build colour-mapped depth visualisation ─────────────────────────────
depth_min = depth.min()
depth_max = depth.max()

depth_norm = (depth - depth_min) / (depth_max - depth_min + 1e-8)   # Normalise to [0, 1]
depth_vis  = (depth_norm * 255).astype(np.uint8)
depth_vis  = cv2.applyColorMap(depth_vis, cv2.COLORMAP_INFERNO)

# Working copies of the display frames (redrawn on each click)
img_show   = raw_img.copy()
depth_show = depth_vis.copy()

# Storage for the most recently clicked pixel
last_point = {"x": None, "y": None}


def redraw():
    """Redraw both display frames, overlaying a marker at the last clicked point."""
    global img_show, depth_show
    img_show   = raw_img.copy()
    depth_show = depth_vis.copy()

    if last_point["x"] is not None and last_point["y"] is not None:
        x, y  = last_point["x"], last_point["y"]
        value = depth[y, x]

        # Draw a filled green circle at the clicked location
        cv2.circle(img_show,   (x, y), 6, (0, 255, 0), -1)
        cv2.circle(depth_show, (x, y), 6, (0, 255, 0), -1)

        # Overlay coordinate and depth value as text
        text = f'({x}, {y}) depth={value:.4f}'
        cv2.putText(img_show,   text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(depth_show, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)


def on_mouse(event, x, y, flags, param):
    """Mouse callback: on left-click, record the pixel and refresh the display."""
    if event == cv2.EVENT_LBUTTONDOWN:
        last_point["x"] = x
        last_point["y"] = y

        value = depth[y, x]
        print(f'Clicked: x={x}, y={y},  relative depth={value:.6f}')

        redraw()
        cv2.imshow('RGB Image',  img_show)
        cv2.imshow('Depth Map',  depth_show)


# ── Step 5: Open display windows and register mouse callbacks ───────────────────
redraw()

cv2.namedWindow('RGB Image',  cv2.WINDOW_NORMAL)
cv2.namedWindow('Depth Map',  cv2.WINDOW_NORMAL)

cv2.setMouseCallback('RGB Image',  on_mouse)
cv2.setMouseCallback('Depth Map',  on_mouse)

cv2.imshow('RGB Image',  img_show)
cv2.imshow('Depth Map',  depth_show)

print('Instructions:')
print('  - Click anywhere on the RGB image or the Depth Map')
print('  - The terminal will print the relative depth value at that pixel')
print('  - Press "q" to quit')

# ── Main event loop ─────────────────────────────────────────────────────────────
while True:
    key = cv2.waitKey(20) & 0xFF
    if key == ord('q'):
        break

cv2.destroyAllWindows()
