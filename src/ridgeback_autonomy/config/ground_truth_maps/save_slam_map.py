#!/usr/bin/env python3
"""
Helper script to save SLAM maps and prepare them for ground-truth conversion.

This script provides utilities for:
1. Saving the current SLAM map as ground-truth
2. Verifying map dimensions match expected world size
3. Basic cleanup operations

Usage:
    python save_slam_map.py --output ground_truth_hospital --verify-dims 30 20
"""

import subprocess
import os
import sys
import argparse
import numpy as np
from PIL import Image
import yaml


def save_map(output_name, output_dir='./'):
    """Save the current SLAM map using map_saver_cli."""
    output_path = os.path.join(output_dir, output_name)
    cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', output_path]
    
    print(f"[INFO] Saving map as {output_name}...")
    result = subprocess.run(cmd, capture_output=True, text=True)
    
    if result.returncode != 0:
        print(f"[ERROR] Failed to save map")
        print(f"STDOUT: {result.stdout}")
        print(f"STDERR: {result.stderr}")
        return False
    
    print(f"[INFO] Map saved successfully")
    return True


def verify_map_dimensions(yaml_path, expected_width_m, expected_height_m, tolerance_pct=10):
    """Verify that the saved map dimensions match expected world size."""
    
    try:
        with open(yaml_path, 'r') as f:
            map_yaml = yaml.safe_load(f)
    except Exception as e:
        print(f"[ERROR] Failed to read YAML: {e}")
        return False
    
    resolution = map_yaml.get('resolution', 0.05)
    image_path = yaml_path.replace('.yaml', '.pgm')
    
    try:
        img = Image.open(image_path)
        width_px, height_px = img.size
    except Exception as e:
        print(f"[ERROR] Failed to read image: {e}")
        return False
    
    # Calculate actual dimensions in meters
    actual_width_m = width_px * resolution
    actual_height_m = height_px * resolution
    
    # Check if dimensions are within tolerance
    width_error_pct = abs(actual_width_m - expected_width_m) / expected_width_m * 100
    height_error_pct = abs(actual_height_m - expected_height_m) / expected_height_m * 100
    
    print(f"\n[VERIFY] Map dimensions:")
    print(f"  Expected: {expected_width_m:.1f}m x {expected_height_m:.1f}m")
    print(f"  Actual:   {actual_width_m:.1f}m x {actual_height_m:.1f}m")
    print(f"  Image:    {width_px}px x {height_px}px @ {resolution}m/cell")
    print(f"  Error:    {width_error_pct:.1f}% x {height_error_pct:.1f}%")
    
    if width_error_pct > tolerance_pct or height_error_pct > tolerance_pct:
        print(f"[WARN] Dimensions exceed {tolerance_pct}% tolerance")
        return False
    
    print(f"[OK] Dimensions within tolerance")
    return True


def cleanup_map_basic(pgm_path, yaml_path):
    """
    Perform basic cleanup on a SLAM-derived map:
    - Fill unknown cells (gray) inside closed regions with white (free space)
    - This is a placeholder for manual GIMP cleanup
    """
    print(f"\n[INFO] Basic cleanup would process:")
    print(f"  - Read PGM image from {pgm_path}")
    print(f"  - Identify isolated gray regions (unknown)")
    print(f"  - Replace interior unknowns with white (free)")
    print(f"  - Save cleaned version")
    print(f"\n[MANUAL STEP] For proper cleanup, open in GIMP:")
    print(f"  1. Open {pgm_path} in GIMP")
    print(f"  2. Remove stray gray cells in reachable rooms (fill with white)")
    print(f"  3. Fix phantom walls in doorways")
    print(f"  4. Leave unreachable areas (outside building) gray")
    print(f"  5. Export as PNG with same dimensions and save as {pgm_path}")
    

def main():
    parser = argparse.ArgumentParser(
        description='Helper script for SLAM ground-truth map generation'
    )
    parser.add_argument('--save', help='Save current SLAM map with this name prefix')
    parser.add_argument('--output-dir', default='./', help='Output directory for maps')
    parser.add_argument('--verify-dims', nargs=2, type=float, metavar=('WIDTH', 'HEIGHT'),
                        help='Verify map dimensions (in meters): --verify-dims 30 20')
    parser.add_argument('--verify-map', help='Path to YAML file to verify dimensions')
    parser.add_argument('--cleanup', help='Perform basic cleanup on map (PGM path)')
    
    args = parser.parse_args()
    
    if args.save:
        if not save_map(args.save, args.output_dir):
            sys.exit(1)
        
        yaml_path = os.path.join(args.output_dir, f'{args.save}.yaml')
        print(f"\nSaved map files:")
        print(f"  {os.path.join(args.output_dir, args.save)}.pgm")
        print(f"  {yaml_path}")
    
    if args.verify_map and args.verify_dims:
        if not verify_map_dimensions(args.verify_map, args.verify_dims[0], args.verify_dims[1]):
            sys.exit(1)
    
    if args.cleanup:
        yaml_path = args.cleanup.replace('.pgm', '.yaml')
        cleanup_map_basic(args.cleanup, yaml_path)


if __name__ == '__main__':
    main()
