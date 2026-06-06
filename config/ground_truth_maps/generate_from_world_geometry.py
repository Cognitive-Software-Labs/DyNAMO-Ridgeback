#!/usr/bin/env python3
"""
Generate synthetic ground-truth maps by parsing world geometry.

This script analyzes SDF world files to:
1. Extract obstacle and wall positions
2. Generate occupancy grids based on geometry
3. Create PGM + YAML map files

This is useful as a reference for validation or for simple worlds with
clear geometry (e.g., warehouse with regular shelving).

Usage:
    python generate_from_world_geometry.py --world hospital.sdf --output ground_truth_hospital
    python generate_from_world_geometry.py --world warehouse.sdf --output ground_truth_warehouse
"""

import os
import argparse
import numpy as np
from PIL import Image, ImageDraw
import yaml
from pathlib import Path


class WorldGeometryAnalyzer:
    """Parse SDF files and extract collision geometry for map generation."""
    
    def __init__(self, sdf_path):
        """Load and parse SDF file."""
        self.sdf_path = sdf_path
        self.obstacles = []
        self._parse_sdf()
    
    def _parse_sdf(self):
        """Simple SDF parser to extract wall/obstacle boxes."""
        try:
            import xml.etree.ElementTree as ET
            tree = ET.parse(self.sdf_path)
            root = tree.getroot()
            
            # Find all models
            for model in root.findall('.//model'):
                model_name = model.get('name', 'unknown')
                pose_str = model.findtext('pose', '0 0 0 0 0 0')
                pose_vals = [float(x) for x in pose_str.split()[:6]]
                pose = {'x': pose_vals[0], 'y': pose_vals[1], 'z': pose_vals[2]}
                
                # Find all boxes in this model
                for box in model.findall('.//box'):
                    size_str = box.findtext('size', '0 0 0')
                    size_vals = [float(x) for x in size_str.split()]
                    
                    # Only include boxes that look like walls/obstacles (not floors)
                    if size_vals[2] > 0.1:  # Height > 10cm suggests a wall/obstacle
                        self.obstacles.append({
                            'name': model_name,
                            'position': pose,
                            'size': {'x': size_vals[0], 'y': size_vals[1], 'z': size_vals[2]},
                        })
        except Exception as e:
            print(f"[WARN] Failed to parse SDF: {e}")
            print(f"       Manual map creation may be necessary")
    
    def get_obstacles(self):
        """Return list of detected obstacles."""
        return self.obstacles


def generate_occupancy_grid(obstacles, width_m=30, height_m=25, resolution=0.05, 
                           origin_x=-15, origin_y=-12.5):
    """
    Generate occupancy grid from obstacle geometry.
    
    Args:
        obstacles: List of obstacle dicts with 'position' and 'size'
        width_m: Grid width in meters
        height_m: Grid height in meters
        resolution: Resolution in meters per cell
        origin_x, origin_y: World coordinates of grid origin (center)
    
    Returns:
        occupancy_grid: 2D numpy array (0=free, 100=occupied, 127=unknown)
    """
    
    # Calculate grid dimensions
    grid_width = int(width_m / resolution)
    grid_height = int(height_m / resolution)
    
    # Initialize grid as unknown (127)
    grid = np.full((grid_height, grid_width), occupancy_value(UNKNOWN), dtype=np.uint8)
    
    # Draw each obstacle
    for obs in obstacles:
        x = obs['position']['x']
        y = obs['position']['y']
        sx = obs['size']['x']
        sy = obs['size']['y']
        
        # Convert to grid coordinates
        # Grid origin (0,0) corresponds to origin_x, origin_y in world coords
        x_min_grid = int((x - sx/2 - origin_x) / resolution)
        x_max_grid = int((x + sx/2 - origin_x) / resolution)
        y_min_grid = int((y - sy/2 - origin_y) / resolution)
        y_max_grid = int((y + sy/2 - origin_y) / resolution)
        
        # Clamp to grid bounds
        x_min_grid = max(0, min(grid_width - 1, x_min_grid))
        x_max_grid = max(0, min(grid_width - 1, x_max_grid))
        y_min_grid = max(0, min(grid_height - 1, y_min_grid))
        y_max_grid = max(0, min(grid_height - 1, y_max_grid))
        
        # Mark as occupied
        if x_min_grid < x_max_grid and y_min_grid < y_max_grid:
            grid[y_min_grid:y_max_grid, x_min_grid:x_max_grid] = occupancy_value(OCCUPIED)
    
    # Fill remaining unknown cells as free (reachable floor)
    grid[grid == occupancy_value(UNKNOWN)] = occupancy_value(FREE)
    
    return grid


def occupancy_value(state):
    """Map state to PGM value: 0=free, 254=occupied, 127=unknown."""
    if state == 'free':
        return 254
    elif state == 'occupied':
        return 0
    else:  # unknown
        return 127


FREE, OCCUPIED, UNKNOWN = 'free', 'occupied', 'unknown'


def save_map(grid, output_name, output_dir='./', resolution=0.05, 
             origin=(-15, -12.5, 0)):
    """
    Save occupancy grid as PGM + YAML files.
    
    Args:
        grid: 2D numpy array with occupancy values
        output_name: Prefix for output files (no extension)
        resolution: Map resolution in meters/cell
        origin: (x, y, z) world coordinates of grid origin (bottom-left)
    """
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Convert grid to image (flip Y for image coordinates)
    # PGM: 0=black (occupied), 254=white (free)
    img_array = np.flipud(grid)
    img = Image.fromarray(img_array, mode='L')
    
    # Save PGM
    pgm_path = os.path.join(output_dir, f'{output_name}.pgm')
    img.save(pgm_path)
    
    # Save YAML
    yaml_data = {
        'image': f'{output_name}.pgm',
        'resolution': resolution,
        'origin': list(origin),
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    
    yaml_path = os.path.join(output_dir, f'{output_name}.yaml')
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
    
    print(f"[INFO] Saved map files:")
    print(f"  {pgm_path} ({img_array.shape[1]}x{img_array.shape[0]} pixels)")
    print(f"  {yaml_path}")
    print(f"\n[INFO] Map info:")
    print(f"  Resolution: {resolution} m/cell")
    print(f"  Origin: {origin}")
    print(f"  Dimensions: {img_array.shape[1] * resolution:.1f}m x {img_array.shape[0] * resolution:.1f}m")


def main():
    parser = argparse.ArgumentParser(
        description='Generate ground-truth maps from world geometry'
    )
    parser.add_argument('--world', required=True,
                        help='Path to SDF world file')
    parser.add_argument('--output', required=True,
                        help='Output file prefix (without extension)')
    parser.add_argument('--output-dir', default='./',
                        help='Output directory')
    parser.add_argument('--width', type=float, default=30,
                        help='Map width in meters')
    parser.add_argument('--height', type=float, default=25,
                        help='Map height in meters')
    parser.add_argument('--resolution', type=float, default=0.05,
                        help='Map resolution in meters/cell (must match SLAM resolution)')
    
    args = parser.parse_args()
    
    # Parse world geometry
    analyzer = WorldGeometryAnalyzer(args.world)
    obstacles = analyzer.get_obstacles()
    
    print(f"[INFO] Detected {len(obstacles)} obstacles in world file")
    for obs in obstacles[:5]:  # Print first 5
        print(f"  - {obs['name']}: pos=({obs['position']['x']:.1f}, {obs['position']['y']:.1f}), "
              f"size=({obs['size']['x']:.1f}x{obs['size']['y']:.1f})")
    if len(obstacles) > 5:
        print(f"  ... and {len(obstacles) - 5} more")
    
    # Generate occupancy grid
    origin_x = -args.width / 2
    origin_y = -args.height / 2
    grid = generate_occupancy_grid(
        obstacles,
        width_m=args.width,
        height_m=args.height,
        resolution=args.resolution,
        origin_x=origin_x,
        origin_y=origin_y
    )
    
    # Save map
    save_map(grid, args.output, args.output_dir, args.resolution,
             origin=(origin_x, origin_y, 0))


if __name__ == '__main__':
    main()
