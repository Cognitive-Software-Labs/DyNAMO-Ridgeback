# Ground-Truth Occupancy Maps

This directory contains hand-authored ground-truth occupancy maps that define the "true reachable area" for each simulation world. These maps are used as reference standards for measuring exploration coverage.

## Purpose

Coverage metric validation requires a ground-truth: "what fraction of the reachable space did the robot discover?" Without this, you can only measure "how many cells did SLAM see," which doesn't tell you what should have been seen.

## Approach: SLAM-Derived with Manual Cleanup

All ground-truth maps are generated using **Approach 1**: teleoperating the robot through the entire reachable space while SLAM builds a map, then manually cleaning up artifacts.

### Workflow per World

#### 1. Launch Manual Mapping Session

```bash
# Terminal 1: Start the manual mapping setup (Gazebo + SLAM + Teleop only)
ros2 launch ridgeback_autonomy manual_mapping.launch.py world:=mock_hospital
```

Supported worlds: `mock_hospital`, `warehouse`, `office`

#### 2. Teleoperate Through Entire Space

- **Gazebo window**: Shows the simulation; control with the keyboard via teleop
- **RViz window**: Visualizes the map being built
- Drive the robot:
  - **Pass 1**: Drive close to walls to define boundaries and detect doorways
  - **Pass 2**: Drive through open spaces to fill interior floor areas
  - Ensure every reachable corridor, room, and passage is traversed
  - Move smoothly; jerky motions create discontinuities in SLAM

**Keyboard controls** (from `clearpath_control/teleop_base_node.py`):
- Arrow keys or `w/a/s/d`: Forward/Strafe/Backward
- `q/e`: Rotate counterclockwise/clockwise
- Other keys vary; check the teleop node docs if needed

#### 3. Save the Map

Once you've thoroughly covered the space (typically 5–15 minutes per world):

```bash
# Terminal 2: Save the map
ros2 run nav2_map_server map_saver_cli -f ground_truth_<world_name>
```

This creates:
- `ground_truth_<world_name>.pgm` (raw map image)
- `ground_truth_<world_name>.yaml` (metadata)

**Default location**: `$HOME/`

Move them to this directory:
```bash
mv ~/ ground_truth_<world_name>.* src/ridgeback_autonomy/config/ground_truth_maps/
```

#### 4. Manual Cleanup in GIMP

Open the generated `.pgm` in GIMP (or image editor of choice):

```bash
gimp ground_truth_<world_name>.pgm
```

Clean up artifacts:

1. **Interior unknown cells** (gray pixels inside rooms): Fill with white (free space).
2. **Speckle noise on walls**: Use despeckle or manual touch-up; walls should be solid black.
3. **Phantom walls in doorways**: If SLAM created false walls in doorway regions, erase them (fill white).
4. **Unreachable areas** (outside building, inside obstacles): Leave gray (unknown) or fill black.

**Save as PNG** (GIMP native), then export back as `.pgm` with the same filename. Keep the `.yaml` unchanged.

#### 5. Validate .yaml Metadata

Check the generated `.yaml` file:

```yaml
image: ground_truth_<world_name>.pgm
resolution: 0.05            # Must match SLAM runtime resolution
origin: [-10.0, -10.0, 0.0] # World coords of bottom-left pixel (LL corner)
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

Key points:
- **resolution**: Should match `slam_toolbox_params.yaml` `resolution` param (typically 0.05 m/cell).
- **origin**: The world coordinates (x, y, theta) of the map's lower-left corner. If the map doesn't align with obstacles during exploration validation runs, adjust these values.

#### 6. Validation Run

Run a short autonomous exploration with the frontier explorer (or your chosen explorer) against the world:

```bash
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=<world_name> explorer:=frontier_explorer time_limit:=1200
```

Compute coverage within the ground-truth map. A well-tuned explorer should reach **>75% coverage** (sanity check).

If coverage is poor, check:
- Is the map in the correct location in the directory?
- Do the origin and resolution match runtime values?
- Are there unreachable areas marked as white (free) that should be black (obstacle)?

## Worlds

### mock_hospital
- **Approach**: SLAM-derived (complex interior with multiple rooms, corridors, doorways)
- **Status**: TODO
- **Notes**: Long corridors and interconnected rooms; two passes recommended for thorough coverage.

### office
- **Approach**: SLAM-derived (medium-complexity office layout)
- **Status**: TODO
- **Notes**: Smaller scale; watch for cubicle shadows and interior walls.

### warehouse
- **Approach**: SLAM-derived (open floor with racks and obstacles)
- **Status**: TODO
- **Notes**: Long sightlines allow SLAM to map quickly; focus on passing between rack rows.

## Resolution Reference

Current SLAM resolution (from `slam_toolbox_params.yaml`):
```yaml
resolution: 0.05  # 5 cm per cell
```

All ground-truth maps must use this same resolution for proper validation overlay.

## File Format

Standard ROS 2 Nav2 occupancy grid:

```
ground_truth_maps/
├── mock_hospital.pgm
├── mock_hospital.yaml
├── office.pgm
├── office.yaml
├── warehouse.pgm
└── warehouse.yaml
```

Each `.pgm` is a grayscale raster (8-bit):
- **Black (0)**: Obstacle / Unreachable
- **Gray (128–200)**: Unknown
- **White (255)**: Free / Reachable

The `.yaml` defines the mapping from pixel coordinates to world coordinates.

## Troubleshooting

1. **Robot gets stuck during teleoperation**: Open SLAM is imperfect and sometimes gets lost in symmetric spaces. If the map diverges:
   - Teleop back to a known region (near spawn)
   - The map should re-converge
   - Or restart with a new session

2. **Map has huge gaps or discontinuities**: SLAM drift. Try lowering teleop speed or doing a fresh pass more carefully.

3. **Origin value seems wrong**: Gazebo worlds have their own coordinate frames. If overlaying ground-truth on a SLAM map from exploration doesn't align:
   - Run a quick exploration run; capture the SLAM map it produces
   - Open both maps in GIMP; estimate the offset
   - Adjust the `.yaml` origin by that offset and re-test

4. **Too much noise in the final map**: After cleanup in GIMP:
   - Apply **Filters > Enhance > Despeckle** to reduce single-pixel noise
   - Use **Filters > Blur > Median Blur** with low radius (2–3 px) for gentle smoothing
   - Use the paintbrush with black/white to touch up small errors

## Next Steps

1. Generate maps for **mock_hospital**, **office**, and **warehouse** using this workflow
2. Commit `.pgm` and `.yaml` files to this directory
3. Update the **Status** field above for each world
4. Run validation exploration sweeps against each map
5. If coverage is consistently <75%, investigate root causes in the explorer config or map accuracy
