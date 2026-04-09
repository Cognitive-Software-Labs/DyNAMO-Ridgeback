# Graph Report - .  (2026-04-09)

## Corpus Check
- 14 files · ~11,744 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 47 nodes · 46 edges · 11 communities detected
- Extraction: 85% EXTRACTED · 15% INFERRED · 0% AMBIGUOUS · INFERRED: 7 edges (avg confidence: 0.82)
- Token cost: 0 input · 0 output

## God Nodes (most connected - your core abstractions)
1. `g1_detection_node Script` - 6 edges
2. `main()` - 5 edges
3. `ridgeback_slam_exploration CMake Project` - 4 edges
4. `G1Detection.msg Custom Message` - 4 edges
5. `load_config()` - 3 edges
6. `add_topdown_xz_from_floor()` - 3 edges
7. `parse_detection_result()` - 3 edges
8. `display_results()` - 3 edges
9. `redraw()` - 3 edges
10. `on_mouse()` - 3 edges

## Surprising Connections (you probably didn't know these)
- `G1 Humanoid in Warehouse with Ridgeback` --conceptually_related_to--> `g1_detection_node Script`  [INFERRED]
  vlm_distance/g1_photos.jpg → src/ridgeback_slam_exploration/CMakeLists.txt
- `G1 Humanoid in Hospital Hallway (Far View)` --conceptually_related_to--> `g1_detection_node Script`  [INFERRED]
  vlm_distance/image copy.png → src/ridgeback_slam_exploration/CMakeLists.txt
- `G1 Humanoid in Hospital Corner (Close View)` --conceptually_related_to--> `g1_detection_node Script`  [INFERRED]
  vlm_distance/image.png → src/ridgeback_slam_exploration/CMakeLists.txt
- `G1 Humanoid in Warehouse with Ridgeback` --conceptually_related_to--> `SpawnG1 Gazebo GUI Plugin`  [INFERRED]
  vlm_distance/g1_photos.jpg → src/ridgeback_slam_exploration/CMakeLists.txt
- `g1_detection_node Script` --conceptually_related_to--> `VLM Distance Requirements`  [INFERRED]
  src/ridgeback_slam_exploration/CMakeLists.txt → vlm_distance/requirements.txt

## Hyperedges (group relationships)
- **G1 Humanoid Detection Pipeline** — cmakelists_g1_detection_node, cmakelists_g1detection_msg, requirements_vlm_deps, requirements_transformers, requirements_torch [INFERRED 0.85]
- **G1 Detection Test/Reference Images** — g1_photos_warehouse_scene, image_copy_g1_hospital_hallway, image_g1_hospital_close [INFERRED 0.80]

## Communities

### Community 0 - "VLM Detection Pipeline"
Cohesion: 0.25
Nodes (10): add_topdown_xz_from_floor(), display_results(), load_config(), main(), parse_detection_result(), Unitree G1 Robot Detection - Local VLM Version (No API Key Required)  What this, Extract bounding boxes from the model's text output.     Llava often outputs box, Visualize detections and top-down coordinates. (+2 more)

### Community 1 - "Build & Packaging"
Cohesion: 0.24
Nodes (11): camera_windows_node Script, g1_detection_node Script, G1Detection.msg Custom Message, ridgeback_slam_exploration CMake Project, SpawnG1 Gazebo GUI Plugin, gz-gui8 (Gazebo GUI), rosidl_default_generators, std_msgs (+3 more)

### Community 2 - "Depth Inspection Tool"
Cohesion: 0.4
Nodes (5): on_mouse(), inspect_depth.py — Interactive Relative Depth Inspector for Depth-Anything-V2  P, Mouse callback: on left-click, record the pixel and refresh the display., Redraw both display frames, overlaying a marker at the last clicked point., redraw()

### Community 3 - "SpawnG1 Gazebo Plugin"
Cohesion: 0.67
Nodes (0): 

### Community 4 - "SLAM Launch"
Cohesion: 0.67
Nodes (0): 

### Community 5 - "VLM Dependencies"
Cohesion: 0.67
Nodes (3): PyTorch, Hugging Face Transformers, VLM Distance Requirements

### Community 6 - "Exploration Launch"
Cohesion: 1.0
Nodes (0): 

### Community 7 - "Nav2 Launch"
Cohesion: 1.0
Nodes (0): 

### Community 8 - "Simulation Launch"
Cohesion: 1.0
Nodes (0): 

### Community 9 - "Full Exploration Launch"
Cohesion: 1.0
Nodes (0): 

### Community 10 - "Top-Down Visualization"
Cohesion: 1.0
Nodes (0): 

## Knowledge Gaps
- **14 isolated node(s):** `Unitree G1 Robot Detection - Local VLM Version (No API Key Required)  What this`, `Load camera constants from a JSON file.`, `Estimate top-down (x, z) using camera geometry.`, `Extract bounding boxes from the model's text output.     Llava often outputs box`, `Visualize detections and top-down coordinates.` (+9 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **Thin community `Exploration Launch`** (2 nodes): `explore.launch.py`, `generate_launch_description()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Nav2 Launch`** (2 nodes): `nav2.launch.py`, `generate_launch_description()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Simulation Launch`** (2 nodes): `simulation.launch.py`, `generate_launch_description()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Full Exploration Launch`** (2 nodes): `full_exploration.launch.py`, `generate_launch_description()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Top-Down Visualization`** (1 nodes): `top_down_matplot.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `g1_detection_node Script` connect `Build & Packaging` to `VLM Dependencies`?**
  _High betweenness centrality (0.048) - this node is a cross-community bridge._
- **Are the 5 inferred relationships involving `g1_detection_node Script` (e.g. with `G1Detection.msg Custom Message` and `VLM Distance Requirements`) actually correct?**
  _`g1_detection_node Script` has 5 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Unitree G1 Robot Detection - Local VLM Version (No API Key Required)  What this`, `Load camera constants from a JSON file.`, `Estimate top-down (x, z) using camera geometry.` to the rest of the system?**
  _14 weakly-connected nodes found - possible documentation gaps or missing edges._