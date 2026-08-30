"""ROS topic contracts for the reusable target-localization stack.

Keep wire names here so producers, consumers, launch factories, and benchmark
instrumentation cannot silently drift onto different topics.
"""

RAW_DETECTIONS_TOPIC = 'detections/target/raw'
POINTCLOUD_MEASUREMENTS_TOPIC = 'measurements/target/pointcloud'
MASK_MEASUREMENTS_TOPIC = 'measurements/target/mask'

MASK_DEBUG_TOPIC = 'debug/target/mask'
ALIGNED_DEPTH_DEBUG_TOPIC = 'debug/target/mask/aligned_depth'
OVERLAY_IMAGE_TOPIC = 'debug/target/overlay'

ESTIMATE_MARKERS_TOPIC = 'visualization/target/estimates'
POLAR_RAYS_TOPIC = 'visualization/target/polar_rays'

GROUND_TRUTH_TOPIC = 'benchmark/target/ground_truth'
HUD_DISTANCES_PANEL_TOPIC = 'hud/target_distances'
