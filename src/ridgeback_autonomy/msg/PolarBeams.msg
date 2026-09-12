# Which LiDAR beams polar profiling selected and reduced, as indices into the
# scan array they were taken from. Debug-only: the 2D overlay panel consumes
# this instead of re-deriving the selection, so the panel cannot disagree with
# the published measurement.
#
# Two stamps, deliberately. The header carries the detection/measurement stamp,
# so a consumer keys this exactly like the silhouette artifact on
# debug/target/mask. scan_stamp names the array the indices are valid against;
# without it a consumer would apply them to whatever scan happened to be latest,
# which is silently wrong rather than visibly wrong.
std_msgs/Header header
builtin_interfaces/Time scan_stamp
string scan_frame_id

# len(scan.ranges) when the indices were recorded. A hard guard, not decoration:
# a consumer whose cached scan is a different length must drop the highlight
# rather than index a mismatched array.
uint32 beam_count

# Union over the frame's detections. selected is what the mask picked before the
# range band; merged is what survived it and fed the estimate. Their difference
# is the selected-but-discarded set -- the distinction the 3D ray layers exist to
# show. The union is intentional: unlike the ray markers, which draw only the
# nearest detection to keep the 3D view readable, a 2D panel has no clutter
# problem and shows every detection.
uint32[] selected
uint32[] merged
