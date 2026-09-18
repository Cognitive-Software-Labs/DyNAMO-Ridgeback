# Header uses the selected ROS clock. Liveness is evaluated with receiver monotonic time.
std_msgs/Header header
string session_id
string mode
# startup | processing | stale_input | stalled | failure
string state
string detail
string[] estimators
string depth_source
string mask_gate
string[] labels
string[] streams
uint64[] progress
builtin_interfaces/Time[] source_stamps
