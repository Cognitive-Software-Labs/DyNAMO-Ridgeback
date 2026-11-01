# One tracked person.
uint32 id                       # stable track id
geometry_msgs/Point position    # metres, in the enclosing message's header frame
geometry_msgs/Vector3 velocity  # metres/second, same frame
float32 confidence              # detection confidence, 0..1
float32[] appearance            # L2-normalised re-identification embedding; empty while out of camera view
