# Who the leader is and whether the robot may follow. Only LOCKED may drive motion.
std_msgs/Header header
uint8 SEARCHING=0   # no leader yet
uint8 LOCKED=1      # leader confirmed this cycle
uint8 UNCERTAIN=2   # leader missing or not matching; hold still
uint8 LOST=3        # unconfirmed for too long; stop and search
uint8 state
string reason                   # machine-readable cause, e.g. "target_missing", "identity_recovered"
uint32 track_id                 # leader's Person.id; meaningful only when has_leader is true
bool has_leader
geometry_msgs/Point position    # leader position in the header frame when has_leader is true
