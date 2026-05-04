# Data-transmission frame (no positioning information).
# Used in LP Mode (all roles), DR_MODE0, and DR_MODE1 to forward arbitrary
# user payloads received from other nodes over UWB.
# Datasheet V2.3 §5.1.2 (NLINK_LINKTRACK_NODE_FRAME0).
std_msgs/Header header
uint8 role
uint8 id
LinktrackNode0[] nodes
