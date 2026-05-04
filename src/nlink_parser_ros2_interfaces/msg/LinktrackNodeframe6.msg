# DR_MODE1: data-transmission frame for the unlimited-node distributed mode.
# Forwards user payloads received from other nodes over UWB.
# Datasheet V2.3 §5.1.2 (NLINK_LINKTRACK_NODE_FRAME6).
std_msgs/Header header
uint8 role
uint32 id
LinktrackNode6[] nodes
