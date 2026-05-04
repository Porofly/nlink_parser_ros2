# DR_MODE1: ranging frame for the unlimited-node distributed mode.
# Reports per-peer distance and RSSI for every other node in range.
# Datasheet V2.3 §5.1.2 (NLINK_LINKTRACK_NODE_FRAME5).
std_msgs/Header header
uint8 role
uint32 id
uint32 local_time
uint32 system_time
float32 voltage
LinktrackNode5[] nodes
