# LP Mode (TAG role) and DR_MODE0: ranging-only frame.
# Subset of NodeFrame2: omits the pose/IMU block, keeps per-peer distance
# and RSSI. Use this if you only need ranging and not the LP-mode pose.
# Datasheet V2.3 §5.1.2 (NLINK_LINKTRACK_NODE_FRAME3).
std_msgs/Header header
uint8 role
uint8 id
uint32 local_time
uint32 system_time
float32 voltage
LinktrackNode2[] nodes
