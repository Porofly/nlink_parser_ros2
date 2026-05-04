# LP Mode (ANCHOR/CONSOLE roles): localization frame, all-tag positions.
# Reports the coordinates of every tag observed in this anchor's range.
# Datasheet V2.3 §5.1.2 (NLINK_LINKTRACK_NODE_FRAME1).
std_msgs/Header header
uint8 role
uint8 id
uint32 local_time
uint32 system_time
float32 voltage
LinktrackNode1[] nodes
