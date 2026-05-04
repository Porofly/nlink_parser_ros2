# LP Mode (ANCHOR/CONSOLE roles): localization frame.
# Reports each in-range tag's coordinates and the distances to nearest anchors.
# Datasheet V2.3 §5.1.2 (NLINK_LINKTRACK_ANCHOR_FRAME0).
std_msgs/Header header
uint8 role
uint8 id
uint32 local_time
uint32 system_time
float32 voltage
LinktrackTag[] nodes
