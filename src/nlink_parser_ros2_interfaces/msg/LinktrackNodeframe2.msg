# LP Mode (TAG role) and DR_MODE0: localization frame.
# In LP Mode the pose / IMU fields carry the tag's solved position and
# orientation. In DR_MODE0 the hardware does NOT compute a position, so
# pos_3d / eop_3d / vel_3d / angle_3d / quaternion / imu_*_3d are zero;
# only `nodes[]` (per-peer distance and RSSI) is meaningful.
# Datasheet V2.3 §5.1.2 (NLINK_LINKTRACK_NODE_FRAME2).
std_msgs/Header header
uint8 role
uint8 id
uint32 local_time
uint32 system_time
float32 voltage
float32[3] pos_3d
float32[3] eop_3d
float32[3] vel_3d
float32[3] angle_3d
float32[4] quaternion
float32[3] imu_gyro_3d
float32[3] imu_acc_3d
LinktrackNode2[] nodes
