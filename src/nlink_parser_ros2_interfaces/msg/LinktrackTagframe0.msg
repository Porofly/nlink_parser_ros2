# LP Mode (TAG role): localization frame.
# Includes the tag's own pose, distances to all in-range anchors,
# and IMU raw / fused data (only on hardware variants with an IMU).
# Datasheet V2.3 §5.1.2 (NLINK_LINKTRACK_TAG_FRAME0).
std_msgs/Header header
uint8 role
uint8 id
uint32 local_time
uint32 system_time
float32 voltage
float32[3] pos_3d
float32[3] eop_3d
float32[3] vel_3d
float32[8] dis_arr
float32[3] angle_3d
float32[4] quaternion
float32[3] imu_gyro_3d
float32[3] imu_acc_3d
