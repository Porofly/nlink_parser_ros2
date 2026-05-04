std_msgs/Header header
uint8 role
uint8 id
uint32 local_time
uint32 system_time
float32 voltage
# pos_3d/eop_3d/vel_3d/angle_3d/quaternion/imu_*: valid only in LP mode (TAG).
# In DR mode these fields are zero (datasheet V2.3 §5.1.2).
float32[3] pos_3d
float32[3] eop_3d
float32[3] vel_3d
float32[3] angle_3d
float32[4] quaternion
float32[3] imu_gyro_3d
float32[3] imu_acc_3d
LinktrackNode2[] nodes
