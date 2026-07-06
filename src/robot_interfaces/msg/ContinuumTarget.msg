# 连续体机器人目标姿态
# 用于 MPC 控制器订阅 /continuum/target

std_msgs/Header header

# 第一段目标姿态，单位 deg
float32 section1_roll_target
float32 section1_pitch_target

# 第二段目标姿态，单位 deg
float32 section2_roll_target
float32 section2_pitch_target

# 是否启用目标控制
bool enable

# 控制模式说明
# 例如：imu_pose、curvature、tip_pose
string mode
