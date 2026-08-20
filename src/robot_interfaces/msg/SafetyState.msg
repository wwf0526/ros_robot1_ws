# 连续体机器人安全状态

std_msgs/Header header

bool safe_to_control

bool tendon_slack_detected
bool motor_limit_reached
bool imu_timeout
bool motor_timeout

# true = 本次运行的双 IMU runtime zero reference 已建立且仍有效
# 注意：true 不表示机器人当前正位于机械零姿态
bool imu_zero_ready
bool pcc_model_valid
bool residual_too_large
bool emergency_stop_active

float64 max_orientation_residual_deg

string status
