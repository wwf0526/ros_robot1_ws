#作用：保存两个 IMU 的姿态和是否调平
std_msgs/Header header

float32 imu1_roll_deg
float32 imu1_pitch_deg
float32 imu1_yaw_deg

float32 imu2_roll_deg
float32 imu2_pitch_deg
float32 imu2_yaw_deg

float32 roll_tolerance_deg
float32 pitch_tolerance_deg

bool imu1_level
bool imu2_level
bool robot_level

string status
