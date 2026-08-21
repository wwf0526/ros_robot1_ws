# 软件绝对位置闭环执行器状态。
std_msgs/Header header

bool controller_enabled
bool target_valid
bool target_timeout
bool safety_valid
bool safety_timeout
bool feedback_valid
bool batch_in_flight
bool all_reached
bool fault_active

uint8[] motor_ids
float32[] target_position_deg
float32[] measured_position_deg
float32[] position_error_deg
float32[] last_command_delta_deg
bool[] motor_reached

string source
string fault_reason
string status
