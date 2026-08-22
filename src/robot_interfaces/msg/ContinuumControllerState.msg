# PCC-NMPC + 局部几何控制器的统一诊断状态。

uint8 STATE_DISABLED=0
uint8 STATE_WAITING_FOR_DATA=1
uint8 STATE_ACTIVE=2
uint8 STATE_DEGRADED=3
uint8 STATE_FAULT=4

uint8 SENSOR_VISION=1
uint8 SENSOR_IMU1=2
uint8 SENSOR_IMU2=4

std_msgs/Header header

uint8 state
bool controller_enabled
bool target_valid
bool motor_feedback_valid
bool pcc_valid
bool safety_valid
bool nmpc_solution_valid
bool output_enabled

# 当前局部控制实际使用的传感器位掩码。
uint8 local_measurement_mask

float64 target_age_sec
float64 continuum_state_age_sec
float64 motor_feedback_age_sec

float64 nmpc_solve_time_ms
float64 nmpc_objective_value

# 同一时刻 PCC 名义输出减去实测输出。
geometry_msgs/Vector3 tip_position_model_minus_measured_m
geometry_msgs/Vector3 mid_orientation_model_minus_measured_rad
geometry_msgs/Vector3 tip_orientation_model_minus_measured_rad

# 六维数组统一按物理 tendon_id 1..6 / motor_id 1..6 排列。
float64[6] measured_tendon_length_mm
float64[6] nmpc_tendon_velocity_mm_s
float64[6] local_tendon_velocity_mm_s
float64[6] applied_tendon_velocity_mm_s
float64[6] motor_target_deg

string target_mode
string solver_status
string fault_reason
string status
