# 连续体机器人当前状态
#
# 统一坐标系：
#   frame_id = continuum_base
#   +X = 0 deg
#   +Y = 90 deg
#   +Z = base -> tip
#   从 +Z 看向基座，逆时针为角度正方向

std_msgs/Header header

# ================= legacy/basic state =================

float32 section1_kx
float32 section1_ky

float32 section2_kx
float32 section2_ky

# 兼容字段：现在表示 runtime zero + mount quaternion 后的 aligned IMU Euler，单位 deg
# aligned_valid=false 时填 0
float32 imu1_roll
float32 imu1_pitch
float32 imu1_yaw

float32 imu2_roll
float32 imu2_pitch
float32 imu2_yaw

float32[] tendon_length_mm
bool[] tendon_slack

# ================= PCC model output =================
# base -> section2 -> section1 -> tip

bool pcc_model_valid

# section1 PCC input: [DL1, DL2, DL3] = [绳5, 绳3, 绳1]
float64 section1_pcc_dl1_mm
float64 section1_pcc_dl2_mm
float64 section1_pcc_dl3_mm

float64 section1_model_theta_rad
float64 section1_model_phi_rad
float64 section1_model_kappa_1pm
float64 section1_model_arc_length_m
float64 section1_model_center_length_m
float64 section1_model_l0_m

float64 section1_model_px_m
float64 section1_model_py_m
float64 section1_model_pz_m
float64 section1_model_yaw_rad
float64 section1_model_pitch_rad
float64 section1_model_roll_rad

# section2 PCC input: [DL1, DL2, DL3] = [绳4, 绳2, 绳6]
float64 section2_pcc_dl1_mm
float64 section2_pcc_dl2_mm
float64 section2_pcc_dl3_mm

float64 section2_model_theta_rad
float64 section2_model_phi_rad
float64 section2_model_kappa_1pm
float64 section2_model_arc_length_m
float64 section2_model_center_length_m
float64 section2_model_l0_m

float64 section2_model_px_m
float64 section2_model_py_m
float64 section2_model_pz_m
float64 section2_model_yaw_rad
float64 section2_model_pitch_rad
float64 section2_model_roll_rad

float64 end_model_px_m
float64 end_model_py_m
float64 end_model_pz_m
float64 end_model_yaw_rad
float64 end_model_pitch_rad
float64 end_model_roll_rad

# ================= orientation fusion =================

bool imu_zero_ready
bool imu1_aligned_valid
bool imu2_aligned_valid

geometry_msgs/Quaternion section1_model_orientation
geometry_msgs/Quaternion section2_model_orientation
geometry_msgs/Quaternion end_model_orientation

geometry_msgs/Quaternion imu1_aligned_orientation
geometry_msgs/Quaternion imu2_aligned_orientation

geometry_msgs/Quaternion section1_fused_orientation
geometry_msgs/Quaternion section2_fused_orientation
geometry_msgs/Quaternion end_fused_orientation

bool section1_fused_valid
bool section2_fused_valid

float64 section1_orientation_residual_deg
float64 section2_orientation_residual_deg

# ================= reserved vision fusion interface =================
# 当前阶段 vision_tip_valid=false，end_fused_position_m 默认等于 PCC 位置。
# 后续接 AprilTag 时无需再次修改消息接口。

bool vision_tip_valid
geometry_msgs/Point end_vision_position_m
geometry_msgs/Point end_fused_position_m
float64 vision_position_residual_m
