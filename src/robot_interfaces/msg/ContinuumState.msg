#表示机器人当前状态

#两段连续体状态

std_msgs/Header header

# 第一段曲率
float32 section1_kx
float32 section1_ky

# 第二段曲率
float32 section2_kx
float32 section2_ky

# 第一段IMU
float32 imu1_roll
float32 imu1_pitch
float32 imu1_yaw

# 第二段IMU
float32 imu2_roll
float32 imu2_pitch
float32 imu2_yaw

# 两段绳长
float32[] tendon_length_mm

# 松绳标志
bool[] tendon_slack

# ================= PCC model output =================
# 以下字段来自 continuum_pccmodel 正运动学模型
#
# 机器人真实串联顺序：
#   基座 -> section2 -> section1 -> 末端
#
# PCC 计算顺序：
#   T_end = T_section2 * T_section1
#
# 注意：
#   section1 是靠近末端的一段
#   section2 是靠近基端的一段

bool pcc_model_valid

# -------------------------------
# section1 PCC 输入
# section1 对应靠近末端的一段
# YAML 中：
#   [DL1, DL2, DL3] = [绳5, 绳3, 绳1]
# -------------------------------
float64 section1_pcc_dl1_mm
float64 section1_pcc_dl2_mm
float64 section1_pcc_dl3_mm

# section1 局部 PCC 参数
float64 section1_model_theta_rad
float64 section1_model_phi_rad
float64 section1_model_kappa_1pm
float64 section1_model_arc_length_m

# section1 段末端在整体坐标系下的位姿
float64 section1_model_px_m
float64 section1_model_py_m
float64 section1_model_pz_m
float64 section1_model_yaw_rad
float64 section1_model_pitch_rad
float64 section1_model_roll_rad

# -------------------------------
# section2 PCC 输入
# section2 对应靠近基端的一段
# YAML 中：
#   [DL1, DL2, DL3] = [绳4, 绳2, 绳6]
# -------------------------------
float64 section2_pcc_dl1_mm
float64 section2_pcc_dl2_mm
float64 section2_pcc_dl3_mm

# section2 局部 PCC 参数
float64 section2_model_theta_rad
float64 section2_model_phi_rad
float64 section2_model_kappa_1pm
float64 section2_model_arc_length_m

# section2 段末端在整体坐标系下的位姿
float64 section2_model_px_m
float64 section2_model_py_m
float64 section2_model_pz_m
float64 section2_model_yaw_rad
float64 section2_model_pitch_rad
float64 section2_model_roll_rad

# -------------------------------
# 整体末端 PCC 位姿
# -------------------------------
float64 end_model_px_m
float64 end_model_py_m
float64 end_model_pz_m
float64 end_model_yaw_rad
float64 end_model_pitch_rad
float64 end_model_roll_rad
