# 最终层级控制器目标。
# header.frame_id 必须为 continuum_base；header.stamp 是目标生成时刻。

uint8 MODE_TIP_POSITION=1
uint8 MODE_TIP_POSE=2
uint8 MODE_SECTION_SHAPE=3
uint8 MODE_TIP_POSITION_AND_SHAPE=4
uint8 MODE_TIP_POSE_AND_SHAPE=5

std_msgs/Header header

# false 会撤销目标，并要求层级控制器清除积分量和热启动状态。
bool enable

# 使用上面的 MODE_* 常量。
uint8 mode

# 目标有效期；必须为有限正数。控制器使用 header.stamp + valid_for_sec
# 判断目标是否超时，不能无限使用旧目标。
float64 valid_for_sec

# 在 continuum_base 中定义的末端目标。
geometry_msgs/Point tip_position_m
geometry_msgs/Quaternion tip_orientation

# 两段目标曲率分量，单位 1/m。
# section2 是靠近基座的段，section1 是靠近末端的段。
float64 section1_kx_1pm
float64 section1_ky_1pm
float64 section2_kx_1pm
float64 section2_ky_1pm

# 仅用于日志和目标来源追踪，例如 operator、trajectory_manager。
string source
