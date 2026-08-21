# 底层电机协议命令，仅供硬件执行层使用。
# mode=0: 停止；mode=2: 相对位置位移。
# 厂家模式2不是绝对位置，闭环/MPC不得直接向本消息写绝对目标。
uint8 motor_id
uint8 mode
float32 target_deg
float32 speed_rad_s
