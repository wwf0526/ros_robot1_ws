# 电机绝对位置闭环执行层 v1

## 设计结论

厂家协议模式 2 的角度字段表示相对位移，不能接收 MPC 的累计绝对位置。
最终控制链因此固定为：

1. MPC、回零或人工测试发布六电机零点坐标绝对目标；
2. `motor_position_controller` 根据编码器反馈计算绝对误差；
3. 误差按每电机 `max_delta_deg_per_step` 分批限幅；
4. 只有上一批命令产生了更新后的 `reached=true` 反馈，才允许发送下一批；
5. `motor_node` 只接受停止或受限的模式 2 相对位移命令。

## 正式接口

- 绝对目标：`/motor/position_target`，`MotorPositionTargetArray`。
- 原始执行：`/motor/raw_command_array`，`MotorCommandArray`。
- 编码器反馈：`/motor/state`，`MotorState`。
- 闭环状态：`/motor/position_control_state`，`MotorPositionControlState`。
- 显式使能：`/motor_controller/set_enabled`。
- 清除故障：`/motor_controller/clear_fault`。
- 回零目标：`/motor/home_motors`。

## 安全状态机

闭环控制器默认禁止。以下任一条件会停止电机、禁止闭环并锁存故障：

- 六电机任一路反馈超时；
- `SafetyState` 超时或 `safe_to_control=false`；
- 急停激活；
- MPC/外部绝对目标心跳超时；
- 一批相对位移命令在规定时间内没有完成；
- 目标缺少电机、包含重复 ID、非有限数值、越过位置限位或速度限位；
- 反馈或闭环计算出现非有限数值。

故障恢复必须依次执行：恢复反馈与安全状态、清除故障、显式使能、发送新目标。

## CAN 反馈

真实驱动为 `can0` 和 `can1` 各建立一个接收线程，反馈请求由 ROS 定时器轮询发送。
接收线程只更新带序号的缓存；ROS 节点仅在收到新反馈时发布 `MotorState`，因此旧缓存
不会伪装成新反馈，也不会掩盖状态估计器的 `motor_timeout`。

## MPC 边界

`qp_mpc_node` 已改为发布绝对电机目标，并在第一次使能时从六电机反馈初始化
`theta_cmd_deg`。旧的简化 `mpc_node` 已从最终执行路径移除。当前 QP 的 A/B 模型仍需
系统辨识，本阶段完成的是可供最终 MPC 使用的安全执行层，不代表 B 矩阵已经定稿。
