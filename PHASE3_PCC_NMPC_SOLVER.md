# 阶段3：PCC-NMPC滚动优化核心

本阶段在已验证的六绳/四自由度映射和双节PCC模型上，实现与ROS通信解耦的
非线性MPC求解器。它不会发布电机话题，也不会替换原正运动学或现有2模式电机
闭环。后续统一控制节点只执行每次有效求解结果的第一步。

## 文件变更

| 文件 | 类型 | 作用 |
| --- | --- | --- |
| `continuum_mpc/continuum_mpc/pcc_nmpc_solver.py` | 新增 | 非线性PCC滚动预测、代价函数、硬约束、热启动和失败关闭 |
| `robot_bringup/config/hierarchical_controller_params.yaml` | 新增 | 最终控制器的NMPC频率、预测长度、控制分块、权重和求解参数 |
| `continuum_mpc/continuum_mpc/actuator_mapping.py` | 修改 | 增加6×4驱动矩阵及电机限位到绳长限位的统一换算 |
| `continuum_mpc/test/test_pcc_nmpc_solver.py` | 新增 | 验证目标收敛、六绳协调、速度/位置/曲率约束和失败关闭 |
| `continuum_mpc/test/test_actuator_mapping.py` | 修改 | 增加驱动矩阵与限位换算回归测试 |

## 控制定义

- 当前状态：六根物理绳长变化量，顺序为 `tendon_id 1..6`。
- 独立控制量：
  `[section2_DL1, section2_DL2, section1_DL1, section1_DL2]`，单位mm/s。
- 每段第三根等效绳速度自动取 `-(DL1 + DL2)`，保证三绳模型量之和为零。
- 状态更新：`rho(k+1) = rho(k) + Ts * rho_dot(k)`。
- 每个预测步均调用现有双节非线性PCC正运动学。
- 默认预测6步，并采用2个分段常值控制块降低冷启动求解时间。

## 已实现代价项

- 末端三维位置误差；
- 可选末端姿态旋转误差；
- 可选双节曲率分量误差；
- 未指定形状时的曲率正则；
- 六根物理绳速度代价；
- 相邻控制增量代价；
- 终端预测步加权。

## 已实现硬约束

- 四个独立绳速上限；
- 六根物理绳速上限；
- 全预测域电机绝对位置限位；
- 电机限位软件裕量；
- PCC有效剩余绳长；
- 可选双节最大曲率。

曲率上限必须通过实机安全弯曲试验标定，因此配置中默认关闭。求解超时、失败、
结果不可行或当前状态越界时，`result.valid=false`且返回零控制序列。

## 参数归属

30mm卷线轮半径、电机方向、PCC段长、绳孔半径和电机机械限位仍只在
`robot_calibration.yaml`中维护。`hierarchical_controller_params.yaml`只维护控制
频率、预测长度、速度上限、代价权重和数值求解参数，不重复保存物理尺寸。

## 构建与验证

```bash
cd ~/ros_robot1_ws
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select \
  continuum_pccmodel continuum_mpc robot_bringup

source ~/ros_robot1_ws/install/setup.bash

python3 -m pytest -q \
  src/continuum_mpc/test/test_actuator_mapping.py \
  src/continuum_mpc/test/test_pcc_control_model.py \
  src/continuum_mpc/test/test_pcc_nmpc_solver.py
```

预期结果：`18 passed`。

下一阶段将在此求解器旁新增9维任务优先级局部几何控制器，之后再由唯一ROS节点
组合NMPC与局部补偿、进行统一约束并发布 `/motor/position_target`。
