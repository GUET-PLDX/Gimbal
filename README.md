# Gimbal

小 Yaw 和 Pitch 直接跟踪世界姿态目标，使用云台 IMU 姿态和角速度闭环。两个 Yaw 轴保留各自的 `YawSmc`，Pitch 使用 LibXR 双环 PID。

## 控制链

`gimbal_euler` 提供姿态反馈，`gimbal_gyro` 的 Z/Y 分量分别提供小 Yaw/Pitch 角速度反馈。关节角采用 `joint = encoder - zero`，用于发布相对角度、Pitch 限位和大 Yaw 跟随。小 Yaw 不做软件角度限位。`syaw_zero`、`pit_zero` 和 `yaw_zero` 分别是小 Yaw、Pitch 和大 Yaw 的编码器零点。

## 电机角色

- `motor_syaw`：由独立 SMC 跟踪世界 Yaw 目标。
- `motor_pit`：由双环 PID 跟踪世界 Pitch 目标。
- `motor_yaw`：大 Yaw 执行器，正常闭环中由独立 SMC 使用编码器反馈跟随小 Yaw 偏差，死区由 `yaw_follow_deadband_rad` 配置。
- `motor_fold` 通过编码器角度和电机角速度闭环。主控使用单独配置的滑模参数解算力矩。导航 `TOP` 对应展开位 `π/2 rad`，`BOTTOM` 对应折叠位 `0 rad`。

折叠轴从当前编码器角平滑移向请求端点，到位后继续闭环保持。`lifter_current_pos` 由编码器角和端点容差判定，`transform_state` 从展开位的 `0` 增至折叠位的 `100`。折叠轴的初始参考速度 `2 rad/s` 和到位容差 `0.1 rad` 取自 `foldable_gimbal`。惯量 `0.001 kg·m²`、力矩上限 `1 N·m` 及滑模增益为台架试调初值。联调前，在达妙上位机将折叠电机的 `PMAX` 设为 `2π rad`（约 `6.283185 rad`）并保存，与固件 `DM4310_PMAX` 的反馈解码量程一致。

手动摇杆输入继续按控制周期积分成世界姿态目标。AI 命令的 Yaw/Pitch 作为绝对角输入。两路 `YawSmc` 按 QDU TSMC 解算当前周期的力矩并做单级对称限幅。系统辨识使用编码器角速度，`YAW_ONLY` 对应小 Yaw。辨识前须确认机构行程、速度上限和参数。

大、小 Yaw 从零力矩起步或反向时可在当拍达到幅值上限。周期延迟超过 20 ms 时仍按当前参考和反馈解算。非有限输入可能产生非有限力矩，由电机接口接收。

`ThreadFunc()` 每周期依次调用 `Update()`、`ParseCMD()` 和 `Control()`。`Update()` 刷新并发布反馈，`ParseCMD()` 更新控制目标。常规模式下，`Control()` 限制 Pitch 目标并解算力矩，再按各电机的反馈状态选择使能、清错或下发力矩。Pitch 解算得到非有限力矩时，仅该轴使用零力矩。`SetMode()` 集中处理模式进入动作。

## 参数契约

XRobot 按 `Gimbal::Param` 和 `ControlParam` 的声明顺序展开映射。产品配置和非对称回中门限的生成回归位于整车工程的 `tools/tests/gimbal_autoaim/test_config.py`。一个 `ControlParam` 的字段顺序为：

```yaml
pid_pit_angle: {k: 1.0, p: 2.0, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 8.0, cycle: false}
pid_pit_omega: {k: 1.0, p: 0.4, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 2.0, cycle: false}
yaw_smc: {c: 20.0, k: 120.0, epsilon: 0.5, q: 21.0, p: 27.0, error_deadband_rad: 0.0, ftsmc_switch_rad: 0.0174532925, sat_boundary: 1.0, ftsmc_enable: true}
syaw_smc: {c: 20.0, k: 120.0, epsilon: 0.5, q: 21.0, p: 27.0, error_deadband_rad: 0.0, ftsmc_switch_rad: 0.0174532925, sat_boundary: 1.0, ftsmc_enable: true}
fold_smc: {c: 20.0, k: 120.0, epsilon: 0.5, q: 21.0, p: 27.0, error_deadband_rad: 0.0, ftsmc_switch_rad: 0.0174532925, sat_boundary: 1.0, ftsmc_enable: true}
```

`manual` 和 `auto_aim` 各自包含 Pitch 双环 PID、`yaw_smc` 和 `syaw_smc`。大、小 Yaw 的对称力矩幅值上限分别为 `yaw_torque_limit_nm` 和 `syaw_torque_limit_nm`（N·m），转动惯量分别为 `j_yaw` 和 `j_syaw`。省略 `syaw_torque_limit_nm` 时，两轴沿用 `yaw_torque_limit_nm`。

折叠轴在各模式统一读取 `manual.fold_smc`。XRobot 要求 `auto_aim.fold_smc` 也出现在配置中，该字段填 `YawSmc::Param{}`。`Param` 直接配置折叠轴端点 `fold_bottom_rad`、`fold_top_rad`，到位容差 `fold_tolerance_rad`，参考速度 `fold_ramp_rad_s`，惯量 `j_fold` 和力矩上限 `fold_torque_limit_nm`。

小 Yaw 机械边界 `small_yaw_min_rad`、`small_yaw_max_rad` 与普通自瞄回中门限 `auto_aim_recenter_trigger_min_rad`、`auto_aim_recenter_trigger_max_rad` 都是 `Param` 的直接成员，均为相对 `syaw_zero` 的角度。回中完成复用 `yaw_follow_deadband_rad`。

## 检查

```sh
bash tests/yaw_smc_host_regression.sh
bash tests/system_identify_host_regression.sh
```

## 大 Yaw 关节反馈（`nav_yaw_joint`）

`Gimbal::YawJointFeedback` 在 `Gimbal.hpp` 中定义 8 字节小端负载 `{float angle_rad, float velocity_rad_s}`。`Update()` 筛选并发布样本，经导航 USB 链路由 `navigation_tx` 发送。

- 两个字段来自同一帧电机 CAN 反馈：`angle_rad = abs_angle - yaw_zero`，`velocity_rad_s = omega`。电机 `reverse` 已在解码时同时作用于二者；DM 电机 `omega` 为 rad/s，未做额外传动比换算。正方向与现有 `yawmotor_angle` 相同。
- 采样时间不放在负载内，而是写入 LibXR 包头的 48 位微秒时间戳，取值为该帧 CAN 接收时刻（MCU `Timebase` 微秒，截断到 48 位）。不是发布或打包时刻。
- 仅在电机在线、无错误、数值有限且序号更新时发布，同一帧不会重复发送。
- 实机零位 `yaw_zero` 仍是占位值，零位、方向与安装标定由工单 Wanqiq7/Pldx_ws#11 验证。

测试：整车工程 `tools/tests/gimbal_autoaim` 的 `yaw_joint_feedback` 用例，从真实 `Gimbal::Update()` 到 Topic 回调检查采样、时间戳和负载字节。
