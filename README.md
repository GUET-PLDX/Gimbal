# Gimbal

该模块用 PDF《二轴云台末端指向控制》的向量反解生成小 Yaw/Pitch 关节目标。两个 Yaw 轴均使用 `YawSmc`：大 Yaw 跟踪世界角参考，小 Yaw 跟踪 IK 关节角；Pitch 使用 LibXR 双环 PID。

## 控制链

`TwoAxisIK::Solve()` 仅使用目标的世界 Yaw/Pitch 构造视轴，目标 Roll 不参与指向；基座姿态通过 ZYX 旋转矩阵补偿。默认 `imu_at_end: true`，用末端 IMU 姿态和两轴编码器角反求基座姿态；IMU 若安装在固定基座上，应设为 `false`。Pitch 目标按配置限位裁剪，Yaw 在允许的角度分支中选择离当前角最近的一支，天顶/天底处保留当前 Yaw。

IK 的零点和方向配置采用物理关节坐标：`joint = sign * (encoder - zero)`。`yaw_sign` 和 `pitch_sign` 只接受 `+1` 或 `-1`；小 Yaw SMC 使用物理关节角和编码器角速度，输出力矩再映射回电机方向。大 Yaw SMC 使用 IMU 世界 Yaw/角速度；启用 `yaw_follow` 时，小 Yaw 偏差只改变大 Yaw SMC 的参考角，不经过 PID。`j_yaw`、`j_syaw` 和 `j_pit` 需按机构重新辨识和整定。

## 电机角色

- `motor_syaw`：PDF 所定义的小 Yaw 关节，参与 IK，并由独立 SMC 跟踪 IK 角度。
- `motor_pit`：Pitch 关节，参与 IK 和 Pitch 双环 PID。
- `motor_yaw`：大 Yaw 执行器，由独立 SMC 跟踪世界 Yaw；`yaw_follow.enabled` 时参考角叠加实测小 Yaw 偏差，同样由 SMC 执行跟随。
- `motor_fold`：目前只更新反馈。

手动摇杆输入继续按控制周期积分成世界姿态目标；AI 命令的 Yaw/Pitch 作为绝对角输入。输入姿态或陀螺仪超时、关节电机离线、周期异常或解算结果无效时，控制输出进入 RELAX。Pitch 限位以 `two_axis_ik.pitch_min_rad` 和 `pitch_max_rad` 为准。系统辨识的 `YAW_ONLY` 现在激励小 Yaw 电机，辨识前须再次确认机构行程、速度上限和参数，不应直接用旧配置上电辨识。

## 参数契约

XRobot 按 `Gimbal::Param`、`ControlParam`、`YawFollowParam` 和 `TwoAxisIK::Config` 的声明顺序展开映射。配置结构和默认值回归见 `tests/gimbal_config_order_regression.py` 与 `tests/gimbal_config_fixture.yaml`。一个 `ControlParam` 的字段顺序为：

```yaml
pid_pit_angle: {k: 1.0, p: 2.0, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 8.0, cycle: false}
pid_pit_omega: {k: 1.0, p: 0.4, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 2.0, cycle: false}
yaw_smc: {c: 20.0, k: 120.0, epsilon: 0.5, q: 21.0, p: 27.0, error_deadband_rad: 0.0, ftsmc_switch_rad: 0.0174532925, sat_boundary: 1.0, ftsmc_enable: true}
syaw_smc: {c: 20.0, k: 120.0, epsilon: 0.5, q: 21.0, p: 27.0, error_deadband_rad: 0.0, ftsmc_switch_rad: 0.0174532925, sat_boundary: 1.0, ftsmc_enable: true}
```

`manual` 和 `auto_aim` 各自包含 Pitch 双环 PID、`yaw_smc` 和 `syaw_smc`；两路 SMC 共用 `yaw_output_limits`，但分别使用 `j_yaw` 与 `j_syaw`。迁移固件 YAML 时，需加入两个 SMC 参数块、`yaw_output_limits`、`two_axis_ik` 和 `yaw_follow`，并确认 `motor_yaw`、`motor_syaw`、`motor_pit` 三个实例被绑定。仓库测试夹具只守护模块接口，不会自动修改使用该模块的固件配置。

## 检查

```sh
bash tests/yaw_smc_host_regression.sh
bash tests/two_axis_ik_host_regression.sh
bash tests/system_identify_host_regression.sh
```

IK host 用例覆盖单位姿态、底座 Roll 耦合、末端 IMU 反解、零点/方向、限位、Yaw 角度分支和天顶奇异点。
