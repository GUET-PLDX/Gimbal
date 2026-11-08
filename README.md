# Gimbal

该模块用 PDF《二轴云台末端指向控制》的向量反解替换小 Yaw 的原滑模控制链路。控制任务以目标世界姿态、IMU 姿态和小 Yaw/Pitch 编码器角为输入，先计算末端 X 轴在基座系的指向，再解算关节角，最后用 LibXR 双环 PID 跟踪两个关节目标。

## 控制链

`TwoAxisIK::Solve()` 仅使用目标的世界 Yaw/Pitch 构造视轴，目标 Roll 不参与指向；基座姿态通过 ZYX 旋转矩阵补偿。默认 `imu_at_end: true`，用末端 IMU 姿态和两轴编码器角反求基座姿态；IMU 若安装在固定基座上，应设为 `false`。Pitch 目标按配置限位裁剪，Yaw 在允许的角度分支中选择离当前角最近的一支，天顶/天底处保留当前 Yaw。

IK 的零点和方向配置采用物理关节坐标：`joint = sign * (encoder - zero)`。`yaw_sign` 和 `pitch_sign` 只接受 `+1` 或 `-1`；角度环在物理坐标中闭环，输出力矩再映射回电机编码器方向。反馈速度来自电机编码器，不从 IMU 陀螺仪替代关节速度环反馈。`j_syaw` 和 `j_pit` 只用于有限幅值的惯性前馈，需按新控制链重新辨识和整定。

## 电机角色

- `motor_syaw`：PDF 所定义的小 Yaw 关节，参与 IK 和 Yaw 双环 PID。
- `motor_pit`：Pitch 关节，参与 IK 和 Pitch 双环 PID。
- `motor_yaw`：独立的大 Yaw 执行器，不进入二轴 IK；仅在 `yaw_follow.enabled` 时按小 Yaw 相对偏角回中跟随。
- `motor_fold`：目前只更新反馈。

手动摇杆输入继续按控制周期积分成世界姿态目标；AI 命令的 Yaw/Pitch 作为绝对角输入。输入姿态或陀螺仪超时、关节电机离线、周期异常或解算结果无效时，控制输出进入 RELAX。Pitch 限位以 `two_axis_ik.pitch_min_rad` 和 `pitch_max_rad` 为准。系统辨识的 `YAW_ONLY` 现在激励小 Yaw 电机，辨识前须再次确认机构行程、速度上限和参数，不应直接用旧配置上电辨识。

## 参数契约

XRobot 按 `Gimbal::Param`、`ControlParam`、`YawFollowParam` 和 `TwoAxisIK::Config` 的声明顺序展开映射。配置结构和默认值回归见 `tests/gimbal_config_order_regression.py` 与 `tests/gimbal_config_fixture.yaml`。一个 `ControlParam` 的字段顺序为：

```yaml
pid_yaw_angle: {k: 1.0, p: 2.0, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 8.0, cycle: true}
pid_yaw_omega: {k: 1.0, p: 0.4, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 2.0, cycle: false}
pid_pit_angle: {k: 1.0, p: 2.0, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 8.0, cycle: false}
pid_pit_omega: {k: 1.0, p: 0.4, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 2.0, cycle: false}
```

`manual` 和 `auto_aim` 各自包含以上四个 PID；Pitch/Yaw 机械范围、编码器零点和方向放在 `two_axis_ik`。迁移固件 YAML 时，需把原来 `yaw_smc`、`yaw_output_limits` 和独立 Pitch 限位字段替换为新 `param` 结构，并确认 `motor_syaw`、`motor_pit` 两个实例被绑定。仓库测试夹具只守护模块接口，不会自动修改使用该模块的固件配置。

## 检查

```sh
bash tests/two_axis_ik_host_regression.sh
bash tests/system_identify_host_regression.sh
```

IK host 用例覆盖单位姿态、底座 Roll 耦合、末端 IMU 反解、零点/方向、限位、Yaw 角度分支和天顶奇异点。
