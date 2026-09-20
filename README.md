# Gimbal

## 1. 模块作用
云台控制模块。实现 roll/yaw 闭环控制和模式切换。

### Yaw SMC/FTSMC 数学约定

`YawSmc` 基于参考工程的 SMC/FTSMC 结构实现，并使用 SI 单位和力矩接口：角度为 `rad`，角速度为 `rad/s`，角加速度为 `rad/s^2`，力矩为 `N*m`。当前实现直接消费调用方提供的目标角度、角速度和角加速度，不使用目标角度的无周期差分。

Yaw 误差几何与手动 PID 相同，使用 `CycleValue` 最短路（`e = θ - θd`，结果在 `[-π, π]`），避免 `[0, 2π]` 目标与 `atan2` 反馈直接相减走出长弧：

```text
e = CycleValue(theta) - theta_ref
e_dot = omega - omega_ref
tau_ff_alpha = J * alpha_ref
```

`Reference::theta_rad`、`Reference::omega_rad_s` 和 `Reference::alpha_rad_s2` 均直接参与控制律。调用方负责提供同一参考轨迹的完整运动学状态。

当 `abs(e) < error_deadband_rad` 时立即返回零控制量，边界等号仍计算。当 `abs(e) >= ftsmc_switch_rad` 时使用 FTSMC，否则使用线性 SMC。FTSMC 的幂次符号函数及其导数为：

```text
sig_r(e) = sign(e) * abs(e)^r
r = q / p
s = e_dot + c * sig_r(e)
surface_dot_term = c * r * abs(e)^(r - 1) * e_dot
tau_smc = J * (-surface_dot_term - epsilon * Sat(s) - k * s)
```

导数项中的 `abs(e)^(r - 1)` 不包含 `sign(e)`，因此正负镜像状态具有一致的动力学。FTSMC 只在远离零点的配置区间使用，以避开 `r < 1` 时的原点奇异性。

核心输出为 `tau_pre_limit_nm = tau_ff_alpha_nm + tau_smc_nm`，其中 `tau_ff_alpha_nm = J * alpha_ref`。工程随后依次应用软限幅、硬限幅和力矩变化率限制；`tau_cmd_before_slew_nm` 是前两项之后、slew 之前的值，`tau_cmd_nm` 才是提交给电机的最终受保护命令。

运行时保留参考代码的宽松 `p/q` 行为，不增加奇数校验。修复参考运动学与 FTSMC 负误差方向后，移动目标和负方向输出会与旧实现不同。升级后应重新执行正负角度阶跃、匀速目标和加速目标测试，再进行实车参数整定。

## 2. 主要函数说明
1. ThreadFunc: 云台控制主线程。
2. ParseCMD: 解析 CMD 输入并更新目标。
3. Control: 角度环与角速度环计算控制输出。
4. Update: 刷新电机反馈并发布状态。
5. SetMode / GetEvent: 模式管理与事件接口。
6. DebugCommand: 调试命令入口（Debug 构建）。

## 3. 接入步骤
1. 添加模块并绑定 motor_roll、motor_yaw、cmd。
2. 配置零位、限位、惯量与 PID 参数。
3. 手动 Yaw 用 `yaw_manual_controller` 选择 `YawManualController::PID` 或 `YawManualController::SMC`。自瞄 Yaw 用 `yaw_ai_controller` 选择 `YawAiController::SMC` 或 `YawAiController::LQR_ESO`。两套选择独立；LQR/ESO 仅自瞄可用。
4. 先验证模式切换，再联调控制参数。首次启用滑模时降低 `yaw_smc.torque_soft_limit_nm`，确认力矩极性后再抬升。

云台姿态输入 topic：
- `gimbal_cmd`：CMD 发布的云台控制命令。
- `gimbal_euler`：云台 IMU 融合后的欧拉角。
- `gimbal_gyro`：云台 IMU 原始角速度。
- `chassis_motion_state`：底盘运动状态（在线、模式、yaw 角速度），供小陀螺模式前馈使用。

云台状态输出 topic：
- `yawmotor_angle` / `pitchmotor_angle`：电机绝对角，rad。
- `yawmotor_omega` / `pitchmotor_omega`：与内环同符号的 IMU 实测角速度，rad/s（yaw=`gyro.z`，pitch=`gyro.y` 已按云台约定取反）。仅在陀螺 50 ms 内新鲜时发布。


标准命令流程：
    xrobot_add_mod Gimbal --instance-id gimbal
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: Gimbal
entry_header: Modules/Gimbal/Gimbal.hpp
constructor_args:
  - cmd: '@cmd'
  - task_stack_depth: 2048
  - pid_yaw_angle:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: true
  - pid_yaw_omega:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: true
  - pid_roll_angle:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - pid_roll_omega:
      k: 0.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - motor_roll: '@&motor_roll'
  - motor_yaw: '@&motor_yaw'
  - roll_max_angle: 0.0
  - roll_min_angle: 0.0
  - roll_lc: 0.0
  - roll_theta: 0.0
  - yaw_k: 0.0
  - j_roll: 0.0
  - j_yaw: 0.0
  - roll_zero: 0.0
  - yaw_zero: 0.0
  - patrol_range: 0.0
  - patrol_omega: 0.0
  - roll_reverse_flag: false
  - thread_priority: LibXR::Thread::Priority::MEDIUM
  - rotor_ff_enabled: false
  - yaw_manual_controller: YawManualController::PID
  - yaw_ai_controller: YawAiController::LQR_ESO
  - yaw_smc:
      j_kg_m2: 0.03
      c: 20.0
      k: 120.0
      epsilon: 0.5
      q: 21.0
      p: 27.0
      error_deadband_rad: 0.0
      ftsmc_switch_rad: 0.0174533
      sat_boundary: 1.0
      torque_soft_limit_nm: 2.0
      torque_min_nm: -2.223
      torque_max_nm: 2.223
      torque_slew_rate_nm_s: 1000.0
      ftsmc_enable: true
      torque_slew_enable: true
template_args:
[]

## 5. 依赖与硬件
Required Hardware:
[]

Depends:
  - pldx/CMD
  - pldx/Motor
  - pldx/BMI088

## 6. 代码入口
Modules/Gimbal/Gimbal.hpp
