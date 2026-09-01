# Gimbal

## 1. 模块作用
云台控制模块。实现 roll/yaw 闭环控制和模式切换。

### Yaw SMC/FTSMC 数学约定

`YawSmc` 严格按 `/home/wanqiq/桌面/smc_controller/SMC/slidingmodec.cpp` 的 `SMC_Tick()` 计算顺序移植。工程接口使用 SI 单位：角度为 `rad`，角速度为 `rad/s`，角加速度和力矩字段分别标为 `rad/s^2`、`N*m`。

控制律顺序与 `SMC_Tick()` 一致。Yaw 误差几何与手动 PID 相同，使用 `CycleValue` 最短路（`e = θ − θd`，结果在 `[-π, π]`），避免 `[0, 2π]` 目标与 `atan2` 反馈直接相减走出长弧：

```text
e = CycleValue(theta) - target
target_dot = CycleValue(target) - target_last
target_ddot = target_dot - target_dot_previous
e_dot = omega - target_dot_previous
```

`target_dot` 和 `target_ddot` 来自控制器内部的目标历史，`Reference.omega_rad_s`、`Reference.alpha_rad_s2` 仅为兼容公共接口保留，不参与源算法核心计算。死区判断前更新 `target_dot`，只有完成一次非死区、有限值的控制计算后才更新 `target_last`；这些差分仍不按 `dt` 归一化。`Reset()` 用当前反馈角初始化 `target_last`。

当 `abs(e) < error_deadband_rad` 时立即返回零控制量，边界等号仍计算。源代码先计算 FTSMC，随后在 `abs(e) < ftsmc_switch_rad` 时改用线性滑模，等号进入 FTSMC。FTSMC 保留源代码的负误差符号项：

当前 YAML 的 `ftsmc_switch_rad=0.0174533` 是源代码一单位（原始实现中的 1 度）对应的弧度配置值。

```text
e_qp = sign(e) * abs(e)^(q/p)
s = e_dot + c * e_qp
ds = -epsilon * Sat(s) - k * s
tau_smc = J * (ds - c * (q/p) * e_dot * e_qp / abs(e))
```

核心输出为 `tau_pre_limit_nm = tau_ff_alpha_nm + tau_smc_nm`，其中 `tau_ff_alpha_nm = J * target_ddot`。工程随后依次应用软限幅、硬限幅和力矩变化率限制；`tau_cmd_before_slew_nm` 是前两项之后、slew 之前的值，`tau_cmd_nm` 才是提交给电机的最终受保护命令。

运行时保留源代码的宽松 `p/q` 行为，不增加奇数校验。YAML 中的 `c=20`、`k=120`、`epsilon=0.5`、`J=0.03` 等数值保持为 Mock/初始调参参数，不代表已完成实机物理重整定。

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
- `chassis_gyro_z`：双板底盘侧发布的 Z 轴角速度，供小陀螺模式前馈使用。
- `dualboard_chassis_mode`：双板底盘模式；仅 ROTOR 模式激活底盘角速度前馈。


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
  - pldx/Referee

## 6. 代码入口
Modules/Gimbal/Gimbal.hpp
