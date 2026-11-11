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

### Yaw 控制器配置

`YawSmc` 在构造期锁存控制律参数与共享输出限幅；`Calculate()` 接收参考、反馈、`dt` 和共享的 `j_yaw`。`manual` 与 `auto_aim` 各有独立的 Pitch 双环 PID 和 Yaw SMC 控制律参数；力矩软硬限幅及变化率限制由 `yaw_output_limits` 共用。控制配置切换时，切入的控制器状态会重置，Yaw SMC 的变化率状态以最近一次下发力矩为起点。`tests/gimbal_config_order_regression.py` 校验配置字段顺序与默认值。

## 2. 主要函数说明
1. ThreadFunc: 云台控制主线程，也是 CMD 的唯一运行任务；每轮依次 UpdateInputs、读取本次命令、解释参考并控制电机。
2. ParseCMD: 解析 CMD 输入并更新目标。
3. Control: 角度环与角速度环计算控制输出。
4. Update: 刷新电机反馈并发布状态。
5. SetMode / GetEvent: 模式管理与事件接口。
6. DebugCommand: 调试命令入口（Debug 构建）。

## 3. 接入步骤
1. 添加模块并绑定 motor_pit、motor_yaw、cmd。
2. 配置零位、限位、惯量和两套控制 profile。
3. 操作手、自动回退和巡逻使用 `manual`；自动控制下 AI 在线且选择自瞄/小符/大符任务时使用 `auto_aim`。Yaw 始终使用 SMC。`gimbal_cmd` 的 yaw/pit 是绝对角（操作手输入由 Gimbal 积分，自瞄为视觉轨迹）。
4. 两套 profile 分别整定 `yaw_smc`，力矩限幅与变化率配置在共用的 `yaw_output_limits` 中。

云台姿态输入 topic：
- `gimbal_cmd`：CMD 发布的云台控制命令。
- `gimbal_euler`：云台 IMU 融合后的欧拉角。
- `gimbal_gyro`：云台 IMU 原始角速度。

前三个输入名称由 `Gimbal::Param` 的 `gimbal_cmd_topic_name`、`euler_topic_name`、
`gyro_topic_name` 配置，默认值如上。整个 `Param` 是一个完整映射：YAML 的 `param`
键须与结构体字段同名同序（xrobot 在生成期校验），三个 Topic 字段排在
`system_identify` 之后。Topic 名称字符串必须在模块运行期间保持有效，推荐使用
YAML 字符串或 C++ 字符串字面量；订阅名必须与发布者一致。构造期不校验电机指针与
Topic 名称，配置错误会在控制线程首次使用时暴露。

云台状态输出 topic：
- `yawmotor_angle` / `pitchmotor_angle`：电机绝对角，rad。
- `yawmotor_omega` / `pitchmotor_omega`：与内环同符号的 IMU 实测角速度，rad/s（yaw=`gyro.z`，pitch=`gyro.y` 已按云台约定取反）。每个控制周期无条件发布，不做陀螺新鲜度判定。


标准命令流程：
    xrobot_add_mod Gimbal --instance-id gimbal
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 4. 配置示例（YAML）
module: Gimbal
entry_header: Modules/Gimbal/Gimbal.hpp
args:
  - cmd: cmd
  - motor_pit: motor_pit
  - motor_yaw: motor_yaw
  - param:
      task_stack_depth: 2048
      manual:
        pid_pit_angle: {k: 0.0, p: 0.0, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 0.0, cycle: false}
        pid_pit_omega: {k: 0.0, p: 0.0, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 0.0, cycle: false}
        yaw_smc: {c: 20.0, k: 120.0, epsilon: 0.5, q: 21.0, p: 27.0, error_deadband_rad: 0.0, ftsmc_switch_rad: 0.017453292519943295, sat_boundary: 1.0, ftsmc_enable: true}
      auto_aim:
        pid_pit_angle: {k: 0.0, p: 0.0, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 0.0, cycle: false}
        pid_pit_omega: {k: 0.0, p: 0.0, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 0.0, cycle: false}
        yaw_smc: {c: 20.0, k: 120.0, epsilon: 0.5, q: 21.0, p: 27.0, error_deadband_rad: 0.0, ftsmc_switch_rad: 0.017453292519943295, sat_boundary: 1.0, ftsmc_enable: true}
      yaw_output_limits:
        torque_soft_limit_nm: 2.0
        torque_min_nm: -2.223
        torque_max_nm: 2.223
        torque_slew_rate_nm_s: 1000.0
        torque_slew_enable: true
      pit_max_angle: 0.0
      pit_min_angle: 0.0
      pit_lc: 0.0
      pit_theta: 0.0
      j_pit: 0.0
      j_yaw: 0.0
      pit_zero: 0.0
      yaw_zero: 0.0
      patrol_pitch_amplitude_rad: 0.0
      patrol_pitch_angular_rate_rad_s: 0.0
      patrol_yaw_rate_rad_s: 0.0
      reverse_flag: false
      thread_priority: LibXR::Thread::Priority::MEDIUM
      system_identify:
        enabled: false
        axis_select: IdentifyAxis::BOTH
        arming_delay_s: 3.0
        step_torque_yaw_nm: 0.5
        step_torque_pit_nm: 0.2
        step_half_period_s: 0.5
        step_cycles: 10
        settle_time_s: 1.0
        lpf_cutoff_hz: 50.0
        alpha_gate_rad_s2: 1.0
        rls_delta: 1000.0
        rls_lambda: 1.0
        coulomb_tanh_scale: 0.1
        omega_abort_yaw: 8.0
        omega_abort_pit: 4.0
        pit_angle_margin_rad: 0.1
        axis_timeout_s: 30.0
      euler_topic_name: "gimbal_euler"
      gyro_topic_name: "gimbal_gyro"
      gimbal_cmd_topic_name: "gimbal_cmd"
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

## LibXR 2.0 静态装配

使用 xrobot 1.0.1 的 `module` / 有序 `args`；构造函数显式注入外设与服务，实例按依赖顺序创建。参数名和顺序以头文件中的构造函数为准，固件示例见 `User/RobotConfig/`。

保留 PLDX 控制、协议、超时与离线保护。 `OnMonitor()` 由生成的监控循环直接调用。

控制任务在事件注册完成后创建，并等待启动信号。首次 `OnMonitor()` 在静态模块构造和 EventBinder 绑定完成后释放启动信号。Gimbal 调用 `CMD::UseExternalRunner()`，因此全局监控循环不会重复消费 CMD 输入。任务周期仍为 1 ms；查询 CMD 模式和 AI 在线状态与参考解释在同一任务完成。
