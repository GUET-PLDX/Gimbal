# Gimbal

## 1. 模块作用
两轴 Yaw/Pitch 云台控制模块。Yaw 使用已有 SMC/FTSMC，Pitch 使用角度外环、角速度内环 PID。

### 普通自瞄的论文运动学

仅在 `CMD_AUTO_CTRL`、AI 云台在线且任务为 `SET_VISION_AUTO_AIM` 时启用关节控制链：

```text
AHRS 末端四元数 + 两轴编码器角 -> 反解基座姿态
世界 yaw/pit 目标 -> 世界视轴 -> 基座视轴 -> 球面 IK -> 受限关节参考
关节参考 + 编码器位置/角速度 -> Yaw SMC / Pitch 串级 PID -> 电机力矩
```

`GimbalKinematics.hpp` 实现《二轴云台末端指向控制》V1.1 式 (17)、(19)、(21)、(23)、(27)、(29)，Yaw 最短路沿用 LibXR `CycleValue`。旋转采用 LibXR `EulerAngle<float>::ToRotationMatrixZYX()`，末端姿态直接使用 `Quaternion<float>`。PID、Topic、事件、电机在线判断继续复用现有接口。LibXR 通用关节链的迭代 IK 不等同于论文的球面闭式解，因此这里只补充该解和 Gimbal 所需的参考历史；当前 LibXR 没有通用低通滤波器。

采用右手坐标系、列向量，`R_BG = Rz(q_yaw) Ry(q_pitch)`，`R_WB = R_WG R_BG^T`；世界视轴为 `[cos(theta)cos(psi), cos(theta)sin(psi), -sin(theta)]`。应用命令 Pitch 在边界转换为 `theta = -cmd.pit`。目标 Roll 不参与视轴解算。Pitch 在关节坐标内裁剪；竖直目标保留上一参考 Yaw，首次取当前编码器关节 Yaw。

直驱轴在电机驱动完成 `reverse` 后，统一使用 `q=s*(CycleValue(encoder)-zero)`、`q_dot=s*feedback.omega`、`tau_motor=s*tau_joint`；`s` 为 `kinematics.yaw_direction/pitch_direction`，只能为 ±1。零点复用 `yaw_zero/pit_zero`。该接口面向直驱、Pitch 范围位于 ±90° 分支且不跨编码器相对零点半圈的机构；带减速器或多圈关节需先增加对应的位置展开/传动映射。

MC02 当前配置按 GM6020 直驱设定 `yaw_direction=+1`、`pitch_direction=-1`，Pitch 编码器范围 `[0.5,1.8]`、零点 `0.8` 映射为关节 `[-1.0,0.3] rad`。这些是软件配置，实际安装零位、轴向及 AHRS 到机构末端系的对齐仍需台架确认。

IK 关节位置参考先展开 Yaw，再差分生成速度；加速度由滤波后的速度差分生成。两者分别使用一阶低通，系数 `1-exp(-2*pi*f*dt)`，`kinematics.reference_cutoff_hz` 默认 30 Hz（单级时间常数约 5.3 ms），`dt` 取实际控制周期。首样本及模式切换后导数为零。测量速度始终为 CAN `omega`，单位 rad/s；视觉协议的世界角速度、角加速度字段保留原语义，普通自瞄新链不直接使用它们。采样保持或 IMU/CAN 分开更新会表现为 IK 参考的阶梯，低通抑制其导数脉动，30 Hz 带宽仍需实测整定。

Pitch 关节角误差采用普通差值，位置 PID 的 D 项也使用编码器关节速度。惯性前馈为 `J*(速度内环总目标-上次总目标)/dt`，其中已包含参考速度变化，首周期建立总目标历史；重力补偿按完整末端姿态投影到实际 Pitch 轴，复用 `pit_lc/pit_theta` 表示等效重力臂，在水平基座时还原原电机侧的 `-pit_lc*sin(application_pitch+pit_theta)`。

普通自瞄与大小符共享 `auto_aim` 参数，具有独立控制状态。两者互切会重置参考历史、PID 和 SMC，并用对应坐标系的反馈初始化；SMC 的 slew 锚点由最近实际下发力矩转换到关节方向。手动、巡逻、自动回退、大小符和系统辨识继续使用原路径。沿用现有输出限幅和模式行为。

### Yaw SMC/FTSMC 数学约定

`YawSmc` 基于参考工程的 SMC/FTSMC 结构实现，并使用 SI 单位和力矩接口：角度为 `rad`，角速度为 `rad/s`，角加速度为 `rad/s^2`，力矩为 `N*m`。SMC 本体直接消费调用方提供的角度、角速度和角加速度；普通自瞄由上述关节参考生成器提供同一条轨迹，旧路径使用原世界目标字段。

Yaw 使用 `CycleValue` 最短路（`e = θ - θd`，结果在 `[-π, π)`；float 半圈边界存在舍入），避免 `[0, 2π]` 目标与 `atan2` 反馈直接相减走出长弧：

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
1. ThreadFunc: 读取命令、欧拉角、陀螺和四元数；每轮依次更新反馈、解释参考并控制电机。
2. ParseCMD: 解析 CMD 输入并更新目标。
3. Control: 角度环与角速度环计算控制输出。
4. Update: 刷新电机反馈并发布状态。
5. SetMode / GetEvent: 模式管理与事件接口。

## 3. 接入步骤
1. 添加模块并绑定 motor_pit、motor_yaw、cmd。
2. 配置零位、限位、惯量和两套控制 profile。
3. 操作手、自动回退和巡逻使用 `manual`；自动控制下 AI 在线且选择自瞄/小符/大符任务时使用 `auto_aim`。Yaw 始终使用 SMC。`gimbal_cmd` 的 yaw/pit 是绝对角（操作手输入由 Gimbal 积分，自瞄为视觉轨迹）。
4. 两套 profile 分别整定 `yaw_smc`，力矩限幅与变化率配置在共用的 `yaw_output_limits` 中。

云台姿态输入 topic：
- `gimbal_cmd`：CMD 发布的云台控制命令。
- `gimbal_euler`：云台 IMU 融合后的欧拉角。
- `gimbal_gyro`：云台 IMU 原始角速度。
- `ahrs_quaternion`：末端到世界系的完整姿态，供普通自瞄反解与重力投影。

四个输入名称由 `Gimbal::Param` 的 `gimbal_cmd_topic_name`、`euler_topic_name`、
`gyro_topic_name`、`quaternion_topic_name` 配置，默认值如上。整个 `Param` 是一个完整映射：YAML 的 `param`
键须与结构体字段同名同序（xrobot 在生成期校验），四个 Topic 字段排在
`system_identify` 之后。Topic 名称字符串必须在模块运行期间保持有效，推荐使用
YAML 字符串或 C++ 字符串字面量；订阅名必须与发布者一致。构造期不校验电机指针与
Topic 名称，配置错误会在控制线程首次使用时暴露。

云台状态输出 topic：
- `yawmotor_angle` / `pitchmotor_angle`：相对原有零点的编码器最短角差，rad；不应用新增的关节方向配置。
- `yawmotor_omega` / `pitchmotor_omega`：原有 IMU 实测角速度，rad/s（yaw=`gyro.z`，pitch=`gyro.y` 已按云台约定取反）。每个控制周期无条件发布，普通自瞄内部编码器反馈不改变公共 Topic 的含义。


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
      kinematics:
        yaw_direction: 1.0
        pitch_direction: -1.0
        reference_cutoff_hz: 30.0
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
      quaternion_topic_name: "ahrs_quaternion"
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

模式仲裁、协议、超时与电机在线判断继续沿用现有接口。

控制任务在事件注册完成后创建。CMD 在输入提交时同步仲裁和发布；Gimbal 每 1 ms 读取命令，查询模式、AI 在线状态并解释参考。

## 主机与固件验证

在 MC02 工程根目录运行：

```bash
bash Modules/GUET-PLDX/Gimbal/tests/gimbal_kinematics_host_regression.sh
bash Modules/GUET-PLDX/Gimbal/tests/gimbal_control_host_regression.sh
bash Modules/GUET-PLDX/Gimbal/tests/yaw_smc_host_regression.sh
bash Modules/GUET-PLDX/Gimbal/tests/system_identify_host_regression.sh
bash Modules/GUET-PLDX/Gimbal/tests/motor_online_ownership_regression.sh
python3 Modules/GUET-PLDX/Gimbal/tests/gimbal_config_order_regression.py --generated User/xrobot_main.hpp
python3 Modules/GUET-PLDX/Gimbal/tests/gimbal_topic_contract_regression.py --config-root User/RobotConfig --configs sentry_gimbal
bash tools/Linux/buildgimbal.sh --skip-format
```

新增两组主机测试支持 `SANITIZE=1`。数学测试使用独立 AngleAxis 回代、可达域采样和势能梯度，覆盖倾斜基座、限位、极点、Yaw 跨界与参考导数。控制集成测试运行真实 Gimbal/CMD/Topic/事件/PID/SMC，仅替换 OS 调度、时钟与电机边界，通过实际力矩及公共 Topic 检查编码器/IMU 路由、模式互切、惯性单次前馈及重力投影。传感器和 CAN 的同步误差、滤波带宽、实机方向与增益性能仍需测量。
