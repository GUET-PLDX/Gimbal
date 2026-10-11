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

### Yaw 控制器配置归属（Config Embedding）

`YawSmc` 与 `YawLqrEso` 各自持有一份 `const Config config_`：配置在**构造期锁存、运行期只读**，`Calculate()` 不再接收 `Config` 形参。被控对象参数（`j_yaw` / `yaw_k` / 硬限幅 `pid_yaw_omega.OutLimit()`）仍按调用传入，**不进入 `Config`**。

```cpp
YawSmc yaw_smc{config};                       // 配置随实例一起构造
yaw_smc.Calculate(reference, feedback, dt_s, j_kg_m2);
```

升级时需要注意的语义变化：

1. 两个控制器**不再有默认构造**，也不再可拷贝/移动赋值（实例内含 `const` 成员）。`Gimbal` 的成员因此从 4 个（`*_config_` + 控制器）收敛为 2 个。
2. **不再支持运行期更换配置**。原先 7 个 `previous_*_enable_` 边沿标志已删除，其中 `eso_comp` / `coulomb` / `lqi` 三个本来就是只写不读的死成员。替代物是 `slew_primed_` / `bias_primed_` 两个「首周期播种」标志；ESO 的首周期初始化仍由既有的 `observer_fresh_` 承担（构造期 `Reset(0,0,0)` 会置位）。相应地，"运行期禁用再重新使能某功能"的场景不存在了，原先覆盖这些下降沿/再入沿的测试改为按配置拆分实例，断言值不变。
3. `Gimbal` 在构造期用 `REQUIRE(Yaw*::ValidateConfig(...))` 校验锁存配置，**校验失败会打印 `Fatal error at file:line` 并永久挂起**，不再像以前那样在运行期静默输出零力矩。校验放在初始化列表的 `CheckedSmcConfig()` / `CheckedLqrEsoConfig()` 内，保证早于控制线程启动；`REQUIRE` 只允许出现在 `Gimbal.hpp`（主机测试不链接 `libxr_fatal_error`）。
4. 这些改动**没有内存收益**（配置总得存在某处），收益是「配置 + 状态」单一归属，以及整体删除一套边沿状态机。

### 配置默认值的归属约定

同一份 `Config` 有两处「默认值」，角色不同、且**必须一致**：

| 位置 | 角色 | 谁守护 |
| --- | --- | --- |
| `Gimbal.hpp` manifest 里 `yaw_lqr_eso` / `yaw_smc` 块的取值 | **建议基线**（IDE/CubeMX 展示、生成 YAML 的初值）。**不是实车整定值** | `tests/gimbal_config_order_regression.py` 校验字段顺序 + 取值 |
| `YawSmc::Config` / `YawLqrEso::Config` 的成员默认值 | 必须**逐字段等于**上面的建议基线，因此 `Config{}` 是**可用**配置，不是全零哨兵 | 同一脚本逐字段比对（`static_assert` 读不到 YAML，故用 Python 守护；对浮点按 float32 归一化比较） |

配套两条硬约束：

1. **`Gimbal` 的 `yaw_lqr_eso` / `yaw_smc` 形参没有默认实参。** `xrobot_gen_main` 按位置展开 `constructor_args`，YAML 漏掉某个键时它会**丢弃该实参**而不是回退到 manifest 默认值，导致其后实参整体左移（实测：省掉 `yaw_lqr_eso` 会把 `yaw_smc` 的聚合值喂给 `yaw_lqr_eso`）。让最后一个形参没有默认值，"漏键"就从静默错配变成**编译错误**。
2. **`k_theta` / `k_omega` 必须严格为正**（`YawSmc` 侧对应 `c > 0`）。两者同时为 0 时角度环与角速度环一起失效、控制器静默输出零力矩——"配置被整块清零 / 聚合实参被截断"恰好是这个形状。这是上电校验（`REQUIRE`）能拦住的最后一道防线。

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
3. 手动和自瞄共用同一套 Yaw 算法实例。`yaw_manual_controller` 选 PID 或 SMC，`yaw_ai_controller` 选 SMC 或 LQR/ESO。`gimbal_cmd` 的 yaw/pit 已是绝对角（操作手积分在 CMD 完成，自瞄为视觉轨迹）。
4. 先验证模式切换，再联调控制参数。首次启用滑模时降低 `yaw_smc.torque_soft_limit_nm`，确认力矩极性后再抬升。

云台姿态输入 topic：
- `gimbal_cmd`：CMD 发布的云台控制命令。
- `gimbal_euler`：云台 IMU 融合后的欧拉角。
- `gimbal_gyro`：云台 IMU 原始角速度。
- `chassis_motion_state`：底盘运动状态（在线、模式、yaw 角速度），供小陀螺模式前馈使用。

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
      c: 20.0
      k: 120.0
      epsilon: 0.5
      q: 21.0
      p: 27.0
      error_deadband_rad: 0.0
      ftsmc_switch_rad: 0.017453292519943295
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
