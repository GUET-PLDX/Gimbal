# Yaw 控制器配置内嵌（Config Embedding）设计

## 目标

把 `Gimbal` 侧持有的 `yaw_lqr_eso_config_` / `yaw_smc_config_` 收进各自的控制器实例，
让控制器成为「配置 + 状态」的单一归属对象：

```
现状（4 成员）                          目标（2 成员）
YawLqrEso::Config yaw_lqr_eso_config_;   YawLqrEso yaw_lqr_eso_;   // 内含 const Config
YawLqrEso         yaw_lqr_eso_;          YawSmc    yaw_smc_;       // 内含 const Config
YawSmc::Config    yaw_smc_config_;
YawSmc            yaw_smc_;
```

顺带收益（本方案的主要价值）：**删除整套「运行期换配置」的边沿状态机**
（7 个 `previous_*_enable_` 标志，其中 3 个本来就是只写不读的死成员）。

## 依据：三个先例都指向同一形状

| 来源 | 位置 | 形状 |
| --- | --- | --- |
| 参考工程 `basic_framework` | `modules/motor/DJImotor/dji_motor.c:159-171` | `DJIMotorInit(config)` 内 `instance->motor_settings = config->controller_setting_init_config;` + `PIDInit(&instance->motor_controller.angle_PID, &config->...)`，配置被实例吞掉；`application/gimbal/gimbal.c` 只在 `GimbalInit()` 里留一个栈上临时 `Motor_Init_Config_s` |
| LibXR | `src/utils/pid.hpp:53-56` | `template <typename P> PID(P&& p) : param_(std::forward<P>(p)) { Reset(); }`，`Param param_;` 为私有首成员 |
| 本仓 Motor 域 | `Modules/RMMotor/RMMotor.hpp:149-152,334` | 构造收 `const Param&`，存 `Param param_; ///< 构造参数副本` |

参考工程**不合并控制器**：`controller_param_init_config` 聚合的是 config 结构体，
内部 `angle_PID` / `speed_PID` 仍是各自独立的 PID 实例。因此本方案只做「配置内嵌」，
不把 LQR-ESO 与 SMC 合成一个控制器。

## 范围

- `Modules/Gimbal/YawSmc.hpp`：`Config` 内嵌、构造锁存、删除 `previous_torque_slew_enable_`。
- `Modules/Gimbal/YawLqrEso.hpp`：`Config` 内嵌、构造锁存、删除 6 个 `previous_*_enable_`。
- `Modules/Gimbal/Gimbal.hpp`：成员 4 → 2，调用点去 config 形参，初始化期一致性校验。
- 受影响的 4 个主机测试与 3 个静态回归脚本。

## 不在范围内

- `Config` 字段、字段顺序、默认值**均不改动** —— manifest / YAML / xrobot 位置聚合契约不变。
  （2026-09-24 后续修正把 **C++ 成员默认值**改为与 manifest 一致，并去掉了 `Gimbal`
  构造函数的两个配置默认实参；字段名与字段顺序仍不动。见「已知缺口」。）
- `Config` → `Param` 重命名（与本目标正交，且会牵动 `gimbal_core_static_regression.sh:205`）。
- 把 `Scalar` 模板化、把 `yaw_manual_controller`/`yaw_ai_controller` 改成编译期策略。
- 运行时被控对象参数（`PARAM.j_yaw` / `PARAM.yaw_k` / `pid_yaw_omega_.OutLimit()`）的归属，
  维持现状：**按调用传入**，不进 `Config`（由 `tests/gimbal_param_ownership_regression.ps1` 守护）。
- `Gimbal::SetMode` 目前不调用控制器的 `Reset()`，本方案不改变这一行为。

## 接口设计

两个控制器采用同构形状（下例为 `YawSmc`，`YawLqrEso` 同理）：

```cpp
class YawSmc final {
 public:
  struct Config { /* 字段与顺序完全不变 */ };
  struct Reference { /* 不变 */ };
  struct Feedback { /* 不变 */ };
  struct Output { /* 不变 */ };

  static_assert(std::is_aggregate_v<Config>);
  static_assert(std::is_trivially_copyable_v<Config>);
  static_assert(std::is_standard_layout_v<Config>);

  /**
   * @brief 用锁存的配置构造控制器。
   * @note 不提供默认构造：控制器不能在没有显式配置的情况下存在。
   *       完美转发对齐 LibXR::PID 的 `template <typename P> PID(P&& p)`；
   *       requires 约束同时阻止本模板劫持拷贝构造。
   */
  template <typename ConfigType>
    requires std::same_as<std::remove_cvref_t<ConfigType>, Config>
  explicit YawSmc(ConfigType&& config)
      : config_(std::forward<ConfigType>(config)) {
    Reset(0.0f, 0.0f, 0.0f);
  }

  /// @brief 只读配置访问器（诊断 / 主机测试用）。
  [[nodiscard]] const Config& GetConfig() const noexcept { return config_; }

  [[nodiscard]] static bool ValidateConfig(const Config& config, float j_kg_m2);

  [[nodiscard]] Output Calculate(const Reference& reference,
                                 const Feedback& feedback, float dt_s,
                                 float j_kg_m2);

  void Reset(float theta_rad, float omega_rad_s,
             float previous_applied_torque_nm);
  void CommitAppliedTorque(float applied_torque_nm);

 private:
  const Config config_;  ///< 构造期锁存，运行期只读（LibXR::PID 的 param_ 是首成员）

  float last_applied_torque_nm_{};
  float slew_anchor_torque_nm_{};
  bool slew_primed_{};  ///< 首周期 slew anchor 初始化标志（取代 previous_torque_slew_enable_）
};
```

要点：

1. **`const Config config_`** —— 不可变性由类型系统保证，而不是靠约定。
   `LibXR::PID` 的 `param_` 非 const 且有 `SetK/SetP/...`；本方案**不提供任何 setter**，
   因为运行期没有任何路径修改这些配置。
2. **删除默认构造** —— `Config{}` 全零态会在 `ValidateConfig` 上被 SMC 拒绝、
   但被 LQR-ESO 接受（见「已知缺口」），因此不允许"无配置的控制器"存在。
3. **构造函数内调用 `Reset()`** —— 与 `LibXR::PID` 构造函数内 `Reset()` 一致，
   并由 `YawLqrEso::Reset()` 把 `observer_fresh_` 置 true，接管 ESO 首周期初始化。
4. **`Calculate` 去掉 `const Config&` 形参** —— 形参表从
   `(config, reference, feedback, dt, j, limit, b)` 缩为 `(reference, feedback, dt, j, limit, b)`。
5. **`ValidateConfig` 保持 `static` 且非 `constexpr`** —— C++20 的 `std::isfinite`
   不是 `constexpr`（C++23 才加入 `<cmath>`），加了编译不过；保持 `static` 以便
   主机测试与 `Gimbal` 初始化期直接调用。

## 复用的 LibXR 设施

`Modules/` 下的控制器当前只用到 `UNUSED` / `LibXR::CycleValue`。本方案按需扩用到：

| 设施 | 位置 | 用途 |
| --- | --- | --- |
| 完美转发构造范式 | `src/utils/pid.hpp:53` | `YawSmc` / `YawLqrEso` 构造签名 |
| `UNUSED` | `src/core/libxr_def.hpp:16` | 已有用法保留 |
| `REQUIRE(arg)` | `src/core/libxr_def.hpp:290` | `Gimbal` 初始化期配置校验（永不编译掉） |
| `[[nodiscard]] / noexcept / constexpr` 惯例 | LibXR 全仓 | `GetConfig()`、`ValidateConfig()`、`Calculate()` |
| concept 惯例（`MemberObjectPointer`、`CommonOrdered`） | `src/core/libxr_def.hpp:70-88` | 构造模板的 `requires` 约束风格依据 |
| `std::same_as` / `std::remove_cvref_t` / `<concepts>` | 同上 | 约束写法 |

`ASSERT(arg)` 在本项目**是空操作**：`cmake/` 与 `CMakeLists.txt` 均未定义
`LIBXR_DEBUG_BUILD`，所以初始化期校验必须用 `REQUIRE`。

⚠️ **`REQUIRE` 不得出现在 `YawSmc.hpp` / `YawLqrEso.hpp` 中**：`REQUIRE` 调用
`libxr_fatal_error`（`src/libxr.cpp:5`），而 `tests/yaw_*_host_regression.sh` 只做
头文件编译、不链接 LibXR 运行时，会出现 undefined reference。
校验放在 `Gimbal` 构造函数体内（`Gimbal.hpp` 不被任何主机测试包含）。

## 状态机收敛

`previous_*_enable_` 的全部读点都只在「`config.X_enable` 为真」时可达，
因此锁存 config 后它们退化为「是否首周期」的单一布尔量。

| 成员（现 7 个） | 现状语义 | 处理后 |
| --- | --- | --- |
| `previous_eso_comp_enable_` | **只写不读（死成员）** | 删除 |
| `previous_coulomb_enable_` | **只写不读（死成员）** | 删除 |
| `previous_lqi_enable_` | **只写不读（死成员）** | 删除 |
| `previous_eso_enable_` (LQR) | ESO 首次/重新使能时重置观测器 | 由既有 `observer_fresh_` 承担 |
| `previous_torque_bias_enable_` (LQR) | 首周期播种 `tau_meas_lpf_nm_` | `bias_primed_` |
| `previous_torque_slew_enable_` (LQR) | 首周期取 slew anchor；`CommitAppliedTorque` 是否跟随 | `slew_primed_` + `config_.torque_slew_enable` |
| `previous_torque_slew_enable_` (SMC) | 同上 | `slew_primed_` + `config_.torque_slew_enable` |

替换后的等价性（逐个读点核对）：

- `YawSmc.hpp:113, 230` 与 `YawLqrEso.hpp:327, 384`：
  `config.torque_slew_enable && !previous_torque_slew_enable_`
  → `config_.torque_slew_enable && !slew_primed_`；两处 `YawSmc.hpp:116, 233` /
  `YawLqrEso.hpp:387` 的 `previous_* = config.X_enable;` → `slew_primed_ = true;`。
- `YawSmc.hpp:175` / `YawLqrEso.hpp:327` 位于 `if (config.torque_slew_enable) {` 之内，
  故短路的 `config_.torque_slew_enable &&` 不改变可达性。
- `CommitAppliedTorque` 的 `if (previous_torque_slew_enable_)`
  → `if (slew_primed_ && config_.torque_slew_enable)`。
  生产侧 `Control` 内恒为「先 `Calculate` 后 `Commit`」，首周期 `slew_primed_` 已置位，
  与现状 `previous_torque_slew_enable_` 已为 true 一致；`Reset()` 之后一周期内也不会提前 `Commit`。
- `YawLqrEso.hpp:282` 的 `if (!previous_torque_bias_enable_)` → `if (!bias_primed_)`，
  所在分支同样被 `config.torque_bias_enable` 门控。
- `YawLqrEso.hpp:199` 的 `!previous_eso_enable_ || observer_fresh_` → `observer_fresh_`。
  前提是 `observer_fresh_` 在首周期为 true —— 由构造函数调用 `Reset(0,0,0)` 保证
  （`YawLqrEso.hpp:141` 已置 true）。
- 语义上「重新使能某功能」在锁存配置后不可能发生，因此不再需要边沿检测。

## 语义变化（需记录并接受）

1. `YawSmc` / `YawLqrEso` **不再是可拷贝赋值类型**（含 `const` 成员）。当前无赋值点。
2. **不再支持运行期更换配置**。两个依赖此能力的测试需改写（见实施计划阶段 1/2），
   改写后覆盖不减少：非法配置路径改为"用非法配置构造另一个实例"。
3. 默认构造消失 → 测试中约 32 处 `YawSmc controller;` / `YawLqrEso controller;`
   改为 `YawSmc controller{cfg};`（`yaw_lqr_eso_physics_test.cpp` 用的是 `PidYawAdapter`，不受影响）。
4. **初始化期校验会改变启动行为**：`REQUIRE` 失败时 `libxr_fatal_error` 打印
   `Fatal error at file:line` 后**永久挂起**（`src/libxr.cpp:5-28`），而不是像现在这样
   在运行期静默输出零力矩。这是在 `-fno-exceptions` 下唯一的 fail-fast 手段，
   属于**需要确认的行为变更**（配置错误从"云台无力矩地摔下去"变成"开机即挂起并打印行号"）。

## 已知缺口（2026-09-24 已修正）

原缺口：`YawLqrEso::Config` 的 C++ 默认值全为零/false，而 manifest 默认值是
`k_theta: 1.0` / `eso_enable: true` / `torque_slew_enable: true`；两者分叉，
且全零配置**能通过 `YawLqrEso::ValidateConfig`**（门槛都是 `<` 或 `<=`，零值不触发）
→ 静默的零增益控制器。`YawSmc` 因 `config.c <= 0.0f` 拒绝而同场景返回 `valid=false`。

修正由三部分组成（详见 `plans/` 的「后续修正」一节）：

1. 两个控制器的 `Config` 成员默认值改为**逐字段等于 manifest 建议基线**，
   并新增 Python 侧守护（`gimbal_config_order_regression.py` 用 float32 归一化比较
   C++ 默认值与 manifest 取值；`static_assert` 读不到 YAML 所以做不到）。
2. `YawLqrEso::ValidateConfig` 的 `k_theta` / `k_omega` 由 `< 0` 收紧为 `<= 0` 拒绝。
3. **执行中发现并一并修掉的更严重问题**：`xrobot_gen_main` 在 YAML 漏键时是
   **丢弃该实参**而不是回退到 manifest 默认值，导致其后实参整体左移
   （实测：省掉 `yaw_lqr_eso` 会把 `yaw_smc` 的聚合值喂给 `yaw_lqr_eso`）。
   由于第 1 步把默认值变成了"可用配置"，这个左移会从"上电校验失败"退化为
   "静默用错参数"。因此 `Gimbal` 构造函数的 `yaw_lqr_eso` / `yaw_smc`
   **去掉了默认实参**——最后一个形参没有默认值，漏键即**编译错误**。

## 验证方式

1. 主机算法测试：`bash Modules/Gimbal/tests/yaw_smc_host_regression.sh` 与
   `bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh`
   （均为 `-std=c++20 -Wall -Wextra -Werror -pedantic`，覆盖物理、仿真、单步）。
2. 静态回归：`tests/ai_yaw_integration_regression.sh`（CI 门禁）、
   `tests/selected_feature_removal_regression.py --header Gimbal.hpp`、
   `tests/gimbal_config_order_regression.py`、`tests/gimbal_param_ownership_regression.ps1`。
3. 固件编译：`pwsh tools/buildgimbal.ps1 --skip-format` 与 `tools/buildchassis.ps1 --skip-format`
   （CMD/Motor 为共享模块，两个配置都要编）。
4. 格式化：`pwsh tools/format_code.ps1 --check`（clang-format 21.1.8）。

## 交付顺序

本设计与 `plans/2026-09-24-yaw-controller-config-embedding.md` 配套，按该计划的阶段 1→5 执行；
阶段 1、2 各自独立可编译可测，阶段 3 才把 `Gimbal` 接线切过去。

## 实施状态（2026-09-24：已完成）

阶段 1–5 全部执行完毕。与本文档的两处偏差（已记入计划文档）：

1. **`REQUIRE` 改为在 `Gimbal` 构造函数的初始化列表内完成**（经
   `CheckedSmcConfig()` / `CheckedLqrEsoConfig()` 两个私有静态函数校验并透传配置），
   而不是构造函数体——`tests/gimbal_core_static_regression.sh` 严格邻接地钉住了
   构造函数体开头 `UNUSED(app); InitializeTopics(); ChassisMotionStateTopic();
   thread_.Create(`，插不进语句；这样也顺带保证校验早于控制线程启动。
2. **"语义变化 #2"有 1 处覆盖真的消失**：LQR/ESO 的
   `test_slew_falling_edge_restarts_from_latest_commit`（关闭 slew → 再打开的
   下降沿/再入沿）在配置锁存后不可表达，测试已删除；其余 8 个依赖"运行期换配置"
   的测试全部按配置拆分实例改写，断言值不变。
   "改写后覆盖不减少"这句话**不成立**，属本方案已知代价。
