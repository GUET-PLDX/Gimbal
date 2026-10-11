# Yaw 控制器配置内嵌 实施计划

> **执行方式：** 按阶段顺序执行，每个阶段结束都要能编译 + 跑通主机测试再进入下一阶段。
> 设计与依据见 `../specs/2026-09-24-yaw-controller-config-embedding-design.md`。

**目标：** 把 `yaw_lqr_eso_config_` / `yaw_smc_config_` 内嵌进控制器实例，
`Gimbal` 成员从 4 个收敛为 2 个，并删除 7 个 `previous_*_enable_` 边沿标志。

**架构：** 控制器构造期锁存 `const Config config_`，`Calculate()` 不再接收 config 形参；
被控对象参数（`j_yaw` / `yaw_k` / 力矩上限）继续按调用传入。

**技术栈：** C++20 固件、C++20 主机测试（`-Wall -Wextra -Werror -pedantic`）、
LibXR（`libxr_def.hpp` 的 `REQUIRE` / concept 惯例、`pid.hpp` 的完美转发构造范式）、
Python/Bash 静态回归、CMake/STM32 交叉编译。

## 全局约束

- `Config` 的**字段名、字段顺序、默认值一律不改**——manifest / YAML / xrobot
  位置聚合契约不动，`gimbal_config_order_regression.py` 的 `EXPECTED_FIELDS` 等常量不更新。
- 不改控制律的数值公式、更新顺序、模式切换、无效输出回落、力矩提交时机。
- 不把 `j_yaw` / `yaw_k` / 力矩上限搬进 `Config`（`gimbal_param_ownership_regression.ps1` 守护）。
- **`REQUIRE` 只能出现在 `Gimbal.hpp`**，不得进入 `YawSmc.hpp` / `YawLqrEso.hpp`
  （主机测试不链接 `libxr_fatal_error`，否则 undefined reference）。
- 不新增 `SetConfig` / setter 之类的运行期配置写入接口。

---

## 阶段 0：记录基线

**Files:** 无改动

- [x] **前置：确认有主机 C++ 编译器。** 两个 host regression 脚本都依赖 `${CXX:-c++}`
  （`-std=c++20 -Wall -Wextra -Werror -pedantic -ffp-contract=off
  -DLIBXR_DEFAULT_SCALAR=float`）。本机 bash PATH 里没有 `c++`/`g++`/`clang++`，
  但 msys64 里有可用的 **GCC 16.1.0**，用法（两个条件都必须满足）：

  ```bash
  export PATH="/c/msys64/ucrt64/bin:$PATH"   # 必须：cc1plus.exe 依赖 ucrt64 的 DLL
  CXX=/c/msys64/ucrt64/bin/g++.exe bash Modules/Gimbal/tests/yaw_smc_host_regression.sh
  ```

  ⚠️ 只设 `CXX` 不设 `PATH` 会得到**静默的 exit 1**（`cc1plus.exe` 起不来、无任何诊断），
  极易被误判成脚本本身的问题。
  ⚠️ 本机 msys64 未装 `libasan`/`libubsan`，`SANITIZE=1` 变体链接失败，只能跑普通变体。
- [x] 记录当前主机测试与静态回归的通过情况，作为"是否引入新失败"的对照：

  ```bash
  bash Modules/Gimbal/tests/yaw_smc_host_regression.sh
  bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
  bash Modules/Gimbal/tests/ai_yaw_integration_regression.sh
  ```

  预期：前两条**当前就是失败的**（存量失同步，见 `.workbuddy/memory/2026-09-24.md`）——
  `tests/yaw_smc_test.cpp` 仍在写 `cfg.j_kg_m2` 且 `calculate_once` 只传 4 个实参，
  而 `YawSmc::Calculate` 现在要 5 个。阶段 1 会顺带修好它。
  `ai_yaw_integration_regression.sh` 会先调 `gimbal_core_static_regression.sh`，
  同样被存量问题拦住。**先把失败清单抄下来**，后面只比"清单是否变短"。

- [x] 确认工作区没有与本方案无关的未提交改动混在 `Modules/Gimbal/` 下。

---

## 阶段 1：`YawSmc` 内嵌 Config ✅ 已完成（2026-09-24，主机回归绿）

**Files:**
- Modify: `Modules/Gimbal/YawSmc.hpp`
- Modify: `Modules/Gimbal/tests/yaw_smc_test.cpp`
- Modify: `Modules/Gimbal/tests/yaw_smc_test_support.hpp`（原计划漏列，见 1.2 注）

### 1.1 头文件

- [x] 在 `struct Config` 之后加编译期契约（保证 xrobot 的位置聚合初始化成立）：

  ```cpp
  static_assert(std::is_aggregate_v<Config>);
  static_assert(std::is_trivially_copyable_v<Config>);
  static_assert(std::is_standard_layout_v<Config>);
  ```

  需要 `#include <concepts>` / `<type_traits>`（`libxr_def.hpp` 已包含 `<concepts>`
  与 `<type_traits>`，按 include 顺序规则显式补充使用到的头）。

- [x] 删除默认构造（`YawSmc` 原本无显式构造，隐式默认构造随之消失），新增：

  ```cpp
  template <typename ConfigType>
    requires std::same_as<std::remove_cvref_t<ConfigType>, Config>
  explicit YawSmc(ConfigType&& config)
      : config_(std::forward<ConfigType>(config)) {
    Reset(0.0f, 0.0f, 0.0f);
  }
  ```

- [x] 新增只读访问器：

  ```cpp
  [[nodiscard]] const Config& GetConfig() const noexcept { return config_; }
  ```

- [x] `Calculate` 签名去掉 config 形参：

  ```cpp
  [[nodiscard]] Output Calculate(const Reference& reference,
                                 const Feedback& feedback, float dt_s,
                                 float j_kg_m2);
  ```

- [x] `ValidateConfig` 保持 `static bool ValidateConfig(const Config&, float j_kg_m2)`
  （**不加 `constexpr`**，C++20 的 `std::isfinite` 不是 `constexpr`）。

- [x] `Calculate` 内 `config.X` → `config_.X`；`Reset` 的 `previous_torque_slew_enable_ = false;`
  → `slew_primed_ = false;`。

- [x] 边沿标志替换（逐点，语义等价见设计文档）：

  | 现位置（按符号） | 现在 | 改为 |
  | --- | --- | --- |
  | 死区提前返回分支 | `if (config.torque_slew_enable && !previous_torque_slew_enable_) { slew_anchor_torque_nm_ = last_applied_torque_nm_; }` | 同形，`config_` + `!slew_primed_` |
  | 死区提前返回分支 | `previous_torque_slew_enable_ = config.torque_slew_enable;` | `slew_primed_ = true;` |
  | slew 主分支 | `if (!previous_torque_slew_enable_) { ... }` | `if (!slew_primed_) { ... }` |
  | 函数尾部 | `if (config.torque_slew_enable && !previous_torque_slew_enable_) { slew_anchor_torque_nm_ = last_applied_torque_nm_; }` | 同形，`config_` + `!slew_primed_` |
  | 函数尾部 | `previous_torque_slew_enable_ = config.torque_slew_enable;` | `slew_primed_ = true;` |
  | `CommitAppliedTorque` | `if (previous_torque_slew_enable_)` | `if (slew_primed_ && config_.torque_slew_enable)` |

- [x] 私有区：`const Config config_;` 作为**首成员**（对齐 `LibXR::PID::param_`），
  删除 `bool previous_torque_slew_enable_{};`，新增 `bool slew_primed_{};`。

### 1.2 主机测试

> **注（执行后发现，原计划漏列）：** 需要改的是**两个**测试文件，不只是 `yaw_smc_test.cpp`。
> `tests/yaw_smc_test_support.hpp` 里有 3 处依赖旧形态：`base_yaw_smc_config()` 的
> `.j_kg_m2 = 0.03f`（字段已不在 `Config` 中）、`smc_code_oracle()` 用 `config.j_kg_m2`
> 参与运算、以及 `ValidateConfig(cfg)` 少一个实参。基线编译失败 19 条错误里大部分来自它。

- [x] `yaw_smc_test_support.hpp`：抽出 `inline constexpr float BASE_YAW_SMC_J_KG_M2 = 0.03f;`，
  `base_yaw_smc_config()` 删掉 `.j_kg_m2`，`smc_code_oracle(config, reference, feedback, j_kg_m2)`
  把惯量改为**显式末位形参**（不给默认值，避免"被控对象参数"在测试里被隐式补全）。
- [x] `calculate_once` 去掉 config 形参，内部固定用 `BASE_YAW_SMC_J_KG_M2`；
  所有 `calculate_once(controller, cfg, ...)` → `calculate_once(controller, ...)`，
  `controller.Calculate(config, ...)` → `controller.Calculate(...)`。
- [x] 14 处 `YawSmc controller;` + `Reset(0,0,0)` → `YawSmc controller{cfg};`
  （构造函数已做同样的 `Reset(0,0,0)`，重复的零值 `Reset` 删除；非零 `Reset` 保留）。
- [x] `test_config_validation()` 改用 `YawSmc::ValidateConfig(cfg, BASE_YAW_SMC_J_KG_M2)`，
  并把原 `cfg.j_kg_m2 = 0.0f` → `CHECK(!YawSmc::ValidateConfig(cfg, 0.0f));`。
- [x] `test_slew_reentry_uses_latest_applied_torque` 改写为等价形式：
  用「带 slew 的配置构造实例 + `Reset(0.0f, 0.0f, -0.5f)`」验证首周期 anchor 取自
  `last_applied_torque_nm_`，期望 `tau_cmd_nm == -0.3f`（与原断言一致）。
  原「运行期重新使能」场景在锁存配置下不可表达，注释说明。
- [x] `test_invalid_inputs_are_rejected` 改写：非法配置改为
  `YawSmc invalid_controller{invalid_cfg};` 上调用 `Calculate` 断言 `!output.valid`；
  其余非法输入（NaN 参考、`valid=false` 反馈、`dt` 越界）保持在合法实例上。
- [x] **原计划漏列的另外两处运行期改配置**（都是"同一实例喂不同 config"）：
  - `test_wrap_and_complete_deadband`：`cfg.error_deadband_rad` 被改 3 次
    （0.05 → 0.04 → 0.0）→ 改为按需要构造 `boundary_controller` / `zero_deadband_controller`。
  - `test_soft_and_hard_limit_order`：中途改 `torque_soft_limit_nm` / `torque_max_nm`
    → 改为 `soft_limited_controller` + `hard_limited_controller` 两个实例。
  两者 `torque_slew_enable` 均为 false，实例状态本来就是惰性的，故拆分后逐点等价。
- [x] 在 `yaw_smc_test.cpp` 顶部加接口契约 `static_assert`（钉住本次引入的形状）：
  `!is_default_constructible_v` / `is_copy_constructible_v` / `!is_copy_assignable_v` /
  `!is_move_assignable_v` / `GetConfig()` 返回 `const Config&`。

### 1.3 验证

- [x] `bash Modules/Gimbal/tests/yaw_smc_host_regression.sh` 通过。
- [x] `bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh` 仍应与阶段 0 基线一致
  （本阶段不动 LQR-ESO）。

---

## 阶段 2：`YawLqrEso` 内嵌 Config ✅ 已完成（主机回归绿）

> **执行后修正（原计划低估了工作量）：**
> - **文件清单漏了两个。** `tests/yaw_lqr_eso_test_support.hpp` 的 `base_yaw_config()` 里有
>   `.b_nms_rad = 0.0f`（已不是 `Config` 字段，是基线唯一那条编译错误的来源），
>   且全部 `Calculate` / `ValidateConfig` 调用都少传 `b_nms_rad` 实参（另一处失同步）。
>   已抽出 `TEST_YAW_B_NMS_RAD` 常量并补全全部调用点。
> - **`yaw_lqr_eso_physics_test.cpp` 实际不用改**：它只走 `PidYawAdapter`，不碰 `YawLqrEso`。
> - **不是 1 个测试依赖"运行期换配置"，而是 9 个**（原计划只点了
>   `test_invalid_inputs_are_rejected`）：`test_soft_and_hard_limit_order`、
>   `test_slew_reentry_*`、`test_coulomb_feedforward_switch`、
>   `test_lqi_integral_limit_and_falling_edge`、
>   `test_bias_measurement_validation_and_lpf_initialization`、
>   `test_bias_limit_and_falling_edge_reset`、
>   `test_slew_falling_edge_restarts_from_latest_commit`、
>   `test_observer_only_isolation_and_falling_edge`、
>   `test_eso_compensation_limit_gates_and_switch`。
>   处理方式统一为**按配置拆分实例**（这些测试的 `torque_slew_enable` 多为 false，
>   实例状态本就惰性，拆分逐点等价），断言值全部保留。
> - **有 1 处覆盖是真的消失了**：`test_slew_falling_edge_restarts_from_latest_commit`
>   验证的是"关闭 slew → 再打开 slew，anchor 重新取最近一次已提交力矩"。配置锁存后
>   该下降沿/再入沿不可能发生，测试已删除，其残留不变量由
>   `test_slew_reentry_uses_latest_applied_torque` 与
>   `test_slew_uses_only_committed_torque` 覆盖（测试文件内留了说明注释）。
>   这与设计文档「语义上重新使能不可能发生，因此不再需要边沿检测」一致，
>   但确实**不是"覆盖不减"**，需要按设计意图接受。
> - 仿真适配器不能只写 `controller_{config_}`（构造期锁存，构造后再改 `config_` 无效），
>   改为 `controller_config(kind)` 静态函数在构造前完成增益覆盖，
>   `Config()` 改为转发 `controller_.GetConfig()`。

**Files:**
- Modify: `Modules/Gimbal/YawLqrEso.hpp`
- Modify: `Modules/Gimbal/tests/yaw_lqr_eso_test.cpp`
- Modify: `Modules/Gimbal/tests/yaw_lqr_eso_simulation_test.cpp`
- Modify: `Modules/Gimbal/tests/yaw_lqr_eso_test_support.hpp`（如需要）

- [x] 与阶段 1.1 同构：加 `static_assert` 三连、完美转发构造 + `Reset(0.0f,0.0f,0.0f)`、
  `GetConfig()`、`Calculate` 去掉 config 形参、`ValidateConfig` 保持 static。
- [x] `Reset()` 中删除 6 行 `previous_*_enable_ = false;`，改为 `bias_primed_ = false; slew_primed_ = false;`
  （`observer_fresh_ = true;` 保留不动，它现在承担 ESO 首周期初始化）。
- [x] 边沿标志替换：

  | 现符号位置 | 现在 | 改为 |
  | --- | --- | --- |
  | ESO 首个分支之后 | `else if (!previous_eso_enable_ \|\| observer_fresh_)` | `else if (observer_fresh_)` |
  | tau-bias 分支 | `if (!previous_torque_bias_enable_)` | `if (!bias_primed_)` |
  | slew 主分支 | `if (!previous_torque_slew_enable_)` | `if (!slew_primed_)` |
  | 函数尾部 | `if (config.torque_slew_enable && !previous_torque_slew_enable_)` | `config_` + `!slew_primed_` |
  | 函数尾部 6 行赋值 | `previous_eso_enable_ = ...; ... previous_torque_slew_enable_ = ...;` | 只留 `bias_primed_ = true; slew_primed_ = true;` |
  | `CommitAppliedTorque` | `if (previous_torque_slew_enable_)` | `if (slew_primed_ && config_.torque_slew_enable)` |

- [x] 私有区：`const Config config_;` 置首；删除 6 个 `bool previous_*_enable_{};`，
  新增 `bool bias_primed_{};` 与 `bool slew_primed_{};`。
- [x] 测试：约 16 处 `YawLqrEso controller;` → `YawLqrEso controller{cfg};`；
  `yaw_lqr_eso_simulation_test.cpp` 的适配器改为持有 `YawLqrEso controller_{config_};`
  并把 `controller_.Calculate(config_, ...)` 的 config 实参去掉。
- [x] `test_invalid_inputs_are_rejected`（`yaw_lqr_eso_test.cpp`）改写为
  `YawLqrEso invalid_controller{invalid_cfg};` 上断言 `!output.valid`。
- [x] `bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh` 通过
  （含 `yaw_lqr_eso_physics_test.cpp`、`yaw_lqr_eso_simulation_test.cpp`）。

---

## 阶段 3：`Gimbal` 接线与初始化期校验 ✅ 已完成（两个固件配置均编译通过）

> **执行后修正：`REQUIRE` 不能放在构造函数体内。**
> `tests/gimbal_core_static_regression.sh` 用
> `(?s)Gimbal\(.*?\)\s*:.*?\{\s*UNUSED\(app\);\s*InitializeTopics\(\);\s*ChassisMotionStateTopic\(\);\s*thread_\.Create\(`
> **严格邻接地**钉住构造函数体的开头，任何插入（包括注释）都会打破它；而校验又必须早于
> 控制线程启动。最终改为在**初始化列表**里经两个私有静态函数完成校验并透传配置：
>
> ```cpp
> yaw_lqr_eso_(CheckedLqrEsoConfig(std::move(yaw_lqr_eso), gimbal_param.j_yaw,
>                                  pid_yaw_omega_.OutLimit(), gimbal_param.yaw_k)),
> yaw_smc_(CheckedSmcConfig(std::move(yaw_smc), gimbal_param.j_yaw)) {
> ```
>
> 两个 helper 内部才调用 `REQUIRE(...)`，并就地返回校验过的配置（`std::move` +
> 返回 prvalue，零额外拷贝）。效果比原计划更好：校验发生在**任何成员完成构造之后、
> 线程创建之前**。

**Files:**
- Modify: `Modules/Gimbal/Gimbal.hpp`

- [x] 成员区（现 4 行）收敛为 2 行，并保持 `PARAM` 之后的顺序：

  ```cpp
  YawLqrEso yaw_lqr_eso_;
  YawSmc yaw_smc_;
  ```

- [x] 构造函数初始化列表把配置 **move** 进控制器：

  ```cpp
  yaw_lqr_eso_(std::move(yaw_lqr_eso)),
  yaw_smc_(std::move(yaw_smc))
  ```

  `requires` 约束保证具名形参（左值）只能按 `const Config&` 拷贝路径匹配；
  显式 `std::move` 避免一次多余拷贝。

- [x] 调用点去掉 config 形参：

  ```cpp
  yaw_lqr_eso_.Calculate(
      {.theta_rad = cmd_data_.yaw, ...},
      {.theta_rad = euler_.Yaw(), ...},
      dt_, PARAM.j_yaw, pid_yaw_omega_.OutLimit(), PARAM.yaw_k);
  ```

  ```cpp
  yaw_smc_.Calculate({.theta_rad = theta_ref, ...}, {...}, dt_, PARAM.j_yaw);
  ```

  ⚠️ `tests/ai_yaw_integration_regression.sh:151` 的正则钉住
  `Calculate(...dt_, PARAM.j_yaw, pid_yaw_omega_.OutLimit(), PARAM.yaw_k)`，
  上述形状**仍然匹配**，不必改；`:179` 钉住 `yaw_smc_.Calculate(yaw_smc_config_, {...})`
  必须同步（阶段 4）。

- [x] 构造函数体内加一次性一致性校验（`REQUIRE`，永不编译掉）：

  ```cpp
  REQUIRE(YawLqrEso::ValidateConfig(yaw_lqr_eso_.GetConfig(), PARAM.j_yaw,
                                    pid_yaw_omega_.OutLimit(), PARAM.yaw_k));
  REQUIRE(YawSmc::ValidateConfig(yaw_smc_.GetConfig(), PARAM.j_yaw));
  ```

  位置：在 `thread_.Create(...)` **之前**，与其它构造期约束放在一起。

- [x] 验证：`pwsh tools/buildgimbal.ps1 --skip-format` 与
  `pwsh tools/buildchassis.ps1 --skip-format` 均 `Done.`。

---

## 阶段 4：回归脚本同步 ✅ 已完成

**Files:**
- Modify: `Modules/Gimbal/tests/ai_yaw_integration_regression.sh`
- Modify: `Modules/Gimbal/tests/selected_feature_removal_regression.py`
- Modify: `Modules/Gimbal/tests/gimbal_core_static_regression.sh`（如涉及）

- [x] `ai_yaw_integration_regression.sh`：
  - `:179` 的 `const auto YAW_SMC_OUTPUT =\s*yaw_smc_\.Calculate\(yaw_smc_config_,\s*\{...`
    → 去掉 `yaw_smc_config_,\s*`。
  - `:151` 保持不变。
- [x] `selected_feature_removal_regression.py`：
  - `required` 中 `"direct AI config": r"yaw_lqr_eso_\.Calculate\(\s*yaw_lqr_eso_config_"`、
    `"direct SMC config": r"yaw_smc_\.Calculate\(\s*yaw_smc_config_"` 改为匹配
    `yaw_lqr_eso_\.Calculate\(\s*\{` / `yaw_smc_\.Calculate\(\s*\{`（仍钉住"直接构造参考、无中间快照"）。
  - `forbidden` 增补：
    `"config edge flag": "previous_eso_enable_|previous_torque_bias_enable_|previous_torque_slew_enable_|previous_eso_comp_enable_|previous_coulomb_enable_|previous_lqi_enable_"`。
  - `required` 增补 `"latched config": r"const Config config_;"`。
- [x] `gimbal_core_static_regression.sh`：`:205` 钉的是 `Gimbal` 构造函数签名，
  本方案不改该签名，预期无需改动；若因格式换行导致失配，按实际重新钉。
  ⚠️ 该脚本在工作区里已有 9 条存量失配（与本次无关），
  **不要**在本阶段顺手改它们，否则无法区分新旧失败。
- [x] `gimbal_param_ownership_regression.ps1`、`gimbal_config_order_regression.py`
  预期**不需要改**——用于反向验证（跑通即说明被控对象参数归属与配置契约没被破坏）。

> **执行后补充（两处计划外改动）：**
>
> 1. **`gimbal_core_static_regression.sh` 必须改一条**（原计划说"预期无需改动"）。
>    断言 `forbid 'float torque_|this->torque_'` 的本意是禁止"重复的 Pitch 重力前馈
>    调试缓存成员"，但模式没有右边界，`CheckedLqrEsoConfig()` 里合法的形参
>    `float torque_limit_nm` 也会命中。已收紧为
>    `forbid 'float torque_[A-Za-z0-9_]*[;=]|this->torque_'`（要求成员声明终止符），
>    既恢复绿，又保留原意。
> 2. **`ai_yaw_integration_regression.sh` 增补 4 条新断言**（该脚本是 CI 门禁）：
>    新增 `need_file()` 辅助函数，钉住
>    `const Config config_;`（两个控制器头）与
>    `requires std::same_as<std::remove_cvref_t<ConfigType>, Config>`（两个构造函数），
>    并 `forbid_file` 掉控制器头里的 `previous_*_enable_` 边沿标志。
>    计划里写的"在 `selected_feature_removal_regression.py` 里加
>    `required "latched config": r"const Config config_;"`"**是错的**：
>    该脚本只读 `Gimbal.hpp`，而 `config_` 在控制器头里；已改到 ai_yaw 脚本。

---

## 阶段 5：整体验证 ✅ 已完成（除下述既有失败外全绿）

- [x] 主机算法测试：

  ```bash
  bash Modules/Gimbal/tests/yaw_smc_host_regression.sh
  bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
  ```

- [x] 静态回归：

  ```bash
  bash Modules/Gimbal/tests/ai_yaw_integration_regression.sh
  python3 Modules/Gimbal/tests/selected_feature_removal_regression.py --header Modules/Gimbal/Gimbal.hpp
  python3 Modules/Gimbal/tests/gimbal_config_order_regression.py --header Modules/Gimbal/Gimbal.hpp
  python3 Modules/Gimbal/tests/parse_cmd_behavior_regression.py --header Modules/Gimbal/Gimbal.hpp
  ```

  `gimbal_param_ownership_regression.ps1` 是 PowerShell 脚本，单独走 PowerShell 工具：

  ```
  pwsh -File Modules/Gimbal/tests/gimbal_param_ownership_regression.ps1
  ```

  注：`gimbal_config_order_regression.py` 的路径默认值都是基于 `__file__` 推导的绝对路径，
  从仓库根调用即可；`--header-only` 会跳过 YAML 那一半校验，别加。

  `parse_cmd_behavior_regression.py` 是另一条 CI 门禁。它与本方案无关，
  但在当前工作区已因「`ParseCMD` 整体重构尚未同步测试」而失败，
  因此只需确认其失败清单**与阶段 0 基线一致**，不需要在本方案里修。

- [x] 固件编译（两个配置都要，CMD/Motor 是共享模块）：

  ```
  pwsh tools/buildgimbal.ps1 --skip-format
  pwsh tools/buildchassis.ps1 --skip-format
  ```

- [x] 格式化检查（必须走 PowerShell 工具，bash 里调 pwsh 会被安全策略拦）：

  ```
  pwsh tools/format_code.ps1 --check
  ```

- [x] 与阶段 0 基线对比失败清单：**只允许变短，不允许新增**。
- [x] 把「语义变化」里的 4 条写进 `Modules/Gimbal/README.md` 或提交信息。

---

## 可选后续（本计划**未执行**）

- [ ] `ValidateConfig` 返回 `LibXR::ErrorCode`（对齐 `Motor::Update()` 惯例），
  让"为什么无效"可诊断，而不是只给一个 `bool`。
- [ ] 修 `Modules/Gimbal/README.md` §4 配置示例：该块是早期 manifest 形态
  （`roll_*` 命名、参数平铺而非 `gimbal_param:` 聚合），已整体过时，
  建议直接由 manifest 重新生成。

---

## 后续修正：配置默认值与零增益缺口（2026-09-24 已完成）

> 对应上面「可选后续」的第 1、3 条。**这里有意放宽了「全局约束」中
> "`Config` 默认值一律不改"这一条**：字段名与字段顺序仍然不动，
> 只把 C++ 成员默认值改成与 manifest 一致（manifest 本身未改）。

**问题（设计文档「已知缺口」）：** `YawLqrEso::Config` 的 C++ 默认值全为零，
manifest 默认值是正增益；且全零配置**能通过 `YawLqrEso::ValidateConfig`**
（门槛都是 `<` / `<=`，零值不触发）→ 静默零增益控制器。

**执行中发现更严重的一段：** `xrobot_gen_main` 按位置展开 `constructor_args`，
YAML 漏掉某个键时它**丢弃该实参**而不是回退到 manifest 默认值（实测对比
`build/sentry_gimbal/generated/xrobot_main.hpp` 与一份删掉 `yaw_lqr_eso` 的配置：
后者生成的 `Gimbal` 构造调用少一个实参，`yaw_smc` 的聚合值被喂给了 `yaw_lqr_eso`）。
一旦把 C++ 默认值改成"可用配置"，这个左移就会从"上电校验失败"退化为
"静默用错参数"——所以第 1 条必须与第 3 条一起做。

**改动：**

1. `YawSmc::Config` / `YawLqrEso::Config` 的成员默认值 → **逐字段等于** manifest 建议基线。
2. `YawLqrEso::ValidateConfig`：`k_theta` / `k_omega` 由 `< 0.0f` 收紧为 `<= 0.0f` 拒绝
   （`YawSmc` 侧 `c <= 0` 本来就拒绝，无需改）。
3. `Gimbal` 构造函数的 `yaw_lqr_eso` / `yaw_smc` **去掉默认实参** → 漏键变编译错误。
4. 新增守护：`gimbal_config_order_regression.py` 增加 `config_defaults()` /
   `check_cpp_defaults()`，逐字段比对 C++ 默认值与 manifest 取值
   （浮点按 float32 归一化，`static_assert` 读不到 YAML 所以只能放在 Python 侧）。
5. 新增不变量测试：`YawLqrEso` 的 `k_theta == 0` / `k_omega == 0` / 两者同时为 0
   必须被拒绝；两个控制器的 `Config{}` 必须是**可用**配置。
6. `Modules/Gimbal/README.md`：把「已知缺口」一节改写成「配置默认值的归属约定」。

**验证：**

| 检查项 | 结果 |
| --- | --- |
| 新守护的负向测试（把 C++ `c{20.0f}` 改成 `19.0f`、`eso_enable{true}` 改成 `false`） | 分别报 `SMC/LQR C++ Config default mismatch: <key>`，退出码 1 ✓ |
| 漏键编译验证（用删掉 `yaw_lqr_eso` 的 YAML 构建） | `error: no matching constructor for initialization of 'Gimbal'`（原先可编译）✓ |
| `yaw_smc_host_regression.sh` / `yaw_lqr_eso_host_regression.sh` | 通过 ✓ |
| `gimbal_config_order_regression.py` / `selected_feature_removal_regression.py` / `gimbal_param_ownership_regression.ps1` | 通过 ✓ |
| `gimbal_core_static_regression.sh` / `ai_yaw_integration_regression.sh` 自身断言 | 9 条失败（清单与基线逐条相同）/ 0 条 ✓ |
| `buildgimbal.ps1` / `buildchassis.ps1` | `Done.`（0 error）✓ |
| `format_code.ps1 --check` | 通过 ✓ |

**踩坑：** 注释里写 `User/RobotConfig/*.yaml` 会触发 `-Wcomment`
（`-Werror` 下是编译错误），已改写为 `User/RobotConfig/` 下的机器人配置。

---

## 验证结果（2026-09-24 实测）

主机测试工具链（本机 Windows / Git Bash，msys64 GCC 16.1.0）：

```bash
export PATH="/c/msys64/ucrt64/bin:$PATH"   # 必须：cc1plus.exe 依赖 ucrt64 的 DLL
CXX=/c/msys64/ucrt64/bin/g++.exe bash Modules/Gimbal/tests/yaw_smc_host_regression.sh
```

⚠️ 两个静态回归脚本各调用上百次 `rg`，本机每次进程启动约 1.7 s，整轮约 3 min。
**必须给 bash 命令显式长超时**（否则会被默认超时 SIGTERM 掐断，表现为"脚本莫名死在半路"）。

| 检查项 | 改动前 | 改动后 |
| --- | --- | --- |
| `yaw_smc_host_regression.sh` | 编译失败（19 条 error，测试未同步） | **通过** |
| `yaw_lqr_eso_host_regression.sh`（含 656 例仿真矩阵 + 物理测试） | 编译失败（1 条 error，测试未同步） | **通过** |
| `gimbal_core_static_regression.sh` | 9 条失败 | **9 条失败，清单逐条相同** |
| `ai_yaw_integration_regression.sh` 自身断言 | 0 条失败（它的整体失败来自嵌套调用的核心脚本） | **0 条失败，且新增 4 条断言** |
| `selected_feature_removal_regression.py` | 通过 | **通过**（forbidden 新增 6 条、required 更新 2 条） |
| `gimbal_config_order_regression.py` | 通过 | **通过** |
| `gimbal_param_ownership_regression.ps1` | 通过 | **通过** |
| `parse_cmd_behavior_regression.py --negative-checks` | 失败：`ParseCMD body not found` | **同一失败，未变化** |
| `buildgimbal.ps1 --skip-format` | 通过 | **通过（0 error）** |
| `buildchassis.ps1 --skip-format` | 通过 | **通过（0 error）** |
| `format_code.ps1 --check` | 通过 | **通过** |

拿到"完整失败清单"而非首个失败的方法：把两个 bash 回归脚本机械转换为"计数不退出"版本
（`exit 1` → 计数），并把 `HEADER` / `ALGORITHM_HEADER` / `SMC_HEADER` 改成环境变量，
以便用**同一套断言**分别跑改动前/改动后的文件。

⚠️ 基线必须取**改动前的工作区快照**，不能取 `git show HEAD:`：本工作区在本次改动前
已含大量未提交重构，`HEAD` 版本的核心回归会报 41 条失败、ai_yaw 报 10 条，与真实基线
（9 / 0）完全不同。转换脚本只存在于系统临时目录，未进入仓库。
