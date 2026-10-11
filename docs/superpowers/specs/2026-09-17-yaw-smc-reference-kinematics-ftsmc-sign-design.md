# Yaw SMC 参考运动学与 FTSMC 符号修复设计

## 背景

当前 `YawSmc` 基于 `D:\RoboMaster\实例\smc_controller` 中的 `SMC_Tick()` 改造，并将接口统一为 SI 单位和力矩输出。现有实现仍保留了原参考代码中不带采样周期的目标角度差分，同时在 FTSMC 负角度误差分支中使用了与原始代码及数学推导不同的符号。

这两个问题分别造成：

1. `Reference::omega_rad_s` 和 `Reference::alpha_rad_s2` 虽由调用方传入，却没有参与控制；内部 `rad/sample` 差分结果与反馈 `rad/s` 混算，控制行为依赖目标更新方式且物理量纲不成立。
2. FTSMC 中 `sig^r(e)` 的导数项在负误差时多出一个符号，破坏正负状态的奇对称性，可能造成左右响应不一致。

## 目标

- 直接使用调用方提供的目标角速度和目标角加速度。
- 使线性 SMC 与 FTSMC 控制律在 SI 单位下量纲一致。
- 修正 FTSMC 分数次幂导数项，使其与数学推导和原始参考代码一致。
- 保持 `YawSmc::Config`、`YawSmc::Reference`、`Calculate()`、`Reset()` 和 YAML 构造参数的外部结构不变。
- 保持死区、角度回绕、软限幅、硬限幅和力矩变化率限制的现有工程行为。

## 非目标

- 本次不调整 `sentry_gimbal.yaml` 中的 `c`、`k`、`epsilon`、`q/p` 或力矩限制。
- 本次不改变死区内直接输出零力矩的语义。
- 本次不增加参考运动学的内部差分、滤波或兼容模式。
- 本次不解决 FTSMC 与线性 SMC 在切换边界上的连续性问题。
- 本次不修改 Gimbal 模块以外的电机、AHRS 或命令模块。

## 方案选择

采用直接修正方案：`YawSmc` 信任 `Reference` 中完整且有限的 SI 参考状态，不保留旧算法开关。

未采用的方案：

- 内部按 `dt` 差分：会重复调用方已有轨迹信息，并引入噪声、滤波参数与相位滞后。
- 旧新行为配置切换：会永久保留错误量纲和错误符号，增加调参与维护歧义。
- 分两次上线：会产生数学语义不完整的中间版本，并要求重复实车整定。

## 控制律

定义：

```text
e     = wrap(theta - theta_ref)
e_dot = omega - omega_ref
r     = q / p
```

线性 SMC 区域：

```text
s = e_dot + c * e
surface_dot_term = c * e_dot
```

FTSMC 区域：

```text
sig_r(e) = sign(e) * abs(e)^r
s = e_dot + c * sig_r(e)
surface_dot_term = c * r * abs(e)^(r - 1) * e_dot
```

控制输出：

```text
tau_ff_alpha = J * alpha_ref
tau_smc = J * (-surface_dot_term
               - epsilon * sat(s / sat_boundary)
               - k * s)
tau_pre_limit = tau_ff_alpha + tau_smc
```

随后沿用现有顺序：

```text
软限幅 -> 硬限幅 -> 力矩变化率限制 -> tau_cmd
```

### FTSMC 符号依据

当：

```text
sig_r(e) = sign(e) * abs(e)^r
```

且 `e != 0` 时：

```text
d(sig_r(e))/dt = r * abs(e)^(r - 1) * e_dot
```

导数系数不包含 `sign(e)`。参考实现中的 `e_qp / error` 也可化简为 `abs(e)^(r - 1)`。当前模块的 `e_qp / abs(error)` 则额外保留了 `sign(e)`，因此需要修正。

## 数据流

```text
Gimbal::SolveSmcYaw
  -> Reference{theta_ref, omega_ref, alpha_ref}
  -> YawSmc::Calculate
       -> 角度最短路误差
       -> 角速度误差
       -> 线性 SMC 或 FTSMC
       -> 目标角加速度前馈
       -> 软/硬力矩限幅
       -> 力矩变化率限制
  -> MotorCmd::torque
```

手动控制继续提供 `target_yaw_cmd_`、`target_yaw_dot_` 和 `target_yaw_ddot_`；AI 控制继续提供 `cmd_data_.yaw`、`cmd_data_.yaw_dot` 和 `cmd_data_.yaw_ddot`。调用点无需修改。

## 接口与状态

### 保持不变

- `YawSmc::Reference` 三个字段及单位。
- `YawSmc::Calculate(..., float dt_s)` 签名。
- `YawSmc::Reset(float theta_rad, float omega_rad_s, float previous_applied_torque_nm)` 签名。
- `YawSmc::Output` 字段。
- YAML `yaw_smc` 配置结构。

### 删除内部状态

删除：

```cpp
float target_last_rad_{};
float target_dot_rad_s_{};
```

`Reset()` 保留三个参数以避免公共接口扩散变更，但 `theta_rad` 和 `omega_rad_s` 不再初始化目标历史；它只恢复已施加力矩和 slew anchor 状态。

## 校验与边界条件

- `theta_ref`、`omega_ref`、`alpha_ref`、反馈和 `dt_s` 必须为有限值。
- 保持当前 `0.0005 < dt_s <= 0.02` 的调用周期检查，即使关闭 slew limit 也不改变。
- `abs(e) < error_deadband_rad` 时仍直接返回有效零输出。
- `abs(e) >= ftsmc_switch_rad` 时进入 FTSMC，等号行为保持不变。
- FTSMC 只在正切换阈值之外计算，因此 `abs(e)^(r-1)` 不会在零点求值。
- 输出保护顺序和 `CommitAppliedTorque()` 语义保持不变。

## 测试策略

测试不得通过复制被测实现来证明正确性。关键数学性质使用显式解析期望值验证。

### 参考运动学测试

1. 改变 `omega_ref`，验证 `e_omega = omega - omega_ref`。
2. 改变 `alpha_ref`，验证 `tau_ff_alpha = J * alpha_ref`。
3. 关闭 slew limit 后，以不同合法 `dt_s` 调用两个全新控制器，验证核心力矩相同。
4. 删除依赖内部目标历史的测试，替换为调用方参考状态测试。

### FTSMC 测试

1. 对 `(+e, +e_dot)` 与 `(-e, -e_dot)` 使用相同零前馈，验证 `tau_smc` 奇对称。
2. 使用负角度误差和非零角速度误差，按 `c*r*abs(e)^(r-1)*e_dot` 计算显式期望值。
3. 保留切换阈值、死区与角度回绕测试。

### 回归测试

- 配置校验。
- 线性 SMC 解析值。
- 软限幅和硬限幅顺序。
- slew 只锚定已提交力矩。
- 非有限输入和非法 `dt_s` 拒绝逻辑。
- Gimbal 静态集成检查。
- Gimbal 与 Chassis 两套固件构建。

## 文档与参数迁移

README 应明确：

- 当前实现来源于参考工程，但已修正为 SI 参考状态接口，不再逐行复刻 `SMC_Tick()`。
- `Reference::omega_rad_s` 和 `Reference::alpha_rad_s2` 会直接参与控制。
- FTSMC 导数项使用 `abs(e)^(r-1)`。
- 修复后移动目标和负误差方向的输出会变化，需要重新执行正负阶跃与运动目标测试。

本次不修改 YAML 参数。算法修复与实车整定必须分开提交，以免无法判断行为变化来自公式还是参数。

## 风险与控制措施

| 风险 | 影响 | 控制措施 |
| --- | --- | --- |
| 调用方目标速度或加速度质量差 | 移动目标产生错误力矩 | 保留有限值校验；实车记录三项参考状态 |
| 负误差方向输出发生明显变化 | 原参数可能只在一侧表现稳定 | 执行对称正负阶跃，先使用保守力矩限制 |
| 既有测试 oracle 复制旧错误 | 测试通过但公式仍错 | 使用解析值和奇对称性质测试 |
| README 与实现再次偏离 | 后续调参依据错误 | 在同一算法提交中同步 README |

## 验收标准

- `omega_ref` 和 `alpha_ref` 分别进入角速度误差与角加速度前馈。
- `YawSmc` 不再保存或计算目标角度历史差分。
- FTSMC 正负镜像状态的反馈力矩满足奇对称，误差在浮点容差内。
- 所有现有保护逻辑测试通过。
- 主机 SMC 测试、Gimbal 静态回归、格式检查和两套固件构建通过。
- YAML 控制参数未在算法修复提交中改变。
