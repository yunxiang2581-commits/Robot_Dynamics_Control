# B03 MPC Solver Ladder

B03 已从旧的 `rollout predictive sampling demo` 重新定义为 `MPC solver ladder demo`。

它不替代 B02。B02 负责证明双连杆 task-space MPC 已经能稳定跟踪；B03 负责学习不同 MPC solver 如何更快、更稳、更可解释地完成同类跟踪问题。

进入 B03 前仍建议先运行 `B02-regression-light` 做健康检查。

## 1. 学习目标

B03 的重点是统一理解 solver 的结构、接口、指标和可视化方式，而不是一开始追求完整高性能 NMPC。

统一闭环数据流：

```text
current_state
  -> solver.solve(...)
  -> predicted_states
  -> predicted_controls
  -> first_control
  -> env.step(first_control)
  -> log metrics
  -> repeat
```

统一 solver 输入：

- `current_state`
- `target_horizon`
- dynamics / MuJoCo env wrapper
- `cost_config`
- `solver_config`
- optional `previous_solution`

统一 solver 输出：

- `first_control`
- `predicted_states`
- `predicted_controls`
- `best_cost`
- `solver_stats`
- `runtime_ms`

runner、logger、visualizer 不应知道 solver 内部是 shooting、CEM、MPPI、iLQG、SQP 还是 NMPC。

## 2. Solver Ladder

```text
Level 1: Random Shooting / Predictive Sampling
Level 2: Warm-start Predictive Sampling
Level 3: CEM-MPC
Level 4: MPPI-lite
Level 5: iLQR / iLQG-lite
Level 6: SQP-MPC learning version
Level 7: Direct Multiple Shooting NMPC skeleton
```

为什么先在简单模型中学习：

- iLQG、SQP、full NMPC 的数学和工程复杂度高。
- 单关节和二连杆模型能把 rollout、cost、linearization、backward pass、QP subproblem、multiple shooting constraints 拆清楚。
- 低维系统更容易可视化误差、runtime 和控制平滑性。

## 3. Sampling 子专题

旧 B03 rollout predictive sampling 内容现在作为 B03 的 sampling 子专题保留。

一次 predictive sampling 规划：

```text
u_i[0:H-1] ~ sampling distribution
x_i[0:H], p_i[0:H] = rollout(x_current, u_i[0:H-1])
J_i = sum_k residual(x_i[k], p_i[k], target[k], u_i[k])
i* = argmin_i J_i
u_execute = u_i*[0]
```

需要区分：

- `candidate rollout`：planner 对候选控制序列想象出的未来。
- `best rollout`：当前 step cost 最小的预测未来。
- `actual trajectory`：闭环仿真一步步真实执行后产生的轨迹。

MPC 每次只执行 best sequence 的第一步，然后重新观测状态并重规划。

## 4. 各 Solver 核心概念

Random Shooting：

- 随机采样控制序列。
- rollout。
- cost ranking。
- 执行第一步。

Warm-start Predictive Sampling：

- shift previous best sequence。
- sample around shifted mean。
- clip by torque limit。

CEM-MPC：

- sample。
- evaluate。
- elite selection。
- update mean/std。
- repeat。

MPPI-lite：

- sample noise。
- cost-weighted averaging。
- temperature lambda。

iLQR / iLQG-lite：

- nominal trajectory。
- dynamics linearization。
- cost quadratic approximation。
- backward pass。
- forward rollout。
- line search。

SQP-MPC：

- nominal trajectory。
- linearized dynamics constraints。
- QP subproblem。
- update trajectory。

Direct Multiple Shooting NMPC：

```text
z = [x0, u0, x1, u1, ..., xH]
x_{k+1} - f(x_k, u_k) = 0
objective = sum l(x,u) + terminal cost
```

## 5. 与 B02 的关系

B02 是稳定 baseline，不被 B03 改写。

B03 通过 `b02_to_b03_adapter` 接入 B02 的事实层，而不是侵入 B02 controller：

- B02 target trajectory
- B02 current state
- B02 two-link MuJoCo rollout
- B02 task-space cost
- B02 visualization / tracking log 结构

边界：

- B03 solver 不直接依赖 B02 controller 内部实现。
- `B02BaselineSolver` 只作为 comparison baseline。
- Random / Warm-start / CEM / MPPI 通过统一 `MPCProblem.rollout_cost_fn` 调用 rollout 和 cost。

## 6. 输出规划

输出目录：

```text
outputs/runs/B03_mpc_solver_ladder_demo/<run_id>/
```

计划输出：

```text
videos/B03_sampling_comparison.mp4
videos/B03_ilqr_demo.mp4
videos/B03_nmpc_demo.mp4
figures/B03_solver_error_comparison.png
figures/B03_solver_runtime_comparison.png
figures/B03_solver_cost_comparison.png
figures/B03_control_smoothness_comparison.png
figures/B03_trajectory_smoothness_comparison.png
metrics/B03_solver_comparison_metrics.csv
logs/B03_solver_run_log.txt
logs/B03_solver_iteration_log.csv
```

虽然 B03-A 不生成正式 MP4，但 Project B 的最终任务标准仍是 video、figures、metrics、logs 和复现实验命令。

## 7. 指标

- `final_ee_error`
- `mean_ee_error`
- `max_ee_error`
- `runtime_per_control_step`
- `max_abs_torque`
- `mean_abs_torque`
- `control_smoothness = mean(||u_k - u_{k-1}||)`
- `trajectory_smoothness = mean(||p[k+1] - 2p[k] + p[k-1]||)`
- `best_cost`
- `cost_std`
- `solver_success_rate`

## 8. 当前状态与边界

当前 B03-R2A 状态：

- `b02_to_b03_adapter.py` 已建立。
- `MPCProblem` 已支持 `rollout_cost_fn`、`dynamics_fn` 和 `metadata`。
- Random Shooting / Warm-start Sampling 已能消费 adapter 返回的 rollout result。
- `B02BaselineSolver` 已能把 B02 controller 包装成 B03 comparison baseline。

当前仍不做：

- 不完整实现 iLQG。
- 不完整实现 SQP。
- 不完整实现 full NMPC。
- 不运行长时间仿真。
- 不生成正式 B03 MP4。
- 不修改 B02 benchmark / regression。

下一步建议：

- 将 CEM-MPC 接入同一 `rollout_cost_fn` 协议。
- 或把真实 rollout 结果接入更完整的 solver comparison loop。
