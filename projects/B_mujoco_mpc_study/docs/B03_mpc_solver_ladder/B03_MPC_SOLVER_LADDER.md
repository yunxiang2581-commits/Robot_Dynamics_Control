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

Random Shooting 是 B03 solver ladder 的第一层，也是最适合理解 predictive sampling 基本数据流的入口。它的核心想法非常直接：当前时刻并不知道哪一段未来控制序列最好，那就随机采样很多条候选控制序列，对每条候选做 rollout，计算整段 horizon cost，再选出 cost 最小的一条。最后并不会一次性执行整段最优控制序列，而只执行它的第一步控制，然后在下一时刻基于新的真实状态重新规划。

若第 `i` 条候选控制序列记作：

```text
U_i = [u_{i,0}, u_{i,1}, ..., u_{i,H-1}]
```

那么最基础的采样可以写成：

```text
u_{i,t} ~ N(0, sigma^2 I)
u_{i,t} <- clip(u_{i,t}, -u_max, u_max)
```

对每条候选进行 rollout：

```text
x_{i,t+1} = f(x_{i,t}, u_{i,t})
```

并计算总 cost：

```text
J_i = sum_t l(x_{i,t}, u_{i,t}) + l_T(x_{i,H})
```

最终选出：

```text
i* = argmin_i J_i
u_apply = u_{i*,0}
```

它的优点是实现最简单、概念最透明、不依赖梯度或线性化，特别适合作为 B03 的入门 solver；缺点是每一拍都从零开始随机猜测，样本效率较低，维度和 horizon 一增大就会变得吃力。

Warm-start Predictive Sampling：

Warm-start Predictive Sampling 是对 Random Shooting 的第一层改进。它利用上一拍已经找到的最优控制序列作为当前拍的搜索起点，而不是每次都从零开始。假设上一拍最优控制序列为：

```text
U*_{k-1} = [u*_0, u*_1, ..., u*_{H-1}]
```

由于上一拍只真正执行了第一步 `u*_0`，那么到当前拍 `k` 时刻，一个自然的 warm start 初值就是把旧序列向前平移一格：

```text
U_bar_k = [u*_1, u*_2, ..., u*_{H-1}, u*_{H-1}]
```

这就是 `shift previous best sequence`。然后新的候选控制不是围绕 0 采样，而是围绕这个 shifted mean 采样：

```text
u_{i,t} ~ U_bar_k[t] + N(0, sigma^2 I)
u_{i,t} <- clip(u_{i,t}, -u_max, u_max)
```

这种做法的动机来自 receding horizon 的连续性：上一拍规划的未来第二步、第三步，往往正好是当前拍未来第一步、第二步的合理初值。对于平滑 tracking 任务，warm-start 往往能让控制更连续、采样更集中、搜索更高效。它的优点是比纯随机采样更稳定、更平滑；缺点是如果上一拍最优解本身不好，或者系统状态突然发生较大变化，搜索也可能被带到不理想的局部区域，因此仍然需要保留噪声扰动。

CEM-MPC：

CEM-MPC（Cross-Entropy Method MPC）可以看作 sampling MPC 的分布迭代版。Random Shooting 和 Warm-start 通常是一轮采样、一轮评估、一轮选最优；CEM 则会从当前采样结果里挑出一批“精英样本”，再用这些样本更新下一轮采样分布的均值和方差，从而逐轮把搜索集中到更有希望的区域。

假设当前控制分布由 `mean_t` 和 `std_t` 描述，那么每轮迭代的核心步骤是：

```text
1. sample:
   u_{i,t} ~ N(mean_t, std_t^2 I)
2. evaluate:
   rollout every U_i and compute J_i
3. elite selection:
   keep top-K lowest-cost candidates
4. update:
   mean_t <- average of elites at time t
   std_t  <- std of elites at time t
5. repeat
```

若 elite 集合记为 `E`，那么更新可以写成：

```text
mean_t_new = (1 / |E|) * sum_{i in E} u_{i,t}
std_t_new^2 = (1 / |E|) * sum_{i in E} (u_{i,t} - mean_t_new)^2
```

CEM 的关键不在于只看单条最优样本，而在于从“一小群表现好的样本”中总结下一轮更聪明的搜索分布。因此它通常比单次 random shooting 更稳、更高效，又不像梯度法那样依赖显式导数。它的常见问题是：如果 elite 比例太小，或者 `std` 缩得太快，就容易过早塌缩到局部解；如果 `std` 一直很大，又会导致搜索过散、收敛变慢。

MPPI-lite：

MPPI-lite（Model Predictive Path Integral 的轻量学习版）也是采样型 MPC，但它和 CEM 的区别在于：CEM 会对样本做 hard elite selection，而 MPPI 让所有样本都参与更新，只是让低 cost 样本权重大、高 cost 样本权重小。因此它更像是一种带温度参数的 cost-weighted averaging。

设当前 nominal control sequence 为：

```text
U = [u_0, u_1, ..., u_{H-1}]
```

MPPI 会先围绕它采样噪声：

```text
u_{i,t} = u_t + epsilon_{i,t}
epsilon_{i,t} ~ N(0, Sigma)
```

对每条样本 rollout 得到 cost `J_i` 后，计算权重：

```text
w_i ∝ exp(-(J_i - J_min) / lambda)
```

归一化后：

```text
w_i = exp(-(J_i - J_min) / lambda) / sum_j exp(-(J_j - J_min) / lambda)
```

再利用这些权重对噪声做加权平均，更新 nominal control：

```text
u_t_new = u_t + sum_i w_i * epsilon_{i,t}
```

这里的 `lambda` 是 temperature 参数，用来控制权重分布的尖锐程度。当 `lambda` 很小时，最低 cost 的少数样本会占据几乎全部权重，MPPI 的行为会更接近“只相信最好几个样本”；当 `lambda` 较大时，更多样本会共同参与更新，更新更平滑，但也可能不够果断。相比 CEM，MPPI 的特点是更新更连续、更软，更适合在当前 nominal trajectory 附近做平滑修正，因此它也是 B03 中非常重要的一层 sampling-to-optimization 过渡方法。

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

## 4.5 当前代码实现状态（sampling solver）

截至当前 B03 代码状态，sampling solver ladder 的前四层已经从“纯文档骨架”推进到“统一接口下的最小可运行学习版”，但实现强度仍然保持在 B03 可学习范围内，而不是直接追求 full covariance NMPC 风格的重型实现。

### Random Shooting

- 已实现统一 `solve(problem, previous_solution=None)` 接口。
- 已实现 `sample_candidate_controls(...)`、`rank_rollouts(...)` 和统一 `MPCSolution` 打包。
- 已能通过 `problem.rollout_cost_fn(candidate_controls, problem)` 消费 adapter 返回的：
  - 纯 `costs`
  - 或包含 `costs / predicted_states / predicted_ee_positions` 的结果对象。
- 当前定位是 sampling MPC 的最基础对照组，用来解释 candidate controls、rollout、cost ranking、selected rollout 和 receding horizon 执行逻辑。

### Warm-start Predictive Sampling

- 已实现 `shift_control_sequence(...)`。
- 已实现从 `previous_solution.predicted_controls` 自动提取 warm-start 均值序列。
- 当前每拍会围绕 shifted mean 重新采样，而不是从零均值重新开始。
- 已能消费同一套 `rollout_cost_fn` 协议，因此可以直接复用 B02-to-B03 adapter 提供的真实 two-link rollout cost。
- 当前定位是 Random Shooting 的第一层增强版，重点学习 MPC 中“上一拍最优解如何作为下一拍初值”。

### CEM-R2

- 已从 TODO skeleton 升级为可运行的 CEM-R2 学习版。
- 当前实现包含：
  - warm-start `mean_sequence`
  - 多轮采样迭代
  - `elite_ratio -> elite_count`
  - elite controls 更新均值
  - 标量 `std` 更新
  - `smoothing_alpha`
  - `min_std / max_std`
  - global best tracking
  - early stop
- 当前仍然是 B03 友好的轻量实现：
  - 还不是 full covariance CEM
  - 还没有 weighted elite update
  - 还没有 batched / parallel rollout
- 当前定位是“从单轮采样进入分布迭代优化”的第一版实践。

### MPPI-R2

- 已从 TODO skeleton 升级为可运行的 MPPI-R2 学习版。
- 当前实现包含：
  - warm-start `mean_sequence`
  - 多轮采样迭代
  - `temperature lambda`
  - soft weights
    ```text
    w_i ∝ exp(-(J_i - J_min) / lambda)
    ```
  - 基于所有样本的 cost-weighted averaging
  - 标量 `noise_std` 更新
  - `smoothing_alpha`
  - `min_std / max_std`
  - global best tracking
  - early stop
- 当前仍保持轻量：
  - 还没有 per-step / per-dim noise covariance
  - 还没有更重的 control smoothing shaping
  - 还没有更完整的 batched rollout 加速
- 当前定位是“从硬 elite selection 过渡到软权重更新”的第一版实践。

### 当前总结

目前 `sampling_mpc_solvers.py` 中四个 sampling solver 的状态可以概括为：

- `RandomShootingSolver`：可运行
- `WarmStartSamplingSolver`：可运行
- `CEMShootingSolver`：CEM-R2 可运行
- `MPPILiteSolver`：MPPI-R2 可运行

这意味着 B03 solver ladder 的前四层已经具备统一接口、统一测试入口和统一 adapter 协议，可以作为后续小规模 solver comparison loop 的直接基础。

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
- 更完整的 adapter 解析和接入说明见 [B02_TO_B03_ADAPTER.md](B02_TO_B03_ADAPTER.md)。

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
- 新 planner 接入前，先按 [B02_TO_B03_ADAPTER.md](B02_TO_B03_ADAPTER.md) 的检查清单确认输入输出协议。

## 9. B03-R3 Sampling Solver Benchmark

### 9.1 当前对比的 solver

| Solver | 层级 | 状态 |
|--------|------|------|
| Random Shooting | Level 1 | 可运行 |
| Warm-start Sampling | Level 2 | 可运行 |
| CEM | Level 3 | CEM-R2 可运行 |
| MPPI-lite | Level 4 | MPPI-R2 可运行 |

### 9.2 当前实验图的结论

- **Random Shooting**：收敛慢，每拍从零采样，样本效率最低。
- **Warm-start Sampling**：误差下降快，但控制抖动较大（因为 shifted mean 周围噪声仍大）。
- **CEM**：综合表现最好，多轮 elite 更新让搜索快速收敛，误差和控制平滑性均优。
- **MPPI-lite**：误差表现好，但控制平滑性需要继续调参（temperature / noise_std）。

### 9.3 CEM 关键参数

| 参数 | 说明 |
|------|------|
| `num_candidates` | 每轮采样候选数 |
| `num_iterations` | 最大迭代轮数 |
| `elite_ratio` | elite 样本比例 |
| `initial_std` | 初始采样标准差 |
| `min_std` | 最小标准差（防过早塌缩） |
| `smoothing_alpha` | 均值/方差平滑系数 |
| `early_stop_patience` | 连续无改善轮数阈值（默认 0 = 关闭） |
| `improvement_tolerance` | 改善判定阈值 |

### 9.4 MPPI-lite 关键参数

| 参数 | 说明 |
|------|------|
| `temperature` | 权重温度 λ，控制权重分布尖锐程度 |
| `noise_std` | 采样噪声标准差 |
| `smoothing_alpha` | 均值/方差平滑系数 |
| 输出策略 | `predicted_controls` 使用 updated_sequence（weighted average），`first_control` 使用 best sample |
| `weight_entropy` | 权重分布熵，记录在 metadata 中 |

### 9.5 统一 metrics 输出

B03-R3 新增统一 comparison metrics CSV：

```text
outputs/runs/B03_mpc_solver_ladder_demo/<run_id>/metrics/B03_solver_comparison_metrics.csv
```

每个 solver 一行，包含：
`solver_name, final_ee_error, mean_ee_error, max_ee_error, mean_best_cost, std_best_cost, mean_runtime_ms, max_runtime_ms, mean_abs_torque, max_abs_torque, control_smoothness, trajectory_smoothness, success_rate, horizon, num_candidates, num_iterations`

### 9.6 下一步

- B03-R4：mini iLQR / iLQG learning skeleton
- B03-R3T：CEM / MPPI 参数网格小规模调参

## 10. B03-R3T Sampling Solver Automated Tuning

B03-R3T 在 B03-R3 基础上，对 CEM、MPPI-lite 和 Warm-start Sampling 做小规模自动化参数扫描，通过加权评分选出各 solver family 的推荐参数。

### 10.1 调参目标

- 输入：`configs/B03_sampling_tuning.yaml`（11 个 variant：4 CEM + 4 MPPI-lite + 3 Warm-start）
- 输出：`outputs/runs/B03_sampling_solver_tuning/<run_id>/`
  - `metrics/sweep_metrics.csv` — 所有 variant 的 metrics + score
  - `figures/` — CEM sweep、MPPI temperature sweep、Warm-start smoothness sweep、score comparison
  - `reports/tuning_report.md` — 完整调参报告

### 10.2 评分机制

```text
score = w_error * (mean_ee_error / max_error)
      + w_runtime * (mean_runtime_ms / max_runtime)
      + w_ctrl * control_smoothness
      + w_traj * trajectory_smoothness
```

lower-is-better。不满足约束（success_rate < 0.95, runtime > 500ms, error > 0.15）的 variant 标记为 infeasible，score = inf。

### 10.3 Variant 概览

| Family | Variant | 关键参数差异 |
|--------|---------|-------------|
| CEM | cem_fast | 128 candidates, 2 iterations |
| CEM | cem_default | 256 candidates, 3 iterations |
| CEM | cem_stable | 256 candidates, elite_ratio=0.20 |
| CEM | cem_early_stop | 4 iterations, early_stop enabled |
| MPPI-lite | mppi_temp_0p5 | temperature=0.5 |
| MPPI-lite | mppi_temp_1p0 | temperature=1.0 |
| MPPI-lite | mppi_temp_2p0 | temperature=2.0 |
| MPPI-lite | mppi_temp_5p0 | temperature=5.0 |
| Warm-start | warm_start_default | torque_rate_weight=0.0 |
| Warm-start | warm_start_smooth_001 | torque_rate_weight=0.001 |
| Warm-start | warm_start_smooth_01 | torque_rate_weight=0.01 |

### 10.4 运行方式

```bash
python projects/B_mujoco_mpc_study/simulator/scripts/run_B03_sampling_solver_tuning.py \
  --config projects/B_mujoco_mpc_study/configs/B03_sampling_tuning.yaml \
  --run-id tuning_v1 \
  --save-figures --save-report
```

### 10.5 代码入口

- Runner: `simulator/scripts/run_B03_sampling_solver_tuning.py`
- Reporter: `simulator/utils/sampling_tuning_reporter.py`
- Config: `configs/B03_sampling_tuning.yaml`

## 11. B03-R3T-Benchmark Recommended Sampling Solver Configs

### 11.1 为什么需要固定推荐配置

B03-R3T 自动调参已经确认了 CEM、MPPI-lite 和 Warm-start 三个 solver family 的最佳参数方向。在进入 B03-R4（mini iLQR / iLQG）之前，需要将这些调参结论固化为一组推荐配置，并用统一的短 benchmark 验证它们的相对表现。这样后续可以：

- 以推荐配置作为 sampling-family 的默认 baseline
- 在 iLQG 实现完成后直接与推荐配置对比
- 避免每次实验都重新调参

### 11.2 四类 Solver 定位

| Solver Config | Solver Family | 定位 |
| --- | --- | --- |
| random_shooting | Random Shooting | baseline 对照组，不使用任何先验知识 |
| warm_start_fast | Warm-start Sampling | 低 runtime 优先，复用上一拍最优解 |
| cem_default | CEM-MPC | 综合默认推荐，精度与 runtime 平衡 |
| mppi_smooth | MPPI-lite | 平滑性优先，软权重平均产生最平滑控制 |

### 11.3 推荐配置表

| 参数 | random_shooting | warm_start_fast | cem_default | mppi_smooth |
| --- | --- | --- | --- | --- |
| num_candidates | 128 | 96 | 128 | 128 |
| sampling_std / initial_std / noise_std | 2.0 | 1.0 | 2.0 | 1.5 |
| torque_limit | 10.0 | 10.0 | 10.0 | 10.0 |
| num_iterations | - | - | 2 | - |
| elite_ratio | - | - | 0.10 | - |
| min_std | - | - | 0.10 | - |
| temperature | - | - | - | 2.0 |
| torque_rate_weight | - | 0.01 | - | - |
| early_stop | - | - | enabled | - |

### 11.4 运行方式

```bash
python projects/B_mujoco_mpc_study/simulator/scripts/run_B03_sampling_solver_tuning.py \
  --recommended-benchmark \
  --run-id recommended_v1 \
  --num-steps 100 \
  --horizon 16 \
  --save-figures \
  --save-recommended-report
```

### 11.5 输出

```text
outputs/runs/B03_sampling_recommended_benchmark/<run_id>/
  figures/B03_recommended_error_comparison.png
  figures/B03_recommended_runtime_comparison.png
  figures/B03_recommended_control_smoothness.png
  figures/B03_recommended_trajectory_smoothness.png
  metrics/B03_recommended_comparison_metrics.csv
  reports/B03_recommended_sampling_solver_report.md
  logs/B03_recommended_benchmark_run_log.txt
```

### 11.6 当前限制

- 短 benchmark（100 步），不是最终长 benchmark（1000 步）。
- 单一 seed，未测试 seed 敏感性。
- 仅 circle target，未测试 lissajous 或 figure-8。
- 不包含 iLQG / SQP / NMPC solver。

### 11.7 下一步

- B03-R4：mini iLQR / iLQG learning skeleton
- iLQG 实现后，用推荐 sampling 配置作为 baseline 进行对比

## B03-R4B mini iLQR Core Loop

B03-R4B 将 `iLQR / iLQG-lite` 从 learning skeleton 推进到最小可运行版本。当前实现仍然是 deterministic mini iLQR-lite，用于理解局部轨迹优化的核心数据流，不是工业级 iLQG / NMPC。

本步实现的核心流程：

1. `rollout_nominal_trajectory(...)`：从 `x0` 和控制序列 `U` 生成 nominal states、stage costs 和 total cost。
2. `linearize_trajectory_dynamics(...)`：沿 nominal trajectory 通过有限差分得到每一步的 `A_t = df/dx` 与 `B_t = df/du`。
3. `quadratize_trajectory_cost(...)`：对 tracking cost 写出解析一阶项和二阶项。
4. `backward_pass(...)`：从 terminal cost 反向递推 value approximation，求 feedforward `k` 与 feedback `K`。
5. `forward_pass(...)`：用 `u_new = u_bar + alpha * k + K dx` 在真实 dynamics 上前向 rollout，并配合 line search 判断是否接受。
6. `ILQGLiteSolver.solve(...)`：组合 rollout、linearization、quadratization、backward pass 和 forward line search，返回统一 `MPCSolution`。

当前 toy smoke demo 使用 double-integrator：

```text
x = [position, velocity]
u = [acceleration]
x_next[0] = x[0] + dt * x[1]
x_next[1] = x[1] + dt * u[0]
```

运行入口：

```bash
python projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqg_lite_demo.py \
  --config projects/B_mujoco_mpc_study/configs/B03_ilqg_lite.yaml \
  --run-id ilqg_toy_smoke \
  --model-family toy \
  --horizon 32 \
  --max-iterations 10 \
  --save-report
```

当前仍不包含：

- stochastic iLQG noise propagation
- full DDP 二阶动力学项
- constraints / SQP / full NMPC
- OSQP / IPOPT / acados
- MuJoCo long simulation 或正式 MP4
- 对 B02 benchmark / regression 配置的修改
- 对 B02 controller 核心逻辑的修改

## B03-R4C mini iLQR Two-Link State Tracking Smoke

B03-R4C 已在不改变 B02 controller、不修改 B02 benchmark/regression 配置的前提下，把 B02 two-link MuJoCo dynamics 接到 `ILQGLiteSolver`，形成一个可运行、可复盘的 joint-space state tracking smoke。

当前 two-link 状态和控制约定：

```text
x = [q1, q2, dq1, dq2]
u = [tau1, tau2]
```

### R4C 当前完成链条

```text
TwoLinkEnv
-> get/set/save/restore state
-> dynamics_fn(x,u) -> x_next
-> build_state_tracking_problem(...)
-> ILQGLiteSolver.solve(problem)
-> metrics / cache / figures / report
```

### R4C 分步状态

| Step | 状态 | 内容 |
| --- | --- | --- |
| R4C-1A | 完成 | 保留 skeleton，定义 adapter 契约 |
| R4C-1B | 完成 | two-link state get/set/save/restore |
| R4C-1C | 完成 | `dynamics_fn(x,u)->x_next`，并保证不污染真实 env |
| R4C-1D | 完成 | 构造最小 state tracking `MPCProblem` |
| R4C-1E | 完成 | 输出 cost history CSV、metrics CSV、NPZ cache |
| R4C-1F | 完成 | 输出 state tracking 图和 cost history 图 |
| R4C-1G | 完成 | 输出 smoke Markdown report |

### R4C 运行入口

```bash
D:\anaconda\envs\mujoco_py311\python.exe \
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py \
  --output-dir outputs/pytest_tmp/B03_R4C1G_real_smoke \
  --log-level INFO
```

### R4C 输出文件

```text
outputs/metrics/B03_R4C_ilqr_cost_history.csv
outputs/metrics/B03_R4C_ilqr_two_link_smoke_metrics.csv
outputs/cache/B03_R4C_two_link_state_tracking_placeholder.npz
outputs/figures/B03_R4C_two_link_state_trajectory_todo.png
outputs/figures/B03_R4C_ilqr_cost_history_todo.png
outputs/reports/B03_R4C_ilqr_two_link_smoke_report.md
```

### R4C 示例 smoke 指标

```text
success: True
horizon: 32
state_dim: 4
control_dim: 2
best_cost: 1.5449372487697226e-07
initial_cost: 2.516716749963712e-07
final_cost: 1.5449372487697226e-07
final_state_error_norm: 4.226534633890562e-05
```

### R4C 当前边界

当前 R4C 仍然不包含：

- task-space end-effector tracking cost；
- CEM / MPPI warm-start；
- long benchmark；
- 正式 MP4；
- SQP / full NMPC；
- B02 controller 核心逻辑改写。

下一步可以选择：

1. B03-R4D：把 state tracking 推进到 task-space end-effector tracking；
2. 或 B03-R4C-2：把 sampling-family `predicted_controls` 作为 iLQR-lite warm start。
