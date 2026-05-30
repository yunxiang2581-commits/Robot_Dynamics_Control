# Step B03-R3: Sampling Solver Benchmark Stabilization

## 1. 本步目标

把当前 sampling-family solver 对比整理成可复现 benchmark，并针对 CEM / MPPI-lite 做参数诊断与报告输出。

## 2. 当前 solver comparison 图的结论

| Solver | 收敛速度 | 控制平滑性 | 综合评价 |
| --- | --- | --- | --- |
| Random Shooting | 慢 | 中等 | 基线对照组 |
| Warm-start Sampling | 较快 | 抖动大 | 误差下降快但控制不稳 |
| CEM | 快 | 好 | 综合表现最好 |
| MPPI-lite | 较快 | 需调参 | 误差好但平滑性待优化 |

## 3. 为什么先稳定 sampling-family benchmark

- sampling solver 不需要梯度、线性化或 QP 求解器。
- 在低维系统上可以直接验证 rollout cost、warm start、分布更新、early stop 的正确性。
- 稳定的 benchmark baseline 后续可以作为 iLQG / SQP / NMPC 的对照组。
- 统一 metrics 输出保证不同 solver 之间可比。

## 4. CEM 当前优势与 runtime 问题

优势：

- 多轮 elite 更新让搜索快速收敛。
- global best tracking 避免丢失历史最优。
- smoothing_alpha 防止均值/方差跳变。
- early stop 避免无效迭代。

Runtime 问题：

- num_candidates * num_iterations 次 rollout 在长 horizon 下开销大。
- 当前仍是标量 std，没有 full covariance。
- 没有 batched / parallel rollout。

## 5. MPPI-lite 当前问题与检查点

问题：

- predicted_controls 之前使用 best sample，已修正为 updated_sequence。
- weight 分布可能过于集中（temperature 太小）或过于分散（temperature 太大）。
- 数值稳定性：exp 可能溢出，已加入 logit 平移保护。

检查点：

- temperature 对权重熵的影响。
- noise_std 对搜索范围的影响。
- updated_sequence vs best sample 对闭环跟踪的影响。

## 6. 新增 metrics summary

输出文件：

```text
outputs/runs/B03_mpc_solver_ladder_demo/<run_id>/metrics/B03_solver_comparison_metrics.csv
```

每个 solver 一行，字段包括：

- solver_name
- final_ee_error / mean_ee_error / max_ee_error
- mean_best_cost / std_best_cost
- mean_runtime_ms / max_runtime_ms
- mean_abs_torque / max_abs_torque
- control_smoothness / trajectory_smoothness
- success_rate
- horizon / num_candidates / num_iterations

## 7. 当前不实现内容

- iLQG / iLQG-lite：保持 TODO skeleton。
- SQP-MPC：保持 TODO skeleton。
- Direct Multiple Shooting NMPC：保持 TODO skeleton。
- full covariance CEM。
- per-step / per-dim noise covariance for MPPI。
- batched / parallel rollout。
- 长时间 MuJoCo 仿真。
- 正式长视频生成。

## 8. 下一步建议

- B03-R4：mini iLQR / iLQG learning skeleton。
- B03-R3T：CEM / MPPI 参数网格小规模调参（temperature、noise_std、num_candidates、num_iterations）。

## 9. 验收清单

- [x] 统一 B03 solver comparison metrics
- [x] MPPI-lite 输出语义检查完成（updated_sequence）
- [x] CEM early stopping skeleton 完成（已内置，默认关闭）
- [x] 更新 B03 文档
- [x] 新增或更新测试
- [x] 未修改 B02 benchmark 配置
- [x] 未修改 B02 regression 配置
- [x] 未修改 B02 controller 核心逻辑
- [x] 未运行长时间 MuJoCo 仿真
- [x] 未生成正式长视频
- [x] 未执行 git add / commit / push
