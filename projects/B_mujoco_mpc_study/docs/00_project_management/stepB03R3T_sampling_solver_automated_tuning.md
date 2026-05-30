# Step B03-R3T: Sampling Solver Automated Tuning

## 1. 本步目标

在 B03-R3 基础上，对 CEM、MPPI-lite 和 Warm-start Sampling 做小规模自动化参数扫描，通过加权评分选出各 solver family 的推荐参数。

## 2. 前置条件

- B03-R3 已完成，sampling solver 对比 metrics 已稳定。
- `sampling_mpc_solvers.py` 中四个 solver 均可运行。
- `solver_benchmark_logger.py` 提供统一 metrics 输出。

## 3. 本步产出

### 新增文件

| 文件 | 用途 |
| --- | --- |
| `configs/B03_sampling_tuning.yaml` | 调参配置（11 个 variant） |
| `simulator/scripts/run_B03_sampling_solver_tuning.py` | 调参 runner |
| `simulator/utils/sampling_tuning_reporter.py` | 评分、选优、绘图、报告生成 |
| `tests/test_B03_sampling_tuning_reporter.py` | reporter 单元测试 |
| `docs/00_project_management/stepB03R3T_sampling_solver_automated_tuning.md` | 本文档 |

### 修改文件

| 文件 | 变更 |
| --- | --- |
| `docs/B03_mpc_solver_ladder/B03_MPC_SOLVER_LADDER.md` | 新增 Section 10: B03-R3T |
| `README.md` | 新增 tuning runner 脚本和运行命令 |
| `docs/README.md` | 新增步骤管理文档索引 |

### 输出目录

```text
outputs/runs/B03_sampling_solver_tuning/<run_id>/
  metrics/sweep_metrics.csv
  figures/B03_cem_sweep.png
  figures/B03_mppi_temperature_sweep.png
  figures/B03_warm_start_smoothness_sweep.png
  figures/B03_tuning_score_comparison.png
  reports/tuning_report.md
```

## 4. Variant 概览

### CEM (4 variants)

| Variant | num_candidates | num_iterations | elite_ratio | early_stop |
| --- | --- | --- | --- | --- |
| cem_fast | 128 | 2 | 0.10 | off |
| cem_default | 256 | 3 | 0.10 | off |
| cem_stable | 256 | 3 | 0.20 | off |
| cem_early_stop | 256 | 4 | 0.10 | on |

### MPPI-lite (4 variants — temperature sweep)

| Variant | temperature | num_candidates |
| --- | --- | --- |
| mppi_temp_0p5 | 0.5 | 256 |
| mppi_temp_1p0 | 1.0 | 256 |
| mppi_temp_2p0 | 2.0 | 256 |
| mppi_temp_5p0 | 5.0 | 256 |

### Warm-start (3 variants — smoothness sweep)

| Variant | torque_rate_weight |
| --- | --- |
| warm_start_default | 0.0 |
| warm_start_smooth_001 | 0.001 |
| warm_start_smooth_01 | 0.01 |

## 5. 评分机制

```text
score = w_error * (mean_ee_error / max_error)
      + w_runtime * (mean_runtime_ms / max_runtime)
      + w_ctrl * control_smoothness
      + w_traj * trajectory_smoothness
```

权重（默认）：

- error: 1.0
- runtime: 0.2
- control_smoothness: 0.5
- trajectory_smoothness: 0.5

约束：

- max_mean_runtime_ms: 500
- max_mean_ee_error: 0.15
- min_success_rate: 0.95

不满足约束的 variant 标记为 infeasible，score = inf。

## 6. 运行方式

```bash
python projects/B_mujoco_mpc_study/simulator/scripts/run_B03_sampling_solver_tuning.py \
  --config projects/B_mujoco_mpc_study/configs/B03_sampling_tuning.yaml \
  --run-id tuning_v1 \
  --save-figures --save-report
```

可选参数：

- `--solvers cem mppi_lite` — 只跑指定 solver family
- `--max-variants 2` — 每个 family 最多跑 N 个 variant（用于快速验证）
- `--num-steps 20 --horizon 8` — 覆盖仿真步数和 horizon（加速测试）
- `--seed 42` — 覆盖随机种子

## 7. 验收清单

- [x] 创建 `configs/B03_sampling_tuning.yaml`
- [x] 创建 `simulator/scripts/run_B03_sampling_solver_tuning.py`
- [x] 创建 `simulator/utils/sampling_tuning_reporter.py`
- [x] 创建 `tests/test_B03_sampling_tuning_reporter.py`
- [x] 更新 `B03_MPC_SOLVER_LADDER.md` 新增 Section 10
- [x] 更新 `README.md` 和 `docs/README.md`
- [x] 创建本管理文档
- [x] py_compile 通过
- [x] pytest 通过（12 新测试 + 26 已有测试 = 38 全部通过）
- [x] smoke run 通过（11 variants, 9/11 feasible）
- [x] 完整调参通过（run_id=tuning_v2, 最佳: mppi_temp_0p5, score=1.8850）
- [x] 调参结果已应用到 B03_mpc_solver_ladder.yaml（MPPI temp: 1.0→0.5, CEM early_stop 配置已添加）
- [x] B03 benchmark 验证通过（run_id=tuned_v1, 1000 步 lissajous, MPPI ctrl_smooth 从 ~2.8 降到 0.73）

## 8. 当前不实现内容

- 正式长仿真 benchmark（num_steps=1000+）
- iLQG / SQP / NMPC 参数调优
- 多 seed 敏感性测试
- full covariance CEM
- per-step / per-dim noise covariance for MPPI
- batched / parallel rollout

## 9. 下一步建议

- B03-R4：mini iLQR / iLQG learning skeleton
- 用推荐参数重跑 B03 benchmark
