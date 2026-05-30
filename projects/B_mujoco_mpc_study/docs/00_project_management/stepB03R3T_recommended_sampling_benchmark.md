# Step B03-R3T-Benchmark: Recommended Sampling Solver Configs

## 1. 本步目标

基于 B03-R3 和 B03-R3T 的调参结果，固定 B03 sampling-family 的推荐参数，用短 benchmark（100 步）重新比较四类 solver：

- random_shooting（baseline）
- warm_start_fast（fast）
- cem_default（balanced）
- mppi_smooth（smooth）

本步不是长时间正式 benchmark，不生成正式 MP4，不实现 iLQG / SQP / NMPC。

## 2. 为什么在 iLQG 前固定 sampling-family 默认配置

B03-R4 将实现 mini iLQR / iLQG learning skeleton。在此之前需要：

- 确认 sampling-family 各 solver 的推荐参数已经固化
- 用统一 benchmark 验证推荐配置的相对表现
- 为后续 iLQG 对比提供稳定的 sampling baseline
- 避免每次实验都重新调参

## 3. 推荐配置

| Solver Config | Family | num_candidates | 关键参数 | 定位 |
| --- | --- | --- | --- | --- |
| random_shooting | Random Shooting | 128 | sampling_std=2.0 | baseline |
| warm_start_fast | Warm-start | 96 | sampling_std=1.0, torque_rate_weight=0.01 | fast |
| cem_default | CEM | 128 | iterations=2, elite_ratio=0.10, early_stop | balanced |
| mppi_smooth | MPPI-lite | 128 | temperature=2.0, noise_std=1.5 | smooth |

## 4. 输出文件

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

## 5. 当前不实现内容

- 不实现 iLQG / SQP / NMPC
- 不修改 B02 benchmark / regression 配置
- 不修改 B02 controller 核心逻辑
- 不生成正式长视频
- 不执行 git add / commit / push

## 6. 下一步 B03-R4

- mini iLQR / iLQG learning skeleton
- iLQG 实现后，用推荐 sampling 配置作为 baseline 对比

## 7. 验收清单

- [x] 新增 B03_sampling_recommended_benchmark.yaml
- [x] runner 支持 --recommended-benchmark
- [x] 输出 recommended metrics
- [x] 输出 recommended figures
- [x] 输出 recommended report
- [x] 更新 B03 文档
- [x] 未修改 B02 benchmark 配置
- [x] 未修改 B02 regression 配置
- [x] 未修改 B02 controller 核心逻辑
- [x] 未生成正式 MP4
- [x] 未执行 git add / commit / push
