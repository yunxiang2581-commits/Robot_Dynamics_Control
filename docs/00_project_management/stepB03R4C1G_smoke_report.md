# Step B03-R4C-1G: Smoke Report

## 本步目标

B03-R4C-1G 的目标是在 R4C-1E/1F 已经生成 metrics、cache 和 figures 的基础上，补齐最小 Markdown smoke report。

本步仍然只做短 horizon joint-space state tracking smoke，不做 task-space tracking，不生成正式 MP4，不修改 B02 controller 或 B02 benchmark/regression 配置。

## 本步新增输出

脚本现在会在 reports 目录写入：

```text
outputs/reports/B03_R4C_ilqr_two_link_smoke_report.md
```

报告包含：

- 当前 smoke 的用途说明；
- solver success/message/termination_reason；
- horizon、state_dim、control_dim；
- best_cost、initial_cost、final_cost；
- runtime、rollout 数、iteration 数；
- max_abs_control、final_state_error_norm；
- metrics/cost history/cache/figures 的相对路径；
- 当前 scope 边界。

## 本步实现内容

新增 `write_smoke_report(solution, problem, output_paths)`：

1. 从 `solution` 读取 solver stats 和 cost history；
2. 从 `problem` 读取 horizon、control_dim 和 x_refs；
3. 计算 `final_state_error_norm` 和 `max_abs_control`；
4. 汇总 output 文件路径；
5. 写入 Markdown report。

同时新增 `_relative_output_path(...)`，让 report 内路径相对于 run directory，方便阅读和移动。

## 验证命令

```bash
D:\anaconda\envs\mujoco_py311\python.exe -m py_compile \
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py

D:\anaconda\envs\mujoco_py311\python.exe -m pytest -q \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_state_adapter.py \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqg_solver.py
```

真实 MuJoCo smoke：

```bash
D:\anaconda\envs\mujoco_py311\python.exe \
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py \
  --output-dir outputs/pytest_tmp/B03_R4C1G_real_smoke \
  --log-level INFO
```

示例 report 摘要：

```text
success: True
horizon: 32
best_cost: 1.54493724877e-07
initial_cost: 2.51671674996e-07
final_cost: 1.54493724877e-07
final_state_error_norm: 4.22653463389e-05
```

## 当前 R4C 状态

R4C 当前已经具备：

```text
TwoLinkEnv
-> state save/restore
-> dynamics_fn(x,u)
-> state tracking MPCProblem
-> ILQGLiteSolver.solve(problem)
-> metrics/cache/figures/report
```

这意味着 B03-R4C 已从 TODO skeleton 推进到可运行、可复盘的 joint-space iLQR-lite two-link smoke。

## 当前仍保留的 TODO

- 文件名仍有 `_todo`，因为 R4C 仍是 smoke 学习线；
- 还没有 task-space end-effector tracking；
- 还没有 CEM/MPPI warm-start；
- 还没有 long benchmark；
- 还没有正式 MP4；
- 还没有把 R4C 当前状态同步回 B03 主文档。

## 下一步建议

建议下一步做 **B03-R4C-1H: Sync R4C Status Docs**：

1. 更新 `B03_MPC_SOLVER_LADDER.md` 的 R4C 状态；
2. 更新 Project B README / docs index 中的当前入口；
3. 明确下一阶段是 task-space tracking 还是 sampling warm-start。
