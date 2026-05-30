# Step B03-R4C-1H: Sync R4C Status Docs

## 本步目标

B03-R4C-1H 的目标是把 R4C-1C 到 R4C-1G 的当前完成状态同步回 Project B 主文档和 B03 主文档。

本步只改文档，不改算法代码，不改 B02 controller，不运行长仿真，不生成正式 MP4。

## 同步内容

已更新：

- `projects/B_mujoco_mpc_study/README.md`
- `projects/B_mujoco_mpc_study/docs/README.md`
- `projects/B_mujoco_mpc_study/docs/B03_mpc_solver_ladder/B03_MPC_SOLVER_LADDER.md`

同步后的 R4C 状态：

```text
TwoLinkEnv
-> get/set/save/restore state
-> dynamics_fn(x,u) -> x_next
-> build_state_tracking_problem(...)
-> ILQGLiteSolver.solve(problem)
-> metrics / cache / figures / report
```

## 当前 R4C 能力

R4C 当前已经具备 mini iLQR-lite two-link joint-space state tracking smoke：

- one-step MuJoCo dynamics adapter；
- state tracking `MPCProblem`；
- `ILQGLiteSolver.solve(problem)`；
- cost history CSV；
- metrics CSV；
- NPZ cache；
- state tracking figure；
- cost history figure；
- Markdown smoke report。

示例 smoke 指标：

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

## 当前边界

R4C 当前仍不包含：

- task-space end-effector tracking；
- CEM / MPPI warm-start；
- long benchmark；
- 正式 MP4；
- SQP / full NMPC；
- B02 controller 核心逻辑改写。

## 验证方式

本步为文档同步，验证重点是：

1. README 提到 R4C 当前状态和运行入口；
2. docs index 链接到 R4C-1C 到 1G 管理记录；
3. B03 主文档包含 R4C 完成链条、输出文件、示例指标和边界。

## 下一步建议

下一阶段有两个合理方向：

1. **B03-R4D: task-space end-effector tracking**，把当前 joint-space state tracking 推向 B02 原始任务空间误差；
2. **B03-R4C-2: sampling warm-start for iLQR-lite**，把 CEM/MPPI 的 `predicted_controls` 作为 iLQR-lite 初始控制序列。
