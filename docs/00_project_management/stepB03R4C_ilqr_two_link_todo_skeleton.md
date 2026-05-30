# Step B03-R4C: iLQR Two-Link TODO Skeleton

## 任务目标

本步目标是生成 B03-R4C 的学习型 TODO 骨架，把 B03-R4B 已经实现的 `ILQGLiteSolver` 从 toy double-integrator 过渡到 B02 two-link dynamics 适配。

当前只搭建结构，不实现真实 two-link iLQR rollout。后续实现应沿着：

1. B02 two-link 当前状态读取；
2. one-step dynamics adapter `x_{k+1}=f(x_k,u_k)`；
3. state tracking quadratic cost；
4. `ILQGLiteSolver.solve(problem)`；
5. smoke metrics / figures；
6. task-space tracking；
7. sampling-family warm-start。

## B02 Two-Link 适配说明

B02 two-link 环境的状态约定先使用：

```text
x = [q1, q2, dq1, dq2]
u = [tau1, tau2]
```

B03-R4C 后续需要把 `TwoLinkEnv` 封装成 iLQR 可以调用的 deterministic dynamics：

```python
def dynamics_fn(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    # TODO:
    # 1. 保存真实 env state；
    # 2. env.set_state(x)；
    # 3. env.step(u)；
    # 4. 读取 x_next；
    # 5. 恢复真实 env state；
    # 6. return x_next。
```

这个 adapter 是 B03-R4C 的核心，因为 iLQR 需要反复 rollout 和 finite-difference linearization，不能污染真实闭环环境状态。

## TODO 列表

- State tracking smoke run:
  先实现关节空间状态跟踪，验证 `x_refs`、`Q`、`R`、`Q_terminal` 能让 `ILQGLiteSolver.solve()` 返回有限轨迹。

- Task-space tracking TODO:
  后续接入末端位置 `p_ee(q)`，把 B02 的末端 tracking 目标转换为 cost residual。第一版可以先在外层构造近似 cost，暂不进入 full DDP。

- CEM/MPPI warm-start TODO:
  后续可先运行 B03 sampling-family solver，把 `predicted_controls` 作为 `previous_solution` 传入 iLQR-lite，用 shifted controls 做 warm start。

- 绘图 TODO:
  后续实现 `plot_trajectory()` 和 `plot_cost_history()`，输出 state trajectory、task-space trajectory 和 cost history 图像。

## 输出文件规划

脚本默认输出目录：

```text
projects/B_mujoco_mpc_study/outputs/runs/B03_ilqr_lite_two_link_smoke/<timestamp>/outputs/
```

子目录规划：

```text
outputs/cache/
outputs/figures/
outputs/reports/
outputs/metrics/
```

建议文件名：

```text
outputs/cache/B03_R4C_two_link_state_tracking_placeholder.npz
outputs/figures/B03_R4C_two_link_state_trajectory_todo.png
outputs/figures/B03_R4C_ilqr_cost_history_todo.png
outputs/reports/B03_R4C_ilqr_two_link_todo_report.md
outputs/metrics/B03_R4C_ilqr_two_link_smoke_metrics.csv
```

## 验证 / Smoke Run 标准

当前骨架验收标准：

- `python -m py_compile projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py` 通过；
- 脚本中保留 `setup_environment(config_path) -> tuple[TwoLinkEnv, np.ndarray]`；
- 脚本中保留 `setup_solver(config_path) -> ILQGLiteSolver`；
- 脚本中保留 `run_smoke_simulation(env, solver, x_ref) -> MPCSolution`；
- 脚本中保留 `plot_trajectory(solution, x_ref, output_dir) -> None`；
- 脚本中保留 `plot_cost_history(solution, output_dir) -> None`；
- `run_smoke_simulation()` 中保留 `ILQGLiteSolver.solve(problem)` 调用位置；
- 未实现位置抛出清晰 `NotImplementedError`；
- 不运行长时间 MuJoCo；
- 不生成正式 MP4；
- 不修改 B02 benchmark / regression 配置；
- 不修改 B02 controller 核心逻辑；
- 不执行 `git add`、`git commit`、`git push`。

后续 R4C 真正 smoke run 的最低标准：

- horizon 短，例如 8 到 32；
- max iterations 小，例如 3 到 10；
- 输出有限的 `predicted_states` 和 `predicted_controls`；
- `best_cost` 有限；
- cost history 可保存；
- 若 line search 失败，也要返回清晰 failure message 和当前有限轨迹。

## 当前不实现内容

本步明确不实现：

- stochastic iLQG；
- full DDP 二阶动力学项；
- constraints / SQP / full NMPC；
- OSQP / IPOPT / acados 接入；
- B02 controller 重写；
- B02 benchmark / regression 配置改动；
- 长时间 MuJoCo 仿真；
- 正式 MP4 生成；
- 硬件部署；
- sim2real。

## 下一步建议

B03-R4C 后续可以按以下顺序补：

1. 实现 `dynamics_fn(x, u) -> x_next` adapter，并测试 rollout 不污染真实 env；
2. 构造最小 state tracking `MPCProblem`；
3. 用小 horizon 跑 `ILQGLiteSolver.solve()`；
4. 保存 cost history CSV；
5. 实现 state trajectory 和 cost history 图；
6. 再接入 task-space tracking；
7. 最后研究 CEM/MPPI warm-start。
