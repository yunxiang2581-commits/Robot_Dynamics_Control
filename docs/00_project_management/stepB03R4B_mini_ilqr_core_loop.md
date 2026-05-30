# Step B03-R4B mini iLQR Core Loop

## 1. 本步目标

本步把 `B03_mpc_solver_ladder_demo` 中的 iLQR / iLQG-lite learning skeleton 推进到第一版最小可运行核心闭环。当前目标不是工业级 NMPC，而是让学习者看清：

```text
nominal rollout
-> trajectory linearization
-> trajectory cost quadratization
-> backward pass
-> forward pass with line search
-> MPCSolution
```

## 2. iLQR 与 LQR / CEM / MPPI 的关系

- LQR：假设动力学线性、cost 二次，可以一次反向递推得到反馈律。
- iLQR-lite：对非线性离散 dynamics 沿 nominal trajectory 做局部线性化，对 tracking cost 写成二次项，然后反复求局部 LQ 子问题。
- CEM / MPPI：通过采样搜索控制序列，不要求显式 Jacobian。
- 本步 iLQR-lite：从 sampling-family 进入 gradient / local-model family，但仍保持 toy / external dynamics，不替换 B03 sampling 推荐结果。

## 3. 本次实现的数学流程

1. 从 `initial_state` 和 `controls` rollout 得到 `states[0:H]`、stage costs 和 terminal cost。
2. 对每一步 `x_next = f(x, u)` 用中心差分得到 `A_t = df/dx`、`B_t = df/du`。
3. 对 tracking cost 写出：

```text
l_x = Q (x - x_ref)
l_u = R u
l_xx = Q
l_uu = R
l_ux = 0
terminal_x = Q_terminal (x_H - x_ref_H)
```

4. backward pass 从 terminal value 反向递推，求每一步 `k` 和 `K`。
5. forward pass 使用：

```text
u_new[t] = u_bar[t] + alpha * k[t] + K[t] @ (x_new[t] - x_bar[t])
```

6. line search 只接受 cost 下降的候选轨迹。

## 4. 新增/修改文件

- `projects/B_mujoco_mpc_study/simulator/planners/ilqg_solver.py`
- `projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqg_lite_demo.py`
- `projects/B_mujoco_mpc_study/configs/B03_ilqg_lite.yaml`
- `projects/B_mujoco_mpc_study/tests/test_B03_ilqg_solver.py`
- `projects/B_mujoco_mpc_study/docs/B03_mpc_solver_ladder/B03_MPC_SOLVER_LADDER.md`
- `projects/B_mujoco_mpc_study/README.md`
- `projects/B_mujoco_mpc_study/docs/README.md`
- `docs/00_project_management/stepB03R4B_mini_ilqr_core_loop.md`

## 5. 已实现函数

- `finite_difference_jacobian(...)`
- `linearize_discrete_dynamics(...)`
- `rollout_nominal_trajectory(...)`
- `linearize_trajectory_dynamics(...)`
- `quadratize_trajectory_cost(...)`
- `backward_pass(...)`
- `forward_pass(...)`
- `ILQGLiteSolver.solve(...)`

## 6. 当前不实现内容

- stochastic iLQG
- full DDP 二阶动力学项
- constraints / SQP / full NMPC
- OSQP / IPOPT / acados
- B02 controller 内部 iLQR 改写
- 长时间 MuJoCo 仿真
- 正式 MP4
- 硬件部署 / sim2real

## 7. Toy Dynamics Smoke Demo

toy demo 使用 double-integrator：

```text
x = [position, velocity]
u = [acceleration]
x_next[0] = x[0] + dt * x[1]
x_next[1] = x[1] + dt * u[0]
```

运行命令：

```bash
python projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqg_lite_demo.py \
  --config projects/B_mujoco_mpc_study/configs/B03_ilqg_lite.yaml \
  --run-id ilqg_toy_smoke \
  --model-family toy \
  --horizon 32 \
  --max-iterations 10 \
  --save-report
```

输出：

- `metrics/B03_ilqg_lite_cost_history.csv`
- `reports/B03_ilqg_lite_toy_smoke_report.md`

## 8. 下一步 B03-R4C

B03-R4C 可以在不修改 B02 controller 的前提下，把 B02 two-link dynamics / adapter 作为 external dynamics 接到 `ILQGLiteSolver`，并继续保持可视化、benchmark 和 controller 解耦。

## 9. 验收清单

- [x] 实现 `rollout_nominal_trajectory`
- [x] 实现 `linearize_trajectory_dynamics`
- [x] 实现 `quadratize_trajectory_cost`
- [x] 实现 `backward_pass`
- [x] 实现 `forward_pass`
- [x] 实现 `ILQGLiteSolver.solve` 最小版本
- [x] runner 支持 toy dynamics smoke demo
- [x] 新增/更新测试
- [x] 未修改 B02 benchmark 配置
- [x] 未修改 B02 regression 配置
- [x] 未修改 B02 controller 核心逻辑
- [x] 未运行长时间 MuJoCo
- [x] 未生成正式 MP4
- [x] 未执行 git add / commit / push
