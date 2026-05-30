# Step B03-R4C-1D: Minimal State Tracking MPCProblem

## 本步目标

B03-R4C-1D 的目标是在 B03-R4C-1C 的 one-step dynamics adapter 基础上，构造最小可运行的 state tracking `MPCProblem`：

```text
x0 + x_refs + u_nominal + dynamics_fn + Q/R/Q_terminal -> MPCProblem
```

状态和控制约定保持不变：

```text
x = [q1, q2, dq1, dq2]
u = [tau1, tau2]
```

本步只做关节空间 state tracking，不实现 task-space 末端位置 cost，不生成正式 MP4，不修改 B02 controller。

## 为什么需要这一步

B03-R4B 的 `ILQGLiteSolver` 已经能消费统一的 `MPCProblem`，但 two-link 侧还缺少把 B02 环境、参考状态、初始控制序列和二次 cost 权重打包成 problem 的桥接层。

本步补上的就是这个桥接层。它让后续代码可以直接调用：

```python
problem = build_state_tracking_problem(...)
solution = ILQGLiteSolver().solve(problem)
```

## 本步实现内容

本步实现 `build_state_tracking_problem(...)`：

1. 检查 `x0.shape == (4,)`；
2. 检查 `x_refs.shape == (H+1, 4)`；
3. 检查 `u_nominal.shape == (H, 2)`；
4. 检查 `dynamics_fn` 可调用；
5. 从 `simulation.dt` 或 `env.dt` 读取 `dt`；
6. 从 `cost` 配置构造二维矩阵 `Q/R/Q_terminal`；
7. 从 `ilqg_lite` 配置复制 solver 参数；
8. 返回统一 `MPCProblem`，并在 `metadata` 中保存：
   - `x_refs`
   - `initial_controls`
   - `Q`
   - `R`
   - `Q_terminal`
   - `dynamics_fn`
   - state/control convention 说明

同时新增 `_diagonal_weight_matrix(...)` 和 `_build_state_tracking_cost_matrices(...)`，把教学 YAML 中常见的标量、对角列表或二维矩阵转换成 iLQR solver 需要的二维方阵。

## 测试覆盖

新增/更新测试覆盖：

- `build_state_tracking_problem(...)` 返回合法 `MPCProblem`；
- `Q/R/Q_terminal` 能从一维权重列表转换为对角矩阵；
- `initial_controls` 和 `x_refs` 被复制进 metadata；
- 生成的 problem 能被 `ILQGLiteSolver.solve(problem)` 消费；
- R4C-1C 的 dynamics adapter 无副作用测试继续通过。

## 验证命令

```bash
D:\anaconda\envs\mujoco_py311\python.exe -m py_compile \
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py

D:\anaconda\envs\mujoco_py311\python.exe -m pytest -q \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_state_adapter.py

D:\anaconda\envs\mujoco_py311\python.exe -m pytest -q \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqg_solver.py
```

真实 MuJoCo 短 horizon smoke：

```text
problem_horizon 4
predicted_states_shape (5, 4)
predicted_controls_shape (4, 2)
success True
state_restored True
time_restored True
ctrl_restored True
```

## 当前仍保留的 TODO

本步继续保留：

- 更正式的 state tracking smoke runner 输出 CSV；
- trajectory plotting；
- cost history plotting；
- task-space end-effector tracking cost；
- CEM/MPPI warm-start；
- long benchmark；
- 正式 MP4；
- SQP / full NMPC。

## 下一步 B03-R4C-1E

建议下一步实现最小 smoke 输出链路：

1. 在 `run_smoke_simulation(...)` 中传入真实 config；
2. 用短 horizon / 小 iteration 跑一次 state tracking solve；
3. 保存 `cost_history.csv` 和一个最小 metrics CSV；
4. 只输出文本/CSV，不进入正式视频和长 benchmark。
