# Step B03-R4C-2: Sampling Warm-Start for iLQR-lite

## 本步目标

B03-R4C-2 的目标是在 R4C joint-space state tracking smoke 基础上，接入 sampling-family warm-start：

```text
sampling solution.predicted_controls
-> shift_control_sequence(...)
-> iLQR-lite initial_controls
-> ILQGLiteSolver.solve(problem, previous_solution=...)
```

本步不改变 iLQR 的 backward pass、forward pass 或 cost 数学，只改变 iLQR 的初始控制序列来源。

## 为什么需要 warm-start

MPC 每次只执行当前最优控制序列的第一步。上一拍预测的未来第二步、第三步，通常可以作为下一拍的合理初值。因此 sampling solver 的 `predicted_controls` 可以用于给 iLQR-lite 一个更接近可行轨迹的 nominal control sequence。

本步使用与 sampling solver 一致的左移规则：

```text
u_init[:-1] = previous_controls[1:]
u_init[-1]  = previous_controls[-1]
```

## 本步实现内容

新增 `build_warm_start_controls_from_previous_solution(...)`：

1. 如果 `previous_solution` 缺失，返回 `None`；
2. 检查 `previous_solution.predicted_controls.shape == (horizon, control_dim)`；
3. 检查控制序列全为 finite；
4. 调用 `shift_control_sequence(...)` 得到 shifted warm-start controls。

更新 `run_smoke_simulation(...)`：

1. 新增 `previous_solution` 参数；
2. 优先使用 shifted warm-start controls 作为 `u_nominal`；
3. 如果没有 previous solution，则保持零控制初值；
4. 在 `problem.metadata` 中记录：
   - `warm_start_used`
   - `warm_start_source`
5. 调用 `solver.solve(problem, previous_solution=previous_solution)`。

更新输出：

- metrics CSV 新增 `warm_start_used` 和 `warm_start_source`；
- NPZ cache 新增 `initial_controls`；
- smoke report 新增 warm-start 状态。

## 验证命令

```bash
D:\anaconda\envs\mujoco_py311\python.exe -m py_compile \
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py

D:\anaconda\envs\mujoco_py311\python.exe -m pytest -q \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_state_adapter.py \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqg_solver.py
```

真实 MuJoCo warm-start smoke 使用 fake sampling solution 验证：

```text
success True
best_cost 1.1564752096542601e-07
warm_start_used True
warm_start_source fake_sampling_warm_start
horizon 32
initial_controls shape (32, 2)
predicted_controls shape (32, 2)
```

## 当前边界

本步只是把 sampling solution 的 `predicted_controls` 接入 iLQR-lite 初值。

当前仍不做：

- 自动先运行 CEM/MPPI 再调用 iLQR；
- sampling vs iLQR 对比 benchmark；
- task-space end-effector tracking；
- long benchmark；
- 正式 MP4；
- B02 controller 核心逻辑改写。

## 下一步建议

下一步可以做 **B03-R4C-2B: sampling-to-iLQR runner smoke**：

1. 用推荐 CEM 或 MPPI 配置生成一个真实 sampling solution；
2. 将其 `predicted_controls` 传给 R4C iLQR-lite；
3. 比较 zero-init iLQR 与 sampling-warm-start iLQR 的 cost/runtime。

也可以转向 **B03-R4D: task-space end-effector tracking**。
