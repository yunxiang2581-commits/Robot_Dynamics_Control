# Step B03-R4C-1C: Two-Link One-Step Dynamics Adapter

## 本步目标

B03-R4C-1C 的目标是在保留 R4C 学习型 TODO 骨架的前提下，实现最小 one-step dynamics adapter：

```text
dynamics_fn(x, u) -> x_next
```

其中 two-link 状态和控制约定仍然是：

```text
x = [q1, q2, dq1, dq2]
u = [tau1, tau2]
```

本步不构造完整 `MPCProblem`，不运行 two-link iLQR 数值闭环，不生成正式 MP4。

## 为什么需要 dynamics_fn

B03-R4B 已经实现 mini iLQR-lite 的核心循环。iLQR 在 nominal rollout 和 finite-difference linearization 中会反复查询：

```text
x_next = f(x, u)
```

对 B02 two-link 来说，`f(x,u)` 来自 MuJoCo 环境的一步仿真。因此 adapter 必须能临时把环境设置到假想状态、执行一步假想控制、读取下一状态，并恢复真实环境。

## 本步实现内容

本步实现 `build_two_link_dynamics_fn(env)`：

1. 检查 `env.step(torque)` 是否存在；
2. 返回内部函数 `dynamics_fn(x, u)`；
3. 在每次调用时检查 `x.shape == (4,)` 和 `u.shape == (2,)`；
4. 使用 `save_two_link_env_state(env)` 保存真实环境；
5. 使用 `set_two_link_state(env, x)` 设置临时状态；
6. 调用 `env.step(u)` 执行一步临时仿真；
7. 使用 `get_two_link_state(env)` 读取 `x_next`；
8. 在 `finally` 中调用 `restore_two_link_env_state(env, snapshot)` 恢复环境。

同时扩展 snapshot，保存和恢复 `last_applied_torque`，避免临时 rollout 污染真实 `TwoLinkEnv.step()` 记录的上一拍实际力矩。

## 测试覆盖

新增 fake env 测试覆盖：

- `dynamics_fn(x,u)` 的输出来自临时 one-step dynamics；
- 调用后真实 env 的 `state/qpos/qvel/time/ctrl` 恢复；
- 调用后真实 env 的 `last_applied_torque` 恢复；
- 无 `step()` 的环境会给出清晰 `NotImplementedError`。

## 验证命令

```bash
python -m py_compile \
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py

pytest -q \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_state_adapter.py
```

当前环境没有安装 `mujoco`，因此真实 `TwoLinkEnv` smoke 在 `setup_environment()` 阶段被阻止；fake env 和 import-safe 测试已覆盖 adapter 契约。

## 当前仍保留的 TODO

本步继续保留：

- `build_state_tracking_problem(...)` 完整 `MPCProblem` 构造；
- state tracking smoke 数值运行；
- task-space tracking cost；
- CEM/MPPI warm-start；
- trajectory plotting；
- cost history plotting；
- SQP / full NMPC；
- 正式 MP4。

## 下一步 B03-R4C-1D

建议下一步实现最小 state tracking problem builder：

1. 构造 `Q/R/Q_terminal`；
2. 把 `x_refs`、`initial_controls` 和 `dynamics_fn` 放入 `MPCProblem`；
3. 用短 horizon 和小 iteration 调用 `ILQGLiteSolver.solve(problem)`；
4. 保存 cost history CSV；
5. 仍然先用 fake/simple dynamics 验证，再考虑真实 MuJoCo optional smoke。
