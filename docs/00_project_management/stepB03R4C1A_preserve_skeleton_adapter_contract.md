# Step B03-R4C-1A: Preserve Skeleton and Add Adapter Contract

## 本步目标

B03-R4C-1A 的目标是保留 `run_B03_ilqr_lite_two_link_smoke.py` 的 TODO 骨架，只补最小的 two-link dynamics adapter 契约与 state tracking smoke 占位。

本步不是完整实现 B03-R4C，也不是把 TODO 文件改成完整算法文件。当前仍以教学骨架为主，让后续实现者清楚知道：

- one-step dynamics adapter 应该放在哪里；
- state tracking `MPCProblem` 应该由哪些字段组成；
- `ILQGLiteSolver.solve(problem)` 应该在哪里被调用；
- 哪些接口仍然只是 TODO。

## 为什么先定义 dynamics_fn 契约

iLQR-lite 的核心循环需要反复调用离散动力学：

```text
x_next = f(x, u)
```

在 toy double-integrator 中，`f(x,u)` 可以直接手写；但在 B02 two-link 任务中，下一状态来自 MuJoCo 环境。因此必须先定义一个清晰 adapter 契约：

```python
def build_two_link_dynamics_fn(env):
    """
    返回 dynamics_fn(x, u) -> x_next。
    TODO:
    - 保存 env 当前状态；
    - 设置 env 到输入 x；
    - 执行一步控制 u；
    - 读取 x_next；
    - 恢复 env 原始状态；
    - 返回 x_next。
    """
```

这个契约的关键不是马上实现所有 MuJoCo 细节，而是先固定输入输出边界。这样后续补实现时，不会把 MuJoCo 状态管理、iLQR solver 内部逻辑和可视化逻辑混在一起。

## iLQR-lite 为什么需要 x_next = f(x, u)

B03-R4B 的 mini iLQR-lite 依赖以下步骤：

1. nominal rollout：沿着当前控制序列反复计算 `x_next = f(x,u)`；
2. trajectory linearization：在 nominal trajectory 周围用有限差分估计 `A=df/dx` 和 `B=df/du`；
3. backward pass：基于线性化动力学和二次 cost 计算反馈增益；
4. forward pass：用新的控制律重新 rollout，并做 line search。

如果没有稳定的 `dynamics_fn(x,u)` 契约，iLQR-lite 就无法在 two-link 环境上做 rollout 和 finite-difference linearization。

## State Tracking MPCProblem 后续字段

后续 `build_state_tracking_problem(...)` 至少需要构造：

- `current_state`: 当前状态 `x0`，shape `(4,)`；
- `target_horizon`: state reference，占位可用 `x_refs`；
- `horizon`: `H`；
- `control_dim`: two-link torque 控制维度，当前为 `2`；
- `dt`: 环境步长；
- `dynamics_fn`: one-step adapter；
- `metadata["x_refs"]`: shape `(H+1, 4)`；
- `metadata["Q"]`: state tracking 权重；
- `metadata["R"]`: control regularization 权重；
- `metadata["Q_terminal"]`: terminal state 权重；
- `metadata["initial_controls"]`: nominal / warm-start 控制序列，shape `(H, 2)`。

本步只实现 shape validation，不实现真实 cost 权重选择和数值求解。

## 当前保留的 TODO

脚本继续保留：

- `build_two_link_dynamics_fn(env)` 的真实 MuJoCo 状态保存、设置、step、恢复逻辑；
- `build_state_tracking_problem(...)` 的真实 `MPCProblem` 构造；
- state tracking smoke run 的完整数值运行；
- task-space tracking；
- CEM/MPPI warm-start；
- trajectory plotting；
- cost history plotting；
- `m_target_interface`；
- `ik_interface`；
- `viewer_interface`；
- `actuator_interface`。

## 本步新增的最小契约

本步新增或整理：

- `validate_state_shape(x)`；
- `validate_control_shape(u)`；
- `validate_nominal_trajectory_shapes(x_refs, u_nominal)`；
- `ensure_output_dirs(output_dir)`；
- `build_two_link_dynamics_fn(env)`；
- `build_state_tracking_problem(env, dynamics_fn, x0, x_refs, u_nominal, config)`。

其中 shape validation 和输出目录创建是真实实现，因为它们不是核心控制算法；dynamics adapter 和 problem builder 仍保留 `NotImplementedError`。

## 验证命令

```bash
python -m py_compile \
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py

pytest -q \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py
```

## 不实现内容

本步明确不实现：

- stochastic iLQG；
- full DDP 二阶动力学项；
- full two-link iLQR 数值求解；
- task-space tracking；
- CEM / MPPI warm-start；
- SQP / full NMPC；
- OSQP / IPOPT / acados；
- B02 controller 核心逻辑修改；
- B02 benchmark / regression 配置修改；
- 长时间 MuJoCo 仿真；
- 正式 MP4；
- 硬件部署；
- sim2real。

## 下一步

下一步才进入真实 `dynamics_fn(x,u)` 的局部实现。建议先只实现并测试：

1. 保存 env 状态；
2. 设置 env 到 `x`；
3. 执行一步 `u`；
4. 读取 `x_next`；
5. 恢复 env 状态；
6. 验证 adapter 不污染真实环境。
