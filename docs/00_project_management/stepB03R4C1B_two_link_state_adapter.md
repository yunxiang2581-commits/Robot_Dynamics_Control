# Step B03-R4C-1B: Two-Link State Adapter

## 本步目标

B03-R4C-1B 的目标是在保留 R4C TODO 骨架的前提下，完整实现 two-link env state adapter 的状态读写层。

本步只处理状态层 helper：

- `get_two_link_state(env)`
- `set_two_link_state(env, x)`
- `save_two_link_env_state(env)`
- `restore_two_link_env_state(env, snapshot)`

本步不完整实现 `dynamics_fn(x,u)->x_next`，不构造完整 `MPCProblem`，不运行 iLQR two-link 数值闭环。

## 与 B03-R4C-1A 的关系

B03-R4C-1A 已经定义了 adapter 契约和 shape validation：

- `build_two_link_dynamics_fn(env)`
- `build_state_tracking_problem(...)`
- `validate_state_shape(x)`
- `validate_control_shape(u)`
- `validate_nominal_trajectory_shapes(x_refs, u_nominal)`
- `ensure_output_dirs(output_dir)`

B03-R4C-1B 在这个基础上补齐“状态读写能力”。它仍然服务于后续 `build_two_link_dynamics_fn(env)`，但还不把 `dynamics_fn` 完整写出来。

## 为什么 iLQR 需要无副作用状态读写

iLQR-lite 需要反复做假想 rollout 和 finite-difference linearization。

例如有限差分会多次调用：

```text
f(x + eps, u)
f(x - eps, u)
f(x, u + eps)
f(x, u - eps)
```

这些调用都只是“假想未来”，不能污染真实闭环环境。因此后续 dynamics adapter 必须能：

1. 保存真实环境状态；
2. 临时设置环境到某个假设状态；
3. 执行一步控制；
4. 读取下一状态；
5. 恢复真实环境状态。

B03-R4C-1B 正是在为这件事打基础。

## B02 Two-Link 状态约定

当前 two-link state 固定为：

```text
x = [q1, q2, dq1, dq2]
```

其中：

- `q1, q2`: 两个关节角；
- `dq1, dq2`: 两个关节速度。

控制输入固定为：

```text
u = [tau1, tau2]
```

其中：

- `tau1, tau2`: 两个 actuator 的关节力矩。

## 四个 Helper 的职责

### get_two_link_state(env)

职责：

- 优先从 `env.get_state()` 读取状态；
- 兼容 dict 返回，例如 `"state"`、`"qpos"+"qvel"`、`"q"+"dq"`；
- 如果没有 `get_state()`，fallback 到 `env.data.qpos[:2]` 和 `env.data.qvel[:2]`；
- 返回独立 copy，避免调用者拿到 MuJoCo 内部 view；
- 检查 shape 必须是 `(4,)`；
- 检查所有值 finite。

### set_two_link_state(env, x)

职责：

- 检查 `x.shape == (4,)`；
- 检查 `x` finite；
- 优先调用 `env.set_state(x)`；
- 否则写入 `env.data.qpos[:2]` 和 `env.data.qvel[:2]`；
- 写入后调用 forward 刷新 MuJoCo 派生量；
- 不 reset simulation；
- 不 step 仿真。

### save_two_link_env_state(env)

职责：

- 调用 `get_two_link_state(env)` 保存标准 state；
- 如果存在 MuJoCo data，额外保存 `qpos`、`qvel`、`time`、`ctrl`；
- 所有 ndarray 都保存 copy；
- 返回 dict，便于兼容 fake env 和真实 MuJoCo env。

### restore_two_link_env_state(env, snapshot)

职责：

- 检查 snapshot 必须是 dict；
- 至少要求包含 `"state"`；
- 如果 snapshot 和 env 都有完整 `qpos/qvel`，优先恢复完整 MuJoCo 数组；
- 否则通过 `set_two_link_state(env, snapshot["state"])` 恢复标准状态；
- 恢复 `time` 和 `ctrl`；
- 恢复后 forward；
- 恢复后再次读取 state，检查 shape 和 finite。

## 本步完整实现了什么

本步完整实现：

- two-link state 读取；
- two-link state 写入；
- 环境状态 snapshot 保存；
- 环境状态 snapshot 恢复；
- MuJoCo/fake env 兼容 forward；
- 不依赖真实 MuJoCo 的 fake env 单元测试。

## 本步仍保留哪些 TODO

本步继续保留：

- `build_two_link_dynamics_fn(env)` 内部 `dynamics_fn(x,u)->x_next`；
- `build_state_tracking_problem(...)` 完整 `MPCProblem` 构造；
- task-space tracking；
- CEM/MPPI warm-start；
- trajectory plotting；
- cost history plotting；
- `m_target_interface`；
- `ik_interface`；
- `viewer_interface`；
- `actuator_interface`。

## 下一步 B03-R4C-1C

下一步建议在保留骨架前提下，实现最小 `dynamics_fn(x,u)->x_next`：

1. `snapshot = save_two_link_env_state(env)`；
2. `set_two_link_state(env, x)`；
3. `env.step(u)`；
4. `x_next = get_two_link_state(env)`；
5. `restore_two_link_env_state(env, snapshot)`；
6. `return x_next`。

这一步仍然应先用 fake env 测试无副作用，再考虑真实 MuJoCo optional smoke。

## 验证命令

```bash
python -m py_compile \
  projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py

pytest -q \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py \
  projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_state_adapter.py
```

## 未执行事项

- 未运行长时间 MuJoCo；
- 未生成 MP4；
- 未修改 B02 benchmark / regression；
- 未修改 B02 controller 核心逻辑；
- 未执行 `git add`；
- 未执行 `git commit`；
- 未执行 `git push`。
