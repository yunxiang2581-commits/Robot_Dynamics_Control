# Step B03-R2A B02-to-B03 adapter

## 1. 为什么需要拆解 B02

B02 已经证明二连杆 task-space MPC tracking 可以稳定运行，但它主要是一个 baseline demo。B03 现在要系统学习不同 MPC solver，如果直接把 solver 逻辑塞进 B02 controller，会破坏边界，也会让 benchmark / regression 基线变得不稳定。

因此需要一个 adapter 层，把 B02 的任务、状态、target trajectory、rollout 和 cost 暴露成 B03 solver 能消费的统一协议。

## 2. B02 与 B03 的边界

- B02 提供任务和仿真事实。
- B03 提供 solver 家族。
- adapter 负责连接 B02 与 B03。

明确不做的事情：

- 不修改 B02 benchmark 配置。
- 不修改 B02 regression 配置。
- 不修改 B02 controller 核心逻辑。
- 不把可视化逻辑写进 controller。

## 3. adapter 新增文件

- `projects/B_mujoco_mpc_study/simulator/adapters/b02_to_b03_adapter.py`

这个文件当前包含：

- `B02TrackingSnapshot`
- `B02RolloutCostResult`
- `B02AdapterConfig`
- `extract_b02_tracking_snapshot(...)`
- `build_b03_problem_from_b02_snapshot(...)`
- `rollout_cost_candidates(...)`
- `make_b02_rollout_cost_fn(...)`
- `B02BaselineSolver`
- `wrap_b02_controller_as_solver(...)`

## 4. B02 task 如何映射到 MPCProblem

当前映射关系是：

- `snapshot.state -> problem.current_state`
- `target_horizon -> problem.target_horizon`
- `adapter_config.horizon -> problem.horizon`
- `adapter_config.control_dim -> problem.control_dim`
- `adapter_config.ee_weight / dq_weight / torque_weight / terminal_weight -> problem.cost_config`
- `rollout_cost_fn -> problem.rollout_cost_fn`

这样 B03 solver 不需要知道 MuJoCo site 查询、qpos/qvel 恢复或 B02 target trajectory 的细节。

## 5. rollout_cost_fn 协议

当前协议支持两种返回形式：

1. 返回 `np.ndarray costs`
2. 返回对象，至少包含：
   - `.costs`
   - `.predicted_states` optional
   - `.predicted_ee_positions` optional

Random Shooting / Warm-start Sampling 现在已经能消费这两类结果。

## 6. B02BaselineSolver 的用途

`B02BaselineSolver` 的目标不是重写 B02 controller，而是把现有 B02 baseline controller 包装成 B03 统一 solver 接口下的 comparison baseline。

这样在未来的 solver comparison 中，可以把：

- `b02_baseline`
- `random_shooting`
- `warm_start_sampling`
- `cem`
- `mppi_lite`

放到同一套 runner / logger / visualizer 下比较。

## 7. 当前未实现内容

- 还没有把 CEM / MPPI 正式接入 adapter rollout cost。
- 还没有把 iLQG / SQP / NMPC 接到真实 two-link rollout。
- 还没有运行正式 solver comparison loop。
- 还没有生成正式视频。

## 8. 下一步 B03-R2B

B03-R2B 建议优先做两件事：

1. 让 `CEMShootingSolver` 复用相同的 `rollout_cost_fn` 协议。
2. 在 B03 runner 中加入小规模 two-link comparison loop，只跑很少步数，验证 `b02_baseline / random_shooting / warm_start_sampling / cem` 的统一日志输出。

## 9. 验收清单

- [x] 新增 `b02_to_b03_adapter.py`
- [x] `MPCProblem` 支持 `rollout_cost_fn` 或等价协议
- [x] `RandomShootingSolver` 能消费 adapter rollout result
- [x] `WarmStartSamplingSolver` 能消费 adapter rollout result
- [x] 新增 adapter 测试
- [x] 未修改 B02 benchmark 配置
- [x] 未修改 B02 regression 配置
- [x] 未修改 B02 controller 核心逻辑
- [x] 未运行长时间 MuJoCo 仿真
- [x] 未生成正式 B03 MP4
- [x] 未执行 `git add / commit / push`
