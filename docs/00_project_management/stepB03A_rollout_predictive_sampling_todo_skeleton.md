# Step B03-A rollout predictive sampling TODO skeleton

## 1. 为什么进入 B03

B02 已经完成二连杆 task-space MPC tracking，并具备 tracking log、marked MP4、overlay / scene / hybrid 渲染、benchmark 和 regression 基线。下一步需要把 MPC 内部的 predictive sampling 过程拆开学习，而不是只看最终 tracking 曲线。

B03-A 先做 TODO skeleton，是为了在实现真实 rollout 前统一接口、日志 schema、可视化数据结构和测试边界。

## 2. B03 与 B02 的关系

B03 复用 B02 的二连杆模型和 task-space residual 思想，但不替代 B02。

本步骤不修改：

- B02 benchmark 配置。
- B02 regression 配置。
- B02 controller 核心逻辑。
- B02 的正式输出基线。

进入 B03 前，优先用 B02-regression-light 做健康检查；B02 benchmark 继续作为正式长期对照。

## 3. 本次新增文件

- `projects/B_mujoco_mpc_study/docs/B03_rollout_predictive_sampling_demo.md`
- `projects/B_mujoco_mpc_study/configs/B03_rollout_predictive_sampling.yaml`
- `projects/B_mujoco_mpc_study/simulator/planners/predictive_sampling_planner.py`
- `projects/B_mujoco_mpc_study/simulator/utils/rollout_logger.py`
- `projects/B_mujoco_mpc_study/simulator/utils/rollout_visualizer.py`
- `projects/B_mujoco_mpc_study/simulator/scripts/run_B03_rollout_predictive_sampling_demo.py`
- `projects/B_mujoco_mpc_study/tests/test_B03_predictive_sampling_planner.py`
- `projects/B_mujoco_mpc_study/tests/test_rollout_logger.py`
- `projects/B_mujoco_mpc_study/tests/test_rollout_visualizer.py`
- `docs/00_project_management/stepB03A_rollout_predictive_sampling_todo_skeleton.md`

## 4. Predictive sampling 数学数据流

```text
current state
  -> sample candidate control sequences
  -> rollout each sequence
  -> compute horizon cost
  -> rank candidates
  -> select best rollout
  -> execute first control
  -> repeat receding horizon
```

一次 planning step 可理解为：

```text
u_i[0:H-1] ~ N(0, sigma)
u_i = clip(u_i, -torque_limit, torque_limit)
x_i[0:H], p_i[0:H] = rollout(x_current, u_i)
J_i = task-space tracking cost
i* = argmin_i J_i
u_execute = u_i*[0]
```

## 5. 可视化输出规划

B03 后续完整可视化应区分：

- candidate rollouts：所有或部分候选预测轨迹。
- best rollout：当前 cost 最低的预测轨迹。
- actual trajectory：真实闭环已经执行出来的轨迹。
- target trajectory：当前目标和未来目标预览。
- cost distribution：当前候选 cost 的分布或排序。
- best cost time：每个控制周期的最小 cost 曲线。

B03-A 只实现 `build_rollout_marker_plan` 这类纯数据计划，不生成正式 B03 MP4。

## 6. 当前不实现内容

- 不运行长时间 MuJoCo 仿真。
- 不生成正式 B03 MP4。
- 不实现真实 MuJoCo data copy rollout。
- 不实现完整 B02 residual cost。
- 不实现 receding horizon 闭环。
- 不把 rollout 可视化逻辑写进 controller。
- 不接硬件、不做实物部署。

## 7. 下一步 B03-R1

B03-R1 建议只补两个当前纯函数之外的最小环节：

1. 实现单条 control sequence 的 MuJoCo-safe rollout，确保不会污染真实环境状态。
2. 实现 B02 task-space residual cost，并用小规模 fake env 或短 horizon 测试验证 cost 单调性。

仍然不建议在 B03-R1 直接追求正式视频交付。

## 8. 验收清单

- [x] B03 文档已创建。
- [x] B03 配置已创建，且不修改 B02 配置。
- [x] `sample_candidate_controls` 已实现并有纯函数测试。
- [x] `rank_rollouts` 已实现并有纯函数测试。
- [x] rollout logger 已创建并有 CSV 测试。
- [x] rollout visualizer marker plan 已创建并有纯函数测试。
- [x] B03 runner skeleton 已创建。
- [x] B03 仍保留真实 rollout / cost / receding horizon TODO。
- [x] 未运行长时间 MuJoCo 仿真。
- [x] 未生成正式 B03 MP4。
