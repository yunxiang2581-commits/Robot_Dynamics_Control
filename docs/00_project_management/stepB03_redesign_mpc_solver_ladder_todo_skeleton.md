# Step B03 redesign MPC solver ladder TODO skeleton

## 1. 为什么更改 B03 目标

旧 B03 的重点是 predictive sampling rollout 可解释性，这对理解 MPC 闭环很重要，但它还不足以支撑后续 B04 欠驱动系统、B05-B07 OpenLoong 人形 MPC 和 Project C 的更完整控制链学习。

因此 B03 现在升级为 `MPC Solver Ladder`：在简单模型中系统学习 sampling、CEM、MPPI、iLQG、SQP 和 direct multiple shooting NMPC 的统一接口、数据流、输出指标和可视化规划。

## 2. 旧 B03 rollout demo 如何并入新 B03

旧 B03 rollout predictive sampling 不删除，也不被否定。它现在被并入新 B03 的 Level 1/2 sampling solver 学习层，主要作为以下子功能保留：

- candidate rollout visualization
- selected rollout visualization
- cost distribution visualization

也就是说，旧 B03 不是废弃，而是成为新 B03 solver ladder 的 sampling 子模块。

## 3. 新 B03 solver ladder 分层

Level 1:
Random Shooting / Predictive Sampling

Level 2:
Warm-start Predictive Sampling

Level 3:
CEM-MPC

Level 4:
MPPI-lite

Level 5:
iLQG / iLQR-lite

Level 6:
SQP-MPC learning version

Level 7:
Direct Multiple Shooting NMPC skeleton

## 4. 本次新增/修改文件

新增：

- `projects/B_mujoco_mpc_study/docs/B03_mpc_solver_ladder_demo.md`
- `projects/B_mujoco_mpc_study/configs/B03_mpc_solver_ladder.yaml`
- `projects/B_mujoco_mpc_study/simulator/planners/mpc_solver_interface.py`
- `projects/B_mujoco_mpc_study/simulator/planners/sampling_mpc_solvers.py`
- `projects/B_mujoco_mpc_study/simulator/planners/ilqg_solver.py`
- `projects/B_mujoco_mpc_study/simulator/planners/nmpc_solvers.py`
- `projects/B_mujoco_mpc_study/simulator/utils/solver_benchmark_logger.py`
- `projects/B_mujoco_mpc_study/simulator/utils/solver_comparison_visualizer.py`
- `projects/B_mujoco_mpc_study/simulator/scripts/run_B03_mpc_solver_ladder_demo.py`
- `projects/B_mujoco_mpc_study/tests/test_B03_sampling_mpc_solvers.py`
- `projects/B_mujoco_mpc_study/tests/test_B03_mpc_solver_interface.py`
- `projects/B_mujoco_mpc_study/tests/test_B03_solver_benchmark_logger.py`
- `projects/B_mujoco_mpc_study/tests/test_B03_solver_comparison_visualizer.py`
- `docs/00_project_management/stepB03_redesign_mpc_solver_ladder_todo_skeleton.md`

保留旧 B03 子模块来源：

- `predictive_sampling_planner.py`
- `rollout_logger.py`
- `rollout_visualizer.py`
- `run_B03_rollout_predictive_sampling_demo.py`

## 5. 当前实现了哪些纯函数

- `sample_candidate_controls(...)`
- `shift_control_sequence(...)`
- `rank_rollouts(...)`
- `compute_control_smoothness(...)`
- `compute_trajectory_smoothness(...)`
- `SolverBenchmarkLogBuffer` 的基础 CSV IO

## 6. 哪些 solver 仍是 TODO

- `RandomShootingSolver`
- `WarmStartSamplingSolver`
- `CEMShootingSolver`
- `MPPILiteSolver`
- `ILQGLiteSolver`
- `SQPMPCSolver`
- `DirectMultipleShootingNMPCSolver`

这些 solver 当前都只保留函数级 learning skeleton，不在 B03-A 完整实现。

## 7. B03 与 B04 / B05-B07 的关系

- B03 负责学习 solver 方法层。
- B04 会把这些方法带入欠驱动小车倒立二阶摆。
- B05-B07 会把稳定下来的方法与接口迁移到 OpenLoong 人形 MPC。
- Project C 再在更完整的 `MPC target -> WBC-QP -> joint command / torque / PVT` 控制链中使用这些理解。

## 8. 下一步 B03-R1

B03-R1 建议先实现 sampling 系列核心逻辑：

1. Random Shooting
2. Warm-start Predictive Sampling
3. CEM-MPC
4. MPPI-lite 的纯函数核心

iLQG / SQP / full NMPC 继续保留为清晰骨架和学习文档，不在第一步强行完整实现。

## 9. 验收清单

- [x] B03 文档改为 MPC solver ladder
- [x] B03 config 包含 sampling / CEM / MPPI / iLQG / SQP / NMPC
- [x] 创建统一 solver interface
- [x] 创建 sampling solver 骨架
- [x] 创建 iLQG skeleton
- [x] 创建 SQP/NMPC skeleton
- [x] 创建 benchmark logger
- [x] 创建 solver comparison visualizer
- [x] 创建 B03 runner skeleton
- [x] 创建测试
- [x] 未修改 B02 benchmark / regression 配置
- [x] 未运行长时间仿真
- [x] 未生成正式 B03 MP4
- [x] 未执行 git add / commit / push
