# Project B 文档索引

本目录保留精简合并后的 Project B 文档。原则是：

- `00_overview/` 只放项目总览。
- 每个任务目录只保留 1 份主文档。
- 临时实现计划、重复验收说明、旧定位文档已合并进对应主文档。

## 主文档

- [PROJECT_B_OVERVIEW.md](00_overview/PROJECT_B_OVERVIEW.md)
- [B01_SINGLE_JOINT_MPC.md](B01_single_joint_mpc/B01_SINGLE_JOINT_MPC.md)
- [B02_TWO_LINK_MPC_TRACKING.md](B02_two_link_mpc_tracking/B02_TWO_LINK_MPC_TRACKING.md)
- [B03_MPC_SOLVER_LADDER.md](B03_mpc_solver_ladder/B03_MPC_SOLVER_LADDER.md)
- [B02_TO_B03_ADAPTER.md](B03_mpc_solver_ladder/B02_TO_B03_ADAPTER.md)

## 步骤管理文档

- [stepB03R3_sampling_solver_benchmark_stabilization.md](00_project_management/stepB03R3_sampling_solver_benchmark_stabilization.md)
- [stepB03R3T_sampling_solver_automated_tuning.md](00_project_management/stepB03R3T_sampling_solver_automated_tuning.md)
- [stepB03R3T_recommended_sampling_benchmark.md](00_project_management/stepB03R3T_recommended_sampling_benchmark.md)
- B03-R4B root management note: [docs/00_project_management/stepB03R4B_mini_ilqr_core_loop.md](../../../docs/00_project_management/stepB03R4B_mini_ilqr_core_loop.md)
- B03-R4C-1C dynamics adapter note: [docs/00_project_management/stepB03R4C1C_two_link_dynamics_adapter.md](../../../docs/00_project_management/stepB03R4C1C_two_link_dynamics_adapter.md)
- B03-R4C-1D state tracking problem note: [docs/00_project_management/stepB03R4C1D_state_tracking_problem.md](../../../docs/00_project_management/stepB03R4C1D_state_tracking_problem.md)
- B03-R4C-1E smoke outputs note: [docs/00_project_management/stepB03R4C1E_smoke_outputs.md](../../../docs/00_project_management/stepB03R4C1E_smoke_outputs.md)
- B03-R4C-1F smoke figures note: [docs/00_project_management/stepB03R4C1F_smoke_figures.md](../../../docs/00_project_management/stepB03R4C1F_smoke_figures.md)
- B03-R4C-1G smoke report note: [docs/00_project_management/stepB03R4C1G_smoke_report.md](../../../docs/00_project_management/stepB03R4C1G_smoke_report.md)

## 当前重点

- B02 已完成二连杆 MPC tracking，并保留 benchmark / regression 两套基线。
- B03 已重新定义为 MPC solver ladder demo。
- B03-R4C 已完成 mini iLQR-lite two-link joint-space state tracking smoke，具备 metrics / cache / figures / report 输出。
- B03 不替代 B02；进入 B03 前优先使用 B02-regression-light 做健康检查。
- B02 到 B03 的 adapter 和新 planner 接入方式，单独记录在 adapter 文档里。
