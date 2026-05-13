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

## 当前重点

- B02 已完成二连杆 MPC tracking，并保留 benchmark / regression 两套基线。
- B03 已重新定义为 MPC solver ladder demo。
- B03 不替代 B02；进入 B03 前优先使用 B02-regression-light 做健康检查。
