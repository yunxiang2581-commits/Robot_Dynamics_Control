# Project B: MuJoCo MPC 学习区

Project B 是本仓库的 MuJoCo MPC / MJPC learning line。它只做 simulation-only 学习与复现，不做实物部署、sim2real、电机 SDK、CAN / EtherCAT / 串口通信或真实机器人安全测试。

核心路线：

```text
B01-B03: small-model MPC concept validation
B04: underactuated bridge task
B05-B07: OpenLoong humanoid MPC line
```

当前重点：

- B02 已完成二连杆 task-space MPC tracking，并保留 benchmark / regression 两套基线。
- B03 已重新定义为 MPC solver ladder demo。
- B03 不替代 B02；进入 B03 前优先跑 B02-regression-light 做健康检查。

## 文档入口

文档已精简为 4 份主文档：

- [Project B 总览](docs/00_overview/PROJECT_B_OVERVIEW.md)
- [B01 单关节 MPC](docs/B01_single_joint_mpc/B01_SINGLE_JOINT_MPC.md)
- [B02 双连杆 MPC Tracking](docs/B02_two_link_mpc_tracking/B02_TWO_LINK_MPC_TRACKING.md)
- [B03 MPC Solver Ladder](docs/B03_mpc_solver_ladder/B03_MPC_SOLVER_LADDER.md)

总索引：

- [docs/README.md](docs/README.md)

## 输出规则

每个 demo 必须输出：

```text
video + figures + metrics + logs
```

没有可视化输出的任务不算完成；没有 metrics 的视频不算完成；没有复现实验命令的结果不算完成。

统一输出目录：

```text
outputs/runs/<task_name>/<run_id>/
```

## 代码入口

主要目录：

```text
configs/
simulator/models/
simulator/envs/
simulator/planners/
simulator/controllers/
simulator/scripts/
simulator/utils/
tests/
```

当前常用脚本：

```text
simulator/scripts/run_B01_single_joint_mpc_demo.py
simulator/scripts/run_B02_two_link_mpc_tracking_demo.py
simulator/scripts/run_B03_mpc_solver_ladder_demo.py
```

## 运行提示

B02 正式 benchmark：

```bash
python projects/B_mujoco_mpc_study/simulator/scripts/run_B02_two_link_mpc_tracking_demo.py --config projects/B_mujoco_mpc_study/configs/B02_two_link_mpc.yaml --no-show-viewer
```

B02 轻量 regression：

```bash
python projects/B_mujoco_mpc_study/simulator/scripts/run_B02_two_link_mpc_tracking_demo.py --config projects/B_mujoco_mpc_study/configs/B02_two_link_mpc_regression.yaml
```
