# Project C 源码阅读地图

## 重点源码阅读方向

| 模块 | 阅读目标 | 当前建议 |
|---|---|---|
| `DataBus` | 模块间数据字段和读写边界 | 重点阅读 |
| `StateEstimator` | base / joint / contact 状态估计 | 重点阅读 |
| `Pin_KinDyn` | Pinocchio 运动学动力学封装 | 重点阅读 |
| `MPC` | CoM / 接触力 / horizon 优化 | 重点阅读 |
| `WBC_QP` | 全身 QP 决策变量与约束 | 重点阅读 |
| `PVT_Ctr` | 低层关节控制 | 适合阅读 |
| `GaitScheduler` | 支撑相 / 摆动相调度 | 重点阅读 |
| `FootPlacement` | 落脚点规划 | 重点阅读 |
| `JoyStickInterpreter` | 用户命令到运动目标 | 适合阅读 |
| `MJ_Interface` | MuJoCo 仿真接口 | 适合阅读 |
| `walk_wbc.cpp` | WBC 行走 demo | 重点阅读 |
| `walk_mpc_wbc.cpp` | MPC + WBC 行走 demo | 重点阅读 |
| `jump_mpc.cpp` | 跳跃 demo | 只适合阅读 |

## 已从公开资料确认的信息

- 顶层目录包含 `algorithm`、`common`、`demo`、`math`、`models`、`sim_interface` 等。
- README 提到该项目模块化、通过 bus 进行数据交互。
- README 提到 walking、jumping、blind obstacle stepping 示例。

## 待后续源码阅读确认的信息

- 上述类名与文件名的准确路径。
- demo 中每个模块初始化顺序。
- DataBus 字段命名与单位。
- MuJoCo timestep、controller timestep、MPC timestep 的关系。

## 建议阅读顺序

1. README / README-zh。
2. `demo` 中 walking 相关入口。
3. `DataBus` 字段。
4. `StateEstimator` 和 `Pin_KinDyn`。
5. `GaitScheduler` / `FootPlacement`。
6. `MPC`。
7. `WBC_QP`。
8. `PVT_Ctr`。
