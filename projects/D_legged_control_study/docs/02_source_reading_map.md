# Project D 源码阅读地图

## 重点源码阅读方向

| 模块 | 阅读目标 | 当前建议 |
|---|---|---|
| `legged_common` | 通用类型、参数、工具 | 重点阅读 |
| `legged_control` | 主控制流程 | 重点阅读 |
| `legged_controllers` | ros-control controller 接口 | 适合阅读 |
| `legged_estimation` | base 状态估计、Kalman filter | 重点阅读 |
| `legged_examples/legged_unitree` | Unitree 示例和硬件接口 | 只适合阅读 |
| `legged_gazebo` | Gazebo 仿真 | 暂不建议执行 |
| `legged_hw` | 硬件抽象 | 只适合阅读 |
| `legged_interface` | NMPC 接口与问题组织 | 重点阅读 |
| `legged_wbc` | WBC-QP | 重点阅读 |
| `qpoases_catkin` | QP 求解器依赖 | 只适合阅读 |

## 已从公开资料确认的信息

- README 明确项目基于 OCS2 和 ros-control。
- README 明确主模块是 NMPC 和 WBC。
- README 描述了 NMPC、WBC、state estimation 的框架关系。
- README 提到 OCS2 是重型 monorepo，不建议编译整个仓库。

## 待后续源码阅读确认的信息

- 上述模块在当前分支中的准确目录和文件。
- controller 的 update loop 入口。
- NMPC 与 WBC 数据结构如何交接。
- state estimator 的输入输出 topic / handle。

## 建议阅读顺序

1. README。
2. OCS2 introduction。
3. `legged_interface` 中 optimal control problem 组织方式。
4. `legged_wbc` 中 WBC-QP。
5. `legged_estimation` 中 Kalman filter。
6. `legged_controllers` 中主循环。
7. Unitree / Gazebo / hardware 部分只读不跑。
