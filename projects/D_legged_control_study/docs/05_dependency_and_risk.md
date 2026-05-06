# Project D 依赖与风险

## 已确认依赖信息

README 提到：

- 项目基于 OCS2。
- 项目基于 ros-control。
- 构建涉及 catkin。
- OCS2 是重型 monorepo，不建议编译整个仓库。
- 运行涉及 Gazebo / RViz / controller manager / Unitree 示例。

OCS2 文档确认它是 C++ optimal control toolbox，并提供 ROS interface 工具。

## 当前不执行的原因

- 本次任务明确不 clone、不下载、不编译、不运行仿真。
- 项目 README 明确不再维护。
- ROS / OCS2 / Gazebo 依赖重，环境成本高。
- 直接运行会偏离当前 A 的 MuJoCo / Pinocchio 学习主线。

## 技术风险

- 不维护项目可能遇到依赖版本漂移。
- ROS Noetic / catkin / OCS2 / Pinocchio 版本耦合强。
- 真机接口和仿真接口混在同一控制栈中，需要谨慎区分。
- WBC / NMPC 实时性调参成本高。

## 当前安全建议

- 只阅读 README、OCS2 文档和关键源码。
- 先复现小型 contact force QP。
- 后续如要运行，单独做环境评估文档。
