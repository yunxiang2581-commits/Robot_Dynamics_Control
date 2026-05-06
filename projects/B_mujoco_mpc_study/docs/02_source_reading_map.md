# Project B 源码阅读地图

## 阅读原则

本阶段不 clone、不编译、不运行。后续如果需要源码阅读，应先在线阅读或只下载小型文本文件，避免把完整外部工程并入本仓库。

## 重点阅读方向

| 方向 | 关注点 | 当前处理建议 |
|---|---|---|
| `agent` | 当前状态、planner 调用、控制输出 | 适合重点阅读 |
| `planners` | Predictive Sampling、iLQG、Gradient Descent | 适合重点阅读 |
| `tasks` | task residual、cost、transition | 适合重点阅读 |
| `simulate` | MuJoCo rollout / step / reset | 先阅读接口 |
| `app` | GUI 和交互入口 | 只适合阅读 |
| Python interface | Python agent demo、实验 API | 暂不建议执行 |

## 已从公开资料确认的信息

- GitHub 顶层目录包含 `mjpc`、`python`、`docs`。
- README 提到 Python API 示例位于 `python/mujoco_mpc/demos`。
- README 提到 GUI 应用需要 CMake / Ninja / 编译流程。

## 待后续源码阅读确认的信息

- `agent` 是否是 planner 与 simulator 的主协调层。
- `tasks` 中 residual 维度、权重、norm 的具体表达方式。
- `planners` 中 horizon 与控制序列的更新方式。
- Python API 是否通过 service 与 C++ 核心通信。

## 建议阅读顺序

1. README 和 docs 中的 Predictive Control 页面。
2. `mjpc/tasks` 中最简单 task。
3. `mjpc/planners` 中 Predictive Sampling。
4. `mjpc/agent` 如何连接 task、planner、MuJoCo。
5. Python demo 只看调用方式，不运行。
