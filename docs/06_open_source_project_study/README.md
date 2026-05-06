# 开源复杂运动控制项目资料与仿真交付索引

本目录是 Project B/C/D 的总索引，用于把复杂开源控制项目与当前 `A_self_baseline` 学习路线连接起来。

## 项目映射

- Project B：MuJoCo MPC / MJPC
- Project C：OpenLoong-Dyn-Control
- Project D：legged_control

## 最终交付形式

Project B/C/D 不是纯资料阅读项目。最终目标是：

1. 读懂复杂开源运动控制项目的核心思想。
2. 把核心算法抽象成可解释的数学模块。
3. 在 simulation-only 条件下做最小可运行仿真器。
4. 导出可展示的视频 demo。
5. 记录误差、约束违反、运行时间等指标。

## 当前阶段边界

本次只做目录和 Markdown 文档整理：

- 不 clone 外部仓库。
- 不下载大文件。
- 不编译。
- 不运行仿真。
- 不实现 Python/C++ 逻辑。
- 不修改 `A_self_baseline`。
- 不修改已有 Python/C++ 控制代码。

## 阅读入口

- [project_bcd_overview.md](project_bcd_overview.md)
- [project_bcd_priority.md](project_bcd_priority.md)
- [project_bcd_simulation_only_roadmap.md](project_bcd_simulation_only_roadmap.md)
- [project_bcd_video_demo_requirements.md](project_bcd_video_demo_requirements.md)

## 建议阅读顺序

先读 `project_bcd_overview.md`，再读 `project_bcd_simulation_only_roadmap.md`，最后读各 Project 的 `07_simulator_and_video_demo_plan.md`。
