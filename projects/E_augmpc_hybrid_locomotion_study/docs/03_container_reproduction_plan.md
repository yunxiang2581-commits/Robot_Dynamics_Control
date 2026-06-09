# Container Reproduction Plan

Project E 采用 container-first 路线。

## Priority

1. 优先审查 Apptainer / Singularity 方案
2. 再评估 Docker 是否可行
3. 不默认假设当前 host 能直接裸机运行

## Why Container-First

- RL-augmented MPC 项目通常依赖复杂系统栈
- 容器更适合固定图形、GPU、系统依赖和复现命令
- 先做容器 readiness audit，比直接安装依赖风险更低

## E04 / E05 Relation

- E04 只列执行前置条件，不安装任何东西
- E05 只把将来可能执行的命令写成计划，不实际运行
- 在 E03 之前先把公开资源边界确认清楚，避免为了补资料而误触下载或执行
