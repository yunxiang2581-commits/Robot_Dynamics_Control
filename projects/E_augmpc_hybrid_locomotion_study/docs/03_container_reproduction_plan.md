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

## E02 / E03 Focus

- host 是否具备 container runtime
- 是否存在 `Singularity`, `Apptainer`, `Dockerfile` 或 container docs
- 是否有公开 bundle / eval 命令和容器绑定
