# E01 Reproduction Target Decision

## Decision

E01 建议的第一复现目标不是训练，也不是 IsaacSim 全链路评估，而是：

**优先选择公开可见的 `public bundle + public rosbag visualization / inspection` 路线，机器人优先级建议 `Centauro` 或 `B2W`。**

## Why This Target

- 这是公开文档中最明确提到的“无需先跑完整仿真也能观察结果”的路径。
- 它绕开了最重的 IsaacSim / NGC / 私有描述包前置条件。
- 它能先验证 bundle 命名、路径约定、容器路由和公开资源边界是否真实可落地。
- 对 Project E 当前阶段来说，这比直接进入训练更符合“先收敛执行边界，再收敛性能边界”的策略。

## Not Recommended As First Step

- `Kyon` 相关公开复现
- `u24` 容器路线
- 完整训练复现
- 未经容器脚本约束的裸跑尝试

## Suggested E02 Direction

`E02 container readiness audit`
- 只检查宿主机是否具备 Apptainer / NVIDIA / 挂载路径 / 基础命令条件。
- 不启动训练，不下载 bundle，不运行 IsaacSim。
