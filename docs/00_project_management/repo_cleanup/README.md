# 仓库整理方案说明

本目录用于记录仓库整理前后的审计、合并、归档和删除候选规则。它不是执行脚本目录，也不直接触发任何删除或移动操作。

## 整理原则

仓库整理遵循以下顺序：

```text
先审计 -> 再合并 -> 再归档 -> 最后删除
```

核心原则：

- 不直接删除高风险内容。
- 对任何可能有学习价值、历史价值或项目价值的文件，先标记为“待确认”。
- 外部开源仓库只作为源码阅读参考，不合并进主项目。
- `projects/A_self_baseline/` 是当前主学习项目，不做删除型整理。
- 旧文档优先归档，不直接删除。
- 缓存文件可以作为低风险清理候选，但必须单独步骤执行。

## 当前不做的范围

当前不做：

- 实物部署。
- sim2real 实机测试。
- 电机 SDK。
- CAN / EtherCAT / 串口通信。
- 固件。
- 真实机器人安全测试。
- 真实传感器标定。
- 硬件接口。

## Project B/C/D 最终方向

Project B/C/D 不是单纯源码阅读项目。最终目标是：

- simulation-only runnable simulator。
- video demo。
- metrics 输出。
- README 中可复现运行说明。

对应关系：

- Project B：MuJoCo MPC runnable simulator + video demo。
- Project C：OpenLoong-inspired humanoid MPC/WBC simulation-only demo。
- Project D：legged_control-inspired quadruped NMPC/WBC simulation-only demo。
