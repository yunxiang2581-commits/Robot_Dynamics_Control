# Public Bundle Eval Plan

Project E 不把训练作为第一复现目标。先走 public bundle / eval 路线。

## Suggested Order

1. 审查 public bundles 是否存在
2. 审查 rosbag / artifact / model file 说明是否存在
3. 审查 visualization 入口
4. 审查 public eval 命令
5. 确认 metrics / figures / videos 输出形式

## Why This Order

- public bundle / eval 通常比训练更轻量
- 更适合作为“复现是否能落地”的第一检查点
- 能更快看清 high-level RL 输出和 low-level MPC 执行之间的接口
