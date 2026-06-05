# Training Reproduction Later

训练不作为当前第一复现目标。

## Why Training Comes Later

- 训练成本最高
- 环境与依赖要求最重
- 如果 public eval / visualization 入口都未打通，训练投入回报很低

## Training Enters Only After

- upstream static audit 完成
- container readiness 审查完成
- public bundle / model / rosbag 路径确认
- visualization smoke 可规划
- public eval smoke 可规划或可执行
