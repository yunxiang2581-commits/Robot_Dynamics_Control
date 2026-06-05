# Reproduction Scope

## Allowed

- upstream static audit
- container readiness audit
- public bundle / rosbag / model artifact audit
- visualization smoke planning
- eval smoke planning
- metrics / figures / videos planning

## Not Allowed In Current Phase

- real robot deployment
- hardware drivers
- firmware work
- sim2real claim
- private robot resources
- full paper reproduction claim before public eval path is confirmed

## Reproduction Principle

当前 Project E 的重点是“先建立可执行的公开复现路径”，而不是一上来就训练。优先级必须是：

1. 容器和运行包装是否存在
2. public bundle / eval 入口是否存在
3. visualization 是否可做最小 smoke
4. 训练是否值得后置进入
