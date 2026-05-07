# A Simulation-Only Full Motion Control Plan

## 1. Current Direction

A 项目是 simulation-only 学习项目，不做真实硬件、不做 sim2real、不接电机。

当前结构：

```text
Foundation validation: A00-A03
Unified interfaces: Target -> IK -> Viewer -> Actuator
```

## 2. Physical Meaning

- Target interface 定义“机器人应该去哪里”。
- IK interface 定义“关节轨迹如何生成”。
- Viewer interface 定义“目标如何在仿真界面中显示或交互变化”。
- Actuator interface 定义“轨迹如何进入 MuJoCo actuator tracking”。

## 3. Current Non-Implementation

Step R-C 不实现：

- full DLS / QP-IK。
- viewer mouse drag。
- keyboard target movement。
- derived MJCF。
- actuator tracking。
- video recording。

## 4. Later Demo Path

后续可按顺序恢复：

1. R1 TargetDefinition load/save/validate。
2. R2/R3 A05/A06 target JSON 回归。
3. R4-R7 viewer target extension。
4. R8 A07 actuator tracking。
5. A10 video demo。

## 5. Safety Boundary

只有 A07 允许规划 `data.ctrl` 和 `mujoco.mj_step` 控制闭环。A04/A05/A06 不写 actuator control。
