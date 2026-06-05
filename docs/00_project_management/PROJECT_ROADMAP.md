# Project Roadmap

## Repository Roadmap

仓库级路线图按项目拆分，目标是把不同学习线的“下一步”放在一个地方统一查看，而不是把所有历史步骤堆在同一份文档里。

## Roadmap Table

| Project | Near-Term Roadmap |
|---|---|
| Project A | 完成 `TargetDefinition` -> 让 A05 读取 `target_definition_json` -> 让 A06 生成 target 定义 -> 再进入 mocap / actuator / video |
| Project B | 延续 B03 sampling-family 与 iLQR line -> 保持 B02 regression 健康检查 -> 再扩展更完整 tracking / rollout 任务 |
| Project C | 继续 OpenLoong 源码审查 -> 梳理 MPC / WBC / PVT 数据流 -> 再规划最小 simulation-only demo |
| Project D | 继续 legged_control / OCS2 阅读 -> 先讲清 quadruped contact / gait / state estimation -> 再进入简化仿真 demo |
| Project E | E01 upstream static audit -> E02 container readiness audit -> E03 public bundle / model audit -> E04 visualization smoke -> E05 public eval smoke |

## Project A Detailed Roadmap

1. R1：完成 `TargetDefinition` load/save/validate
2. R2：让 A05 读取 `target_definition_json`
3. R3：让 A06 生成 `A06_target_definition.json`
4. R4-R8：补 viewer / mocap / keyboard / drag / kinematic follow
5. R9：实现 A07 actuator tracking
6. R10：最后进入 video demo

## Project B Detailed Roadmap

1. 保持 B02 two-link tracking benchmark / regression
2. 继续 B03 sampling-family solver
3. 继续 mini iLQR / iLQG-lite 学习线
4. 在稳定 smoke 基础上再扩展更完整 tracking 任务

## Project C Detailed Roadmap

1. 继续官方工程复现审查
2. 继续 `walk_wbc` / `StateEst` / interface 数据流阅读
3. 建立 simulation-only humanoid WBC / MPC 学习地图

## Project D Detailed Roadmap

1. 继续 quadruped NMPC / WBC / estimation 阅读
2. 先明确 contact schedule、QP、state estimation 三条主线
3. 后续再进入最小 quadruped simulator

## Project E Detailed Roadmap

1. E00：retarget cleanup
2. E01：upstream static audit
3. E02：container readiness audit
4. E03：public bundle / model audit
5. E04：visualization smoke
6. E05：public model eval smoke
7. E06：metrics / figures / videos
8. E07：minimal training feasibility
9. E08：integration report
