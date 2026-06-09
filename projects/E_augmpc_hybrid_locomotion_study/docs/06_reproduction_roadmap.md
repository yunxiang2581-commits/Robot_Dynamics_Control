# Reproduction Roadmap

Project E 的路线图不是“尽快跑起来”，而是先把复现边界、公开资源、执行前置条件和未来命令计划拆清楚，再进入更高风险的执行动作。

## Dual-View Roadmap Table

| Stage | Management Goal | Learning Goal | Main Inputs | Allowed Actions | Main Outputs | Stop Condition | Next Handoff |
|---|---|---|---|---|---|---|---|
| E00 retarget cleanup | 切断旧 `isaac-quad-loco` 方向，确立新 Project E 边界 | 明确本项目为什么转向 AugMPC / IBRIDO 路线 | 旧 Project E 骨架、仓库总览 | 文档重定位、目录清理、状态重写 | 新 Project E 骨架、项目定位文档 | 不再残留旧主线误导 | E01 |
| E01 upstream static audit | 建立公开上游地图和静态证据 | 理解谁是算法主仓库、谁是框架索引、谁是执行入口 | GitHub 公开仓库、Hugging Face 公开页 | clone、README / LICENSE / 入口脚本静态阅读 | clone 状态、git 快照、架构图、容器路线、风险判断 | 上游层次、公开限制、首个低风险目标已清楚 | E02 |
| E02 full-process reproducibility learning roadmap | 把 E00-E06 串成可管理、可学习的闭环流程 | 理解为什么先查边界、再查资源、再查前置条件、再写未来命令 | E00 / E01 结论 | 路线图设计、阶段门槛整理、双视角总表 | 路线图总表、依赖图、退出条件、阶段解释 | 每阶段存在理由、输入输出和风险边界都已明确 | E03 |
| E03 public resource completeness audit | 确认公开 bundle / rosbag / configs / model metadata 是否足够支撑后续动作 | 学会区分“公开可见”与“真正可运行”之间的差距 | E01 公开路径、E02 阶段定义 | 只做公开资源清点和一致性核对，不下载大文件 | 公开资源清单、缺口清单、可公开验证面 | 已知哪些资源是公开的、哪些仍缺失或受限 | E04 |
| E04 execution prerequisites checklist | 在不安装的前提下列清宿主机与容器执行条件 | 理解执行失败常见原因来自哪一层：GPU、显示、挂载、认证、存储 | E03 资源清单、container docs | 只检查条件，不安装、不修复 | 前置条件清单、风险分层、准备度结论 | 执行前提已经被列清，且与公开资源要求对齐 | E05 |
| E05 delayed reproduction command plan | 先把未来命令序列写对，再决定要不要执行 | 理解从 bundle / bag / eval / container script 到实际命令的映射关系 | E03 资源边界、E04 前提清单、container entrypoints | 只写未来命令计划，不执行 | 命令草案、分场景执行序列、失败回退点 | 未来执行命令已经可审阅、可讨论、可延后触发 | E06 |
| E06 learning report | 回收整个复现学习过程，形成稳定理解 | 总结 AugMPC 的 RL + MPC 架构、复现边界和学习价值 | E00-E05 全部产物 | 汇总、对照、反思、提炼 | 学习报告、架构总结、边界总结、后续建议 | 项目对“值不值得继续执行”已有高质量文字判断 | later execution choice |

## Management View

- 先做 E00-E02，是为了把项目定义、公开上游地图和全流程阶段逻辑固定下来。
- E03-E05 都故意延迟真实执行，把“资源完整性”“执行前提”“未来命令”拆成三个不同问题，避免一次混在一起导致误判。
- E06 不是附属总结，而是是否继续投入执行成本的决策依据。

## Learning View

- E00 学的是“目标重定向”，避免沿着错误问题深入。
- E01 学的是“上游结构识别”，区分算法层、框架层和容器执行层。
- E02 学的是“复现流程设计”，把复杂项目拆成可解释阶段。
- E03 学的是“公开资源边界”，理解 public metadata 不等于 public runnable pipeline。
- E04 学的是“执行依赖分层”，理解系统问题来自哪一层。
- E05 学的是“命令语义映射”，把 README 里的零散片段整理成将来可审阅的执行合同。
- E06 学的是“架构回收”，把零散审计结论转成稳定认知。

## Current Recommendation

- 当前已完成：E00、E01、E02
- 当前下一步：E03 `public resource completeness audit`
- 当前仍禁止：安装依赖、启动容器、下载大体积 bundle / rosbag、运行训练或评估
