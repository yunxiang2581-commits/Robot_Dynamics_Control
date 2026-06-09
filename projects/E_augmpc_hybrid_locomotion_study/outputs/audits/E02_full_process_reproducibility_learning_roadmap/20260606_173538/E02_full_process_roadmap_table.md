# E02 Full Process Roadmap Table

本表是 Project E 的“管理 + 学习”双视角总表。目标不是催促执行，而是先把每一步为什么存在、能做什么、不能做什么、产出什么写清楚。

| Stage | Why This Stage Exists | Inputs | Allowed Actions | Expected Outputs | Learning Focus | Stop Condition | Risks / Not To Do | Handoff |
|---|---|---|---|---|---|---|---|---|
| E00 retarget cleanup | 把旧 `isaac-quad-loco` 路线彻底切掉，避免后续文档和判断继续沿错方向扩散 | 旧 Project E 骨架、仓库总索引 | 文档重定位、目录清理、状态更新 | 新 Project E 目标、边界和目录骨架 | 学会先修正问题定义，再谈复现执行 | 项目目标已经稳定切换到 AugMPC / IBRIDO | 不要继续引用旧 Orbit 路线作为主线 | E01 |
| E01 upstream static audit | 建立上游公开实体的静态地图，判断公开复现到底依赖哪些层 | `LRHControl`、`IBRIDO`、`ibrido-containers`、`AugMPCModels` public metadata | clone、静态阅读 README / LICENSE / entrypoints | clone 状态、git 快照、架构映射、容器路线、资源风险 | 学会识别算法仓库、框架索引仓库、容器执行仓库的分工 | 上游分层、公开限制、首个低风险目标已明确 | 不运行上游入口、不安装依赖、不下载 bundle | E02 |
| E02 full-process reproducibility learning roadmap | 把 E00-E06 串成一条完整的、可解释的复现学习链 | E00/E01 结论、Project E 文档 | 设计路线图、阶段依赖、退出条件、双视角解释 | 路线图总表、管理视角、学习视角、依赖图、退出条件 | 学会把复杂复现任务拆成顺序合理、风险递增的阶段 | 每一阶段的存在理由、输入输出、风险边界都已明确 | 不把后续执行内容提前混入当前阶段 | E03 |
| E03 public resource completeness audit | 在不下载大文件的前提下确认公开资源是否够支撑后续检查和计划 | E01 公开路径、HF metadata、container README | 只做公开 bundle / rosbag / configs / model metadata 清点 | 公开资源清单、缺口清单、一致性判断 | 学会区分“页面上存在”与“真正足够支撑复现” | 已能列出公开可用面和缺失面 | 不下载大文件、不把缺失资源自动脑补为存在 | E04 |
| E04 execution prerequisites checklist | 在执行前把机器、容器、认证、显示、挂载、存储条件先列清 | E03 资源清单、container docs、host reports | 只列条件和风险，不安装不修复 | 前置条件清单、失败风险分层、准备度判断 | 学会定位执行失败会发生在哪一层 | 前置条件和资源要求已对齐成检查表 | 不执行安装、不修改系统、不启动容器 | E05 |
| E05 delayed reproduction command plan | 把未来可能执行的命令先写成合同，再决定要不要触发 | E03 资源清单、E04 条件清单、entrypoint scripts | 只编排命令，不运行 | 延后执行命令清单、场景化命令序列、回退点 | 学会把 README 片段整理成真实可审阅流程 | 将来执行路径已可讨论、可审阅 | 不执行任何命令、不下载、不训练 | E06 |
| E06 learning report | 汇总前五阶段，把复现边界和学习价值变成稳定文字结论 | E00-E05 全部产物 | 汇总、对照、提炼、评估 | 学习报告、架构总结、价值判断、后续建议 | 学会从“查资料”过渡到“形成判断” | 已能解释值不值得继续执行以及为什么 | 不夸大为“完整论文复现”或“已经可运行” | later |
