# A Self Baseline

> English version: [README_en.md](README_en.md)

本项目是 A 主线：对标 `kevinzakka/mink` UR5e 示例的教学版 6-DOF 机械臂控制 baseline。

A 项目不是完整复刻 mink 库，也不是直接调用 mink 替代自己的实现。当前目标是逐步复现 mink UR5e 示例背后的核心链路：

- MuJoCo model inspect
- configuration / site pose
- site Jacobian
- DLS differential IK
- task + limit + QP-IK
- target / mocap-style tracking
- MuJoCo actuator tracking
- optional collision avoidance TODO
- comparison with mink
- 可展示 demo

当前状态：A 项目已完成 mink/UR5e 对标需求、最小参考资产、A00-A10 pipeline 文档整理，以及 A01-A03 最小可运行实现。

A01 已能读取 `shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`，输出 `nq`、`nv`、`nu`、joint、body、site、actuator 和 keyframe 摘要。A02 已能基于 `keyframe:home` 查询 `attachment_site` 和 `wrist_3_link` 的世界系 pose。

## 文档入口

本项目自己的文档目录是 `projects/A_self_baseline/docs/`。

- `docs/A_pipeline_contract.md`：UR5e/mink-style A00-A10 pipeline 契约。
- `docs/mink_capability_vs_A_requirements.md`：mink 能力拆解与 A 项目对标需求总文档。
- `docs/reference_mink_ur5e.md`：mink 与 UR5e 示例的参考说明。
- `docs/A_mink_alignment_plan.md`：A 项目对齐 mink UR5e 的阶段计划。
- `docs/legacy_script_mapping.md`：历史 legacy 脚本到标准学习脚本的映射。
- `docs/self_baseline_learning_scripts_plan.md`：标准 TODO 学习脚本规划。
- `docs/01_model_inspect.md`：A01 model inspect / MJCF inspect 学习说明。
- `docs/02_configuration_site_pose.md`：A02 configuration / site pose 状态说明。
- `docs/legacy_imported/`：旧项目导入文档，仅作历史参考。

## A00-A10 Pipeline 总览

```text
A00 reference and assets
  -> A01 model inspect
  -> A02 configuration / site pose
  -> A03 site Jacobian check
  -> A04 DLS differential IK
  -> A05 task + limit + QP-IK
  -> A06 target / mocap-style tracking
  -> A07 MuJoCo actuator tracking
  -> A08 collision avoidance TODO
  -> A09 comparison report
  -> A10 demo showcase / video recording
```

当前标准入口统一放在 `scripts/` 下的 A00-A10。A01 已完成最小 model inspect，A02 已完成最小 site/body pose，A03 已完成最小 site Jacobian finite difference check；A04-A10 仍按 TODO learning skeleton 逐步推进。

## mink capability vs A requirements

需求总文档见 `docs/mink_capability_vs_A_requirements.md`。

它用于明确 mink 能力、A 项目需要自己实现的最小范围、暂缓功能、A00-A10 任务表和最终可展示成果。最终 demo 不只是报告，还包括 MuJoCo video demo。主展示目标是：

```text
outputs/videos/A07_ur5e_actuator_tracking_demo.mp4
```

这个视频应配套 tracking log、tracking error figure 和 report，并在 README 或 GitHub Release 中展示。

## 标准入口

- `scripts/00_reference_and_assets.py`：A00 reference and assets。审计 mink reference、UR5e assets、许可证和最小复制边界。
- `scripts/01_model_inspect.py`：A01 model inspect / MJCF inspect。检查 UR5e `scene.xml` 的 `nq`、`nv`、`nu`、joint、body、site、actuator 和 keyframe。
- `scripts/02_configuration_site_pose.py`：A02 configuration / site pose。对标 mink `Configuration`，学习从 `q` 查询 site/body pose。
- `scripts/03_site_jacobian_check.py`：A03 site Jacobian check。学习 `site velocity = J(q) dq` 和有限差分验证。
- `scripts/04_dls_differential_ik.py`：A04 DLS differential IK。学习最小无约束 differential IK。
- `scripts/05_task_limit_qp_ik.py`：A05 task + limit + QP-IK。学习 task、limit 和最小 QP-IK。
- `scripts/06_target_mocap_tracking.py`：A06 target / mocap-style tracking。学习 fixed target 与后续 mocap-style target。
- `scripts/07_mujoco_actuator_tracking.py`：A07 MuJoCo actuator tracking。学习把 A04/A05 的 `q_des` 或 `dq_des` 送入 `data.ctrl`。
- `scripts/08_collision_avoidance_todo.py`：A08 collision avoidance TODO。只记录概念、输入输出和未来接入 QP-IK 的方式。
- `scripts/09_comparison_report.py`：A09 comparison report。规划如何比较“自己实现”和“mink 抽象”。
- `scripts/10_demo_showcase_video.py`：A10 demo showcase / video recording。规划最终展示文档和主 demo 视频路径。

## 代码与配置边界

- `scripts/` 是 CLI 入口，负责参数、日志、输入输出契约和学习步骤组织。
- `src/robot_baseline/` 是可复用逻辑位置，后续放模型检查、site pose、Jacobian、IK、QP、tracking 等可测试函数。
- `configs/` 是配置入口，当前 `configs/robot.yaml` 已转为 UR5e/mink-style MJCF 模型模板。
- `outputs/` 是项目级输出目录，报告、缓存、轨迹、日志、图像和视频按子目录保存。

## mink 本地参考资产

`external/mink_upstream/` 是 mink 上游完整仓库的本地只读镜像，不是 A 项目标准入口。

A 项目只复制最小 UR5e assets/examples：

- 模型资产：`shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`
- IK 对照脚本：`projects/A_self_baseline/external/mink/examples/arm_ur5e.py`
- actuator 对照脚本：`projects/A_self_baseline/external/mink/examples/arm_ur5e_actuators.py`
- 上游说明：`projects/A_self_baseline/external/mink/README.md`
- 许可证：`projects/A_self_baseline/external/mink/LICENSE`
- 依赖参考：`projects/A_self_baseline/external/mink/pyproject.toml`

copied files are reference assets/examples, not our implementation。标准实现仍然在 `scripts/` 和 `src/robot_baseline/`。

## legacy_imported 的作用

`legacy_imported/` 保存从旧项目导入的 H1/Pinocchio/URDF/FK/Jacobian/IK 学习脚本和文档，只作为历史参考。

H1 legacy 不删除，但不是当前 A 项目主线。当前主线是 UR5e / 6-DOF manipulator / mink-style baseline。后续实现时应参考 legacy 的学习方式和验证习惯，而不是直接把 legacy 脚本当成标准入口。

## root_imported 的作用

`root_imported/`、`root_imported_src/` 和 `root_imported_utils/` 保存从本仓库早期根目录归位来的 A 项目资产。它们还不是标准 A 实现。

后续应逐步审阅这些文件，再重构到标准位置：

- 可复用库代码 -> `src/robot_baseline/`
- 学习入口脚本 -> `scripts/`
- 机器人或环境模板 -> `configs/` 或 `envs/`
- 实验入口 -> `experiments/`
- 测试 -> `tests/`

## 当前学习顺序

1. A00：确认 mink UR5e 参考范围和模型资产路径。当前状态：已完成。
2. A01：检查 MuJoCo MJCF 模型维度和对象名称。当前状态：最小实现已完成。
3. A02：实现 configuration / site pose 查询。当前状态：最小实现已完成。
4. A03：实现 site Jacobian 并做有限差分验证。当前状态：最小实现已完成。
5. A04：实现 DLS differential IK。当前状态：下一步。
6. A05：实现 task + limit + QP-IK。
7. A06：实现 target / mocap-style tracking。
8. A07：实现 MuJoCo actuator tracking。
9. A08：记录 collision avoidance TODO。
10. A09：输出和 mink 的对照报告。
11. A10：整理 demo showcase / video recording。

每一步都应先输出文本报告，便于复盘和面试讲解。

## Simulation-first validation plan

A 项目当前没有实物 UR5e / 机械臂，因此定位为 simulation-first motion-control baseline。无实物条件下的有效验证依赖可运行脚本、可解释报告、可量化误差、可复现配置和后续 sim2sim validation，而不是伪装成真实机器人验证。

完整规划见 `docs/A_simulation_only_full_motion_control_plan.md`。该文档把当前 A00-A10 主线扩展到 A18；A03 最小 Jacobian check 已完成，当前下一步进入 A04 DLS differential IK。

## 当前实现入口

```bash
python projects/A_self_baseline/scripts/03_site_jacobian_check.py
```

当前 A03 已完成最小 site Jacobian finite difference check。下一步进入 A04 DLS differential IK，仍不扩展到 QP 或控制。

## A01 model inspect status

当前 A01 已完成最小可运行 model inspect。

- 标准入口是 `scripts/01_model_inspect.py`。
- 模块实现是 `src/robot_baseline/model_loader.py`。
- 配置是 `configs/robot.yaml`。
- 文档是 `docs/01_model_inspect.md`。
- 输出报告是 `outputs/reports/A01_model_inspect_report.md`。
- 输出缓存是 `outputs/cache/A01_model_summary.json`。
- 当前检查结果：`nq=6`、`nv=6`、`nu=6`，末端候选中 `attachment_site` 和 `wrist_3_link` 命中。
- 下一步进入 A04 DLS differential IK。

## A02 configuration / site pose status

当前 A02 已完成最小可运行 site/body pose 查询，依赖 A01 的 `outputs/cache/A01_model_summary.json` 和已确认的 `attachment_site` / `wrist_3_link`。

A02 输出是 `outputs/cache/A02_site_pose.json` 和 `outputs/reports/A02_site_pose_report.md`。当前 q source 为 `keyframe:home`，目标 site/body 为 `attachment_site` / `wrist_3_link`。A02 仍不实现 Jacobian、finite difference、IK 或 QP。

## A03 site Jacobian check status

当前 A03 已完成最小可运行 site Jacobian finite difference check，依赖 A01/A02 的 model summary 和 site pose，验证了 `site velocity = J(q) dq`、MuJoCo `mj_jacSite` 和 finite difference 之间的一致性。

A03 输出是 `outputs/cache/A03_jacobian_check.json`、`outputs/reports/A03_jacobian_check_report.md`、`outputs/figures/A03_jacobian_fd_error.png`、`outputs/cache/A03_multi_step_trace.json` 和 `outputs/figures/A03_multi_step_linearization_error.png`。当前默认 `dq_source=unit:shoulder_pan`、`dt=1e-6`、`fd_steps=1000`，linear velocity error norm 约为 `2.55e-4`，angular velocity error norm 约为 `8.26e-11`。sweep 图已展示误差随总位移变化的趋势；下一步进入 A04 DLS differential IK。
