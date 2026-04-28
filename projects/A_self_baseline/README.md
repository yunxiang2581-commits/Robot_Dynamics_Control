# A Self Baseline

## 中文版说明

本项目是 A 主线：自研机器人运动控制基础系统。

目标技术栈是 Pinocchio + MuJoCo + OSQP。后续学习链路包括 URDF 加载、FK、Jacobian、IK、QP-IK、MuJoCo PD 控制和教学版 Mini-WBC。

当前状态是 TODO 教学骨架：已经建立标准脚本、模块接口、配置模板和导入资产分区，但不包含完整算法实现。

本项目必须和 B/C 保持独立。不要把 B 项目的 `legged_control` 复现笔记、C 项目的强化学习训练输出或外部项目源码混入本目录。

## English Summary

This is the self-developed robot motion control baseline project. It uses Pinocchio, MuJoCo, and OSQP as the planned stack, and currently contains TODO learning skeletons rather than complete algorithms.

## 文档入口

- `docs/README.md`：A 项目文档入口。
- `docs/legacy_imported/README_from_Pinocchio_URDF.md`：从旧 Pinocchio_URDF 项目导入的 README 参考。
- `docs/legacy_imported/AGENTS_from_Pinocchio_URDF.md`：从旧项目导入的 agent 说明。
- `docs/legacy_imported/AGENT_from_Pinocchio_URDF.md`：从旧项目导入的 agent 说明。
- `docs/legacy_script_mapping.md`：legacy 脚本到 A 标准学习脚本的映射表。
- `docs/self_baseline_learning_scripts_plan.md`：A 标准 TODO 学习脚本规划。

本项目自己的文档目录是 `projects/A_self_baseline/docs/`。

## 标准学习脚本

- `scripts/01_inspect_urdf.py`：检查 URDF 模型的 `nq`、`nv`、joint 和 frame。
- `scripts/02_fk_frame_pose.py`：目标 frame 正运动学 TODO 入口。
- `scripts/03_jacobian_fd_check.py`：Jacobian 有限差分验证 TODO 入口。
- `scripts/04_dls_ik_demo.py`：Damped Least Squares IK TODO 入口。
- `scripts/05_qp_ik_joint_limit_demo.py`：带关节约束的 QP-IK TODO 入口。
- `scripts/06_mujoco_pd_tracking.py`：MuJoCo 关节 PD 跟踪 TODO 入口。
- `scripts/07_mini_wbc_qp_demo.py`：教学版 Mini-WBC QP 结构 TODO 入口。

## 源码模块

- `src/robot_baseline/model_loader.py`：Pinocchio 模型加载和摘要 TODO。
- `src/robot_baseline/kinematics.py`：FK 和 frame 候选查询 TODO。
- `src/robot_baseline/jacobian_check.py`：Jacobian 与有限差分验证 TODO。
- `src/robot_baseline/ik.py`：DLS IK TODO。
- `src/robot_baseline/qp_ik.py`：带约束 QP-IK TODO。
- `src/robot_baseline/pd_controller.py`：关节 PD 力矩 TODO。
- `src/robot_baseline/mini_wbc.py`：教学版 Mini-WBC QP TODO。
- `src/robot_baseline/metrics.py`：误差、CSV 和绘图辅助 TODO。

## 配置文件

- `configs/robot.yaml`：机器人模型路径和关键 frame 占位。
- `configs/ik.yaml`：DLS IK 参数模板。
- `configs/qp_ik.yaml`：带约束 QP-IK 参数模板。
- `configs/mujoco_pd.yaml`：MuJoCo PD 跟踪参数模板。
- `configs/mini_wbc.yaml`：Mini-WBC QP 结构模板。

## 当前状态

当前 A 项目是 TODO 教学骨架。脚本和模块已经定义入口、函数签名、中文 TODO 和验证预期，但尚未实现完整 FK、Jacobian、IK、QP、WBC 或 MuJoCo 控制算法。

## legacy_imported 的作用

`legacy_imported/` 保存从旧项目导入的历史脚本和文档，只作为参考材料。标准实现时不要直接修改这些文件，应先理解旧脚本意图，再在 `scripts/` 和 `src/robot_baseline/` 中实现干净版本。

标准 TODO 脚本在 `scripts/`；标准模块在 `src/robot_baseline/`。

## root_imported 的作用

`root_imported/`、`root_imported_src/` 和 `root_imported_utils/` 保存从本仓库早期根目录归位来的 A 项目资产。它们还不是标准 A 实现。

后续应逐步审阅这些文件，再重构到标准位置：

- 可复用库代码 -> `src/robot_baseline/`
- 学习入口脚本 -> `scripts/`
- 机器人或环境模板 -> `configs/` 或 `envs/`
- 实验入口 -> `experiments/`
- 测试 -> `tests/`

在用途和依赖明确前，不要把这些导入文件混入标准骨架。

## 推荐实现顺序

1. 检查 URDF 并确认模型、joint 和 frame 名称。
2. 实现 FK frame pose。
3. 实现 Jacobian 和有限差分验证。
4. 实现 DLS IK。
5. 实现带约束 QP-IK。
6. 实现 MuJoCo PD 跟踪。
7. 实现 Mini-WBC QP 结构。
