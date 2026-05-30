# Step 8.6 - Adapt A01-A07 As Pipeline Components

## 1. 为什么要做这次适配

Step 5 创建了 A01-A07 的标准 TODO 学习脚本，但这些脚本更像独立 demo。为了让 A 项目成为完整 baseline，需要明确它们属于同一条 pipeline：

```text
URDF -> FK -> Jacobian -> DLS-IK -> QP-IK -> MuJoCo PD -> Mini-WBC
```

这次适配只补充文档、输出目录约定和 TODO 骨架，不实现完整算法。

## 2. 修改了哪些文件

新增：

- `projects/A_self_baseline/docs/A_pipeline_contract.md`
- `projects/A_self_baseline/outputs/README.md`
- `projects/A_self_baseline/src/robot_baseline/pipeline_io.py`
- `docs/00_project_management/step8_6_A_pipeline_components.md`

更新：

- `projects/A_self_baseline/README.md`
- `projects/A_self_baseline/scripts/01_model_inspect.py`
- `projects/A_self_baseline/scripts/02_configuration_site_pose.py`
- `projects/A_self_baseline/scripts/03_site_jacobian_check.py`
- `projects/A_self_baseline/scripts/04_dls_differential_ik.py`
- `projects/A_self_baseline/scripts/05_task_limit_qp_ik.py`
- `projects/A_self_baseline/scripts/06_target_mocap_tracking.py`
- `projects/A_self_baseline/scripts/07_mujoco_actuator_tracking.py`

新增输出占位目录：

- `projects/A_self_baseline/outputs/cache/`
- `projects/A_self_baseline/outputs/reports/`
- `projects/A_self_baseline/outputs/figures/`
- `projects/A_self_baseline/outputs/trajectories/`
- `projects/A_self_baseline/outputs/logs/`
- `projects/A_self_baseline/outputs/videos/`

## 3. A01-A07 依赖关系

- A01：检查 URDF、joint、frame 和模型维度。
- A02：消费 A01 的模型和 frame 选择，计算 FK 位姿。
- A03：消费 A02 的 frame 选择，验证 Jacobian。
- A04：消费 A03 验证过的 Jacobian，运行 DLS-IK。
- A05：消费 A04 的任务定义，加入关节限制和 QP 结构。
- A06：消费 A04/A05 的关节轨迹，在 MuJoCo 中做 PD tracking。
- A07：消费 A03-A06 的任务、约束和仿真概念，作为 Mini-WBC/QP 过渡。

## 4. 新增 pipeline_io.py 的目的

`pipeline_io.py` 统一管理 A pipeline 路径和输出文件名，避免每个脚本单独决定输出位置。

当前只保留 TODO 骨架，后续实现时再逐步补全：

- A 项目根目录定位；
- 仓库根目录定位；
- 输出目录类别校验；
- A01-A07 标准输出路径生成；
- 轻量 JSON 和 Markdown 占位写入。

## 5. 未实现算法说明

本步骤没有实现完整 FK、Jacobian、IK、QP、WBC 或 MuJoCo 控制算法。七个标准脚本仍保留 `NotImplementedError`，核心逻辑继续通过中文 TODO 描述。

## 6. 下一步

下一步进入 A-01 inspect URDF 实现：

- 先实现 `model_loader.py` 中的模型加载和摘要；
- 再实现 `01_model_inspect.py` 的 CLI 调用；
- 输出 `A01_model_inspect.md` 和 `A01_model_summary.json`；
- 用实际 URDF 验证 `nq`、`nv`、joint 和 frame 列表。
