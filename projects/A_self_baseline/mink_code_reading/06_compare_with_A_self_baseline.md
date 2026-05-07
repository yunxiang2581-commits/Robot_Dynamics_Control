# 06 - mink 与 A_self_baseline 对照学习

## 本节目标

把 mink 源码阅读结果和 A 项目已有教学实现对照起来。

这一步的目标不是把 A 项目改成 mink，而是理解：

- 哪些概念 A 项目已经手动实现。
- 哪些抽象 mink 做得更完整。
- 哪些内容应该继续保持教学拆分。
- 下一步学习时应该补哪块理解。

## 对照入口

### mink 上游源码

- `external/mink_upstream/examples/arm_ur5e.py`
- `external/mink_upstream/src/mink/configuration.py`
- `external/mink_upstream/src/mink/tasks/`
- `external/mink_upstream/src/mink/limits/`
- `external/mink_upstream/src/mink/solve_ik.py`

### A 项目教学实现

- `projects/A_self_baseline/scripts/02_configuration_site_pose.py`
- `projects/A_self_baseline/scripts/03_site_jacobian_check.py`
- `projects/A_self_baseline/scripts/04_ik_dls_wrapper.py`
- `projects/A_self_baseline/scripts/05_ik_qp_wrapper.py`
- `projects/A_self_baseline/configs/ik.yaml`
- `projects/A_self_baseline/configs/qp_ik.yaml`

## 对照表

| 学习问题 | mink 位置 | A 项目位置 | 当前理解 |
| --- | --- | --- | --- |
| 如何加载模型 | examples / MuJoCo model loading | A01 / A02 | TODO |
| 如何管理 q 和 data | `Configuration` | A02 / A03 / A04 | TODO |
| 如何查询 site pose | `Configuration` | A02 | TODO |
| 如何查询 Jacobian | `Configuration` / task | A03 | TODO |
| 如何定义末端任务 | `FrameTask` | A04 / A05 | TODO |
| 如何定义姿态保持 | `PostureTask` | A05 | TODO |
| 如何加入速度限制 | `VelocityLimit` | A05 / `qp_ik.yaml` | TODO |
| 如何加入位置限制 | `ConfigurationLimit` | A05 / `qp_ik.yaml` | TODO |
| 如何构造 QP | `solve_ik.py` | A05 | TODO |
| 如何更新状态 | `integrate_inplace` | A04 / A05 | TODO |

## 推荐对照方法

1. 先读 mink 示例，看用户层 API 有多短。
2. 再回到 A 项目，看同样的数据流被拆成哪些教学步骤。
3. 每读完一个 mink 模块，就在本文件表格中补一行理解。
4. 不急着抽象 A 项目代码，先确认数学逻辑是否真正理解。

## 重点差异

### A 项目更适合学习

A 项目把路径、模型检查、site pose、Jacobian、DLS IK、QP-IK 拆成多个脚本。

好处是每一步都能单独输出报告，便于复盘：

- 输入是什么。
- 输出是什么。
- 矩阵 shape 是否正确。
- 误差是否下降。
- 约束是否生效。

### mink 更适合复用

mink 把状态、任务、限制和 solver 抽象成库接口。

好处是示例代码短，扩展多任务和多限制更方便。

代价是初学时很多细节藏在类和 helper 后面，需要专门阅读源码。

## 下一步学习建议

第一轮建议按这个顺序对照：

1. `arm_ur5e.py` vs A06/A07 目标跟踪。
2. `configuration.py` vs A02/A03。
3. `frame_task.py` vs A04/A05。
4. `velocity_limit.py`、`configuration_limit.py` vs A05。
5. `solve_ik.py` vs A05。

## TODO 学习笔记

- TODO 1: 补全上面对照表的“当前理解”列。
- TODO 2: 标出 A 项目当前和 mink 差距最大的 3 个点。
- TODO 3: 标出不应该马上照搬 mink 的 3 个抽象。
- TODO 4: 写出下一轮最想深读的源码文件。

## 详细学习任务与路径

### 任务 6.1: 对照 Configuration

- 阅读 mink:
  - `source_annotated/src/mink/configuration.py`
- 阅读 A 项目:
  - `projects/A_self_baseline/scripts/02_configuration_site_pose.py`
  - `projects/A_self_baseline/scripts/03_site_jacobian_check.py`
- 对照问题:
  - mink 把哪些功能放进 `Configuration`？
  - A 项目为什么把这些功能拆成 A02/A03？
- 输出:
  - 在对照表中补 “如何管理 q 和 data”、“如何查询 site pose”、“如何查询 Jacobian”。

### 任务 6.2: 对照 FrameTask / PostureTask

- 阅读 mink:
  - `source_annotated/src/mink/tasks/frame_task.py`
  - `source_annotated/src/mink/tasks/posture_task.py`
- 阅读 A 项目:
  - `projects/A_self_baseline/scripts/04_ik_dls_wrapper.py`
  - `projects/A_self_baseline/scripts/05_ik_qp_wrapper.py`
- 对照问题:
  - A04 的 DLS IK 和 FrameTask 有什么关系？
  - A05 的 task 权重和 mink cost 有什么关系？
  - A05 的 posture reference 是否等价于 mink `PostureTask`？
- 输出:
  - 写出 A05 task 结构和 mink task 结构的 3 个相同点、3 个不同点。

### 任务 6.3: 对照 Limits

- 阅读 mink:
  - `source_annotated/src/mink/limits/velocity_limit.py`
  - `source_annotated/src/mink/limits/configuration_limit.py`
- 阅读 A 项目:
  - `projects/A_self_baseline/scripts/05_ik_qp_wrapper.py`
  - `projects/A_self_baseline/configs/qp_ik.yaml`
- 对照问题:
  - A05 的 velocity bound 是否和 mink 的 `VelocityLimit` 一致？
  - A05 的 position margin 是否和 mink 的 `min_distance_from_limits` 类似？
- 输出:
  - 补全对照表中 velocity limit 和 configuration limit 两行。

### 任务 6.4: 对照 solve_ik / QP 构造

- 阅读 mink:
  - `source_annotated/src/mink/solve_ik.py`
- 阅读 A 项目:
  - `projects/A_self_baseline/scripts/05_ik_qp_wrapper.py`
  - `projects/A_self_baseline/src/robot_baseline/qp_ik.py`
- 对照问题:
  - 两边 QP 变量是否都是 `nv` 维？
  - 两边 objective 是否都是 task least-squares + damping？
  - 两边约束是否都能写成 bound 或 inequality？
- 输出:
  - 写出 A05 目前和 mink `solve_ik` 最接近的函数或代码块。

### 任务 6.5: 形成下一步学习清单

- 学习目标:
  - 不急着重构 A 项目，而是明确下一轮补什么。
- 操作:
  - 列出 “马上可以补的 3 个小点”。
  - 列出 “暂时不补的 3 个高级点”。
- 建议分类:
  - 马上可以补: 更清晰 report、更多 shape 输出、QP bound 对照说明。
  - 暂时不补: 完整 collision avoidance、完整 mink API 复刻、复杂多任务优先级系统。
- 验收:
  - 能说明下一步为什么仍然服务 `URDF/MJCF -> FK -> Jacobian -> IK -> QP-IK -> control` 主线。

