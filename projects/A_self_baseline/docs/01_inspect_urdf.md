# A01 Inspect URDF

## 1. A01 在 pipeline 中的位置

A01 是 A pipeline 的第一环。

```text
A01 inspect URDF
  -> A02 FK frame pose
  -> A03 Jacobian FD check
  -> A04 DLS-IK
  -> A05 QP-IK
  -> A06 MuJoCo PD tracking
  -> A07 Mini-WBC QP
```

它负责先把机器人模型路径、自由度、joint 名称和 frame 名称查清楚, 后续所有步骤都依赖这里的结果。

## 2. 为什么 URDF 检查是第一步

- 如果 URDF 路径不稳定, 后续 FK、Jacobian、IK 都会反复卡在路径错误上。
- 如果 joint 名和 frame 名不清楚, 后续很容易靠猜名字写死脚本。
- 如果 `nq`、`nv` 和浮动基设置不清楚, 面试时很难解释 Pinocchio 模型的基本结构。

## 3. 本步骤学习目标

- 学会通过 `__file__` 稳定定位 A_ROOT 和 REPO_ROOT。
- 学会从 YAML 配置和命令行共同决定 URDF 路径。
- 学会组织候选 URDF 路径列表, 并打印清晰错误信息。
- 学会区分固定基和浮动基模型在 `nq`、`nv` 上的差异。
- 学会按关键词搜索候选 frame, 为 A02/A03/A04 提供输入。

## 4. 输入

- `projects/A_self_baseline/configs/robot.yaml`
- 命令行可选 `--urdf`
- 共享模型资源示例:
  - `shared/robot_assets/models/h1_description/urdf/h1_with_hand.urdf`

## 5. 输出

- `projects/A_self_baseline/outputs/reports/A01_inspect_urdf_report.md`
- `projects/A_self_baseline/outputs/cache/A01_model_summary.json`

## 6. TODO 实现清单

1. 读取 `configs/robot.yaml`。
2. 解析 `urdf_path`、`package_dirs`、`free_flyer` 和 `frame_keywords`。
3. 构造候选 URDF 路径列表。
4. 调用 Pinocchio 加载模型。
5. 提取 `nq`、`nv`、joint 列表和 frame 列表。
6. 按关键词搜索候选 frame。
7. 生成 Markdown 报告。
8. 生成 JSON 摘要。

## 7. 推荐 Pinocchio API

- `pin.buildModelFromUrdf`
- `pin.JointModelFreeFlyer`
- `model.names`
- `model.frames`

## 8. 验收标准

- 脚本和模块可以通过 `py_compile`。
- A_ROOT 和 REPO_ROOT 路径定位代码清晰可读。
- 配置模板字段完整。
- 脚本明确写出输入和输出路径。
- 代码仍然是 TODO 学习骨架, 没有伪装成完整实现。

## 9. 常见错误

- `pinocchio` 未安装:
  - 现象: 后续实现时 import 失败。
  - 处理: 先检查 Python 环境和依赖版本。
- URDF 路径不存在:
  - 现象: 找不到模型文件。
  - 处理: 打印候选路径列表, 不要只报一个模糊错误。
- `package_dirs` 不正确:
  - 现象: URDF 中的 `package://...` 或 mesh 路径无法解析。
  - 处理: 优先检查 `shared/robot_assets/models` 是否传入。
- frame 名称找不到:
  - 现象: 后续 FK/Jacobian/IK 脚本无法定位目标 frame。
  - 处理: 先做精确匹配, 再做关键词包含匹配。

## 10. 与 legacy 文件的关系

参考文件:

- `projects/A_self_baseline/scripts/legacy_imported/task1_inspect_humanoid_model.py`

关系说明:

- 可以参考它的学习顺序: 先找 URDF, 再检查固定基/浮动基, 再打印 joint/frame 信息。
- 不直接复制成完整实现。
- 不修改 legacy 原文件。
- 标准入口始终应使用 `projects/A_self_baseline/scripts/01_inspect_urdf.py`。
