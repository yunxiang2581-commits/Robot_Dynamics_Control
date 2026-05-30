# Step 9A - A01 Inspect URDF TODO Skeleton

## 1. 本步骤目标

把 A01 `inspect_urdf` 适配成学习型 TODO 骨架, 让它成为 A pipeline 的标准第一步入口, 同时保留清晰的路径学习、模型搜索学习和报告输出框架。

## 2. 为什么先做 TODO 骨架

- AGENTS.md 明确要求先生成带注释骨架, 再逐步补全关键逻辑。
- URDF 路径定位、候选路径搜索和 frame 搜索本身就是正式学习内容。
- 如果现在直接补成完整实现, 会跳过“为什么这么设计”的学习过程。

## 3. 本次遵守了哪些 AGENTS.md 规则

- 保留了学习型脚本结构: `argparse`、`logging`、`main()`。
- 不做大规模重构。
- 不删除已有教学注释。
- 所有主要代码块都保留中文教学说明。
- 所有 TODO 都写清了:
  - 要补什么
  - 为什么需要这一步
  - 推荐 API
  - 输入
  - 输出
  - 如何验证
- 不实现 FK、Jacobian、IK、QP、WBC、MuJoCo 控制或 RL。

## 4. 修改文件清单

- `projects/A_self_baseline/src/robot_baseline/model_loader.py`
- `projects/A_self_baseline/scripts/01_model_inspect.py`
- `projects/A_self_baseline/configs/robot.yaml`
- `projects/A_self_baseline/docs/01_model_inspect.md`
- `projects/A_self_baseline/README.md`
- `docs/00_project_management/step9A_A01_inspect_urdf_todo_skeleton.md`

## 5. 未实现算法说明

本步骤没有完整实现以下逻辑:

- YAML 配置读取
- Pinocchio 模型加载
- 模型摘要提取
- frame 关键词搜索
- Markdown 报告生成
- JSON 摘要生成

这些逻辑当前都保留在 TODO 学习骨架中。

## 6. 下一步如何逐个补 TODO

1. 先补 `load_yaml_config` 和 `resolve_path`。
2. 再补候选 URDF 路径构造。
3. 再补 `load_pinocchio_model`。
4. 再补 `summarize_model`。
5. 再补 `find_frames_by_keywords`。
6. 最后补报告和 JSON 输出。

每次只补一个小块, 并用 `py_compile` 和文本输出验证。

## 7. 验收清单

- [x] 只修改了允许的 6 个文件。
- [x] 没有修改 `legacy_imported`。
- [x] 没有使用旧项目绝对路径。
- [x] A01 明确写出 pipeline 输入和输出。
- [x] `model_loader.py` 已扩展为 A01 学习模块骨架。
- [x] `configs/robot.yaml` 已提供清晰模板。
- [x] A01 文档已创建。
- [x] 代码仍然是 TODO 学习骨架。
