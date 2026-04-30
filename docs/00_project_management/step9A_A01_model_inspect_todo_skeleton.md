# Step 9A - A01 model inspect TODO skeleton

## 1. 为什么先做 TODO 骨架

A_self_baseline 的目标是学习并复现 mink UR5e 示例背后的机器人控制链路，而不是一次性写完整工程代码。A01 是后续 A02-A07 的模型基础信息入口，因此先写 TODO 骨架可以明确输入、输出、边界和学习顺序。

## 2. 本步骤目标

本步骤只创建或整理 A01 model inspect / MJCF inspect 的 TODO learning skeleton。

目标包括：

- 明确 A01 对标 mink UR5e `scene.xml`。
- 明确后续要检查 `nq`, `nv`, `nu`。
- 明确后续要枚举 joint/body/site/actuator/keyframe。
- 明确后续要检查 end-effector candidates。
- 明确 JSON summary 和 Markdown report 的未来输出路径。

## 3. 修改文件清单

本步骤只修改允许范围内的文件：

- `projects/A_self_baseline/src/robot_baseline/model_loader.py`
- `projects/A_self_baseline/scripts/01_model_inspect.py`
- `projects/A_self_baseline/configs/robot.yaml`
- `projects/A_self_baseline/docs/01_model_inspect.md`
- `docs/00_project_management/step9A_A01_model_inspect_todo_skeleton.md`
- `projects/A_self_baseline/README.md`

## 4. 保留了哪些 TODO

### model_loader.py

- `load_yaml_config`
- `resolve_path`
- `load_mujoco_model`
- `get_mujoco_names`
- `summarize_mujoco_model`
- `write_json_summary`
- `write_model_report`

### 01_model_inspect.py

- TODO 1: 读取 robot.yaml
- TODO 2: 解析 mjcf_path
- TODO 3: 加载 MuJoCo model
- TODO 4: 枚举 joint/body/site/actuator/keyframe
- TODO 5: 检查 end-effector candidates
- TODO 6: 写 JSON summary
- TODO 7: 写 Markdown report

## 5. 未实现算法说明

Step 9A 未实现以下内容：

- MuJoCo model loading
- 模型摘要生成
- JSON/Markdown 报告输出
- FK
- Jacobian
- IK
- QP
- WBC
- MuJoCo 控制
- video recording
- mink 替代实现

## 6. 与 AGENTS.md 规则的对应关系

本步骤对应 AGENTS.md 的要求：

- 优先生成“骨架 + TODO”。
- 每个 TODO 使用中文教学注释。
- 路径设置作为正式学习内容。
- 模型搜索作为正式学习内容。
- 不跳到 FK、Jacobian、IK、QP 或控制。
- 不伪装成完整实现。

## 7. 下一步 Step 9B 如何补最小可运行实现

Step 9B 建议按以下顺序补：

1. 实现 `load_yaml_config`。
2. 实现 `resolve_path`。
3. 实现 `load_mujoco_model`，只打印 `nq/nv/nu`。
4. 实现 `get_mujoco_names`。
5. 实现 `summarize_mujoco_model`。
6. 实现 `write_json_summary`。
7. 实现 `write_model_report`。
8. 运行 A01，生成最小 report 和 summary。

## 8. 验收清单

- [x] 只创建/整理 TODO 骨架；
- [x] 未实现 MuJoCo model loading；
- [x] 未实现 FK/Jacobian/IK/QP/控制；
- [x] 未修改 legacy_imported；
- [x] 未修改 external/mink_upstream；
- [x] py_compile 通过。
