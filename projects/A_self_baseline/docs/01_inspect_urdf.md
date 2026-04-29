# A01 model inspect / MJCF inspect TODO skeleton

## 1. A01 当前定位

A01 当前定位是 **model inspect / MJCF inspect**。

文件名暂时仍为 `01_inspect_urdf.py`，但 A_self_baseline 当前主线对标 `kevinzakka/mink` 的 UR5e MuJoCo 示例，因此第一版重点检查 `scene.xml`，不是在本步骤完整实现 URDF / Pinocchio 流程。

## 2. 为什么 A01 先做模型检查

A02-A07 都依赖模型基础信息：

- `nq`
- `nv`
- `nu`
- joint names
- body names
- site names
- actuator names
- keyframe names
- end-effector candidates

如果不先检查模型对象名称，后续 FK、Jacobian、IK、QP、actuator tracking 很容易靠猜名字写代码，调试成本会很高。

## 3. 对标 mink 的概念

A01 对标 mink UR5e 示例中的以下概念：

- MuJoCo model loading
- UR5e scene inspect
- `Configuration` 的前置模型维度检查

mink 示例后续会围绕 MuJoCo model、configuration、site/task 构建控制逻辑。A_self_baseline 先用 A01 明确模型维度和对象名称。

## 4. 输入

A01 未来输入包括：

- `projects/A_self_baseline/configs/robot.yaml`
- `shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`
- `end_effector_candidates`

默认候选末端包括：

- `attachment_site`
- `tool0`
- `ee_link`
- `wrist_3_link`

## 5. 未来输出

A01 后续最小可运行实现会输出：

- `outputs/reports/A01_model_inspect_report.md`
- `outputs/cache/A01_model_summary.json`

Step 9A 不生成这些真实输出，只整理 TODO 骨架。

## 6. TODO 实现清单

### TODO 1: 读取 YAML

- 推荐 API: `yaml.safe_load`, `Path.read_text`
- 输入: `configs/robot.yaml`
- 输出: Python `dict`
- 验证: 打印 key 列表，确认包含 `mjcf_path` 和 `end_effector_candidates`

### TODO 2: 解析路径

- 推荐 API: `pathlib.Path`, `expanduser`, `is_absolute`, `resolve`
- 输入: YAML 中的 `mjcf_path` 和 CLI 覆盖路径
- 输出: 解析后的 `Path`
- 验证: 打印解析前后路径，并确认路径定位逻辑清楚

### TODO 3: 加载 MJCF

- 推荐 API: `mujoco.MjModel.from_xml_path`
- 输入: `scene.xml`
- 输出: MuJoCo `MjModel`
- 验证: 只打印 `model.nq`, `model.nv`, `model.nu`

### TODO 4: 枚举模型对象

- 推荐 API: `mujoco.mj_id2name`, `mujoco.mjtObj`
- 输入: MuJoCo `model`
- 输出: joint/body/site/actuator/keyframe 名称列表
- 验证: 名称列表长度与 model 中对应数量字段一致

### TODO 5: 检查末端候选

- 推荐 API: Python list/dict, 字符串精确匹配和包含匹配
- 输入: `end_effector_candidates`
- 输出: 每个候选名称是否存在、存在于哪类对象中
- 验证: 报告中明确列出命中和未命中项

### TODO 6: 保存 JSON

- 推荐 API: `json.dumps`, `Path.write_text`
- 输入: 模型 summary dict
- 输出: `outputs/cache/A01_model_summary.json`
- 验证: 打开 JSON，确认字段完整

### TODO 7: 保存 Markdown

- 推荐 API: `Path.write_text`, Markdown 列表和表格
- 输入: 模型 summary dict
- 输出: `outputs/reports/A01_model_inspect_report.md`
- 验证: 报告可读，包含维度、对象名称和末端候选检查

## 7. 当前不做什么

Step 9A 明确不做：

- 不做 FK
- 不做 Jacobian
- 不做 IK
- 不做 QP
- 不做 actuator tracking
- 不做视频录制
- 不调用 mink 替代自己的实现
- 不修改 `external/mink_upstream`
- 不修改 `legacy_imported`

## 8. 下一步

下一步是 **Step 9B：逐个补 TODO，做最小可运行 model inspect**。

Step 9B 才开始实现最小 MuJoCo model loading、对象枚举、JSON summary 和 Markdown report。
