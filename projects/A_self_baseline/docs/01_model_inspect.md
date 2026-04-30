# A01 model inspect / MJCF inspect status

## 1. A01 当前定位

A01 当前定位是 **model inspect / MJCF inspect**。

标准入口已统一为 `scripts/01_model_inspect.py`。A_self_baseline 当前主线对标 `kevinzakka/mink` 的 UR5e MuJoCo 示例，因此第一版重点检查 `scene.xml`，不是在本步骤完整实现 URDF / Pinocchio 流程。

当前状态：**A01 最小可运行实现已完成**。

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

A01 当前输入包括：

- `projects/A_self_baseline/configs/robot.yaml`
- `shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml`
- `end_effector_candidates`

默认候选末端包括：

- `attachment_site`
- `tool0`
- `ee_link`
- `wrist_3_link`

## 5. 当前输出

A01 当前已输出：

- `outputs/reports/A01_model_inspect_report.md`
- `outputs/cache/A01_model_summary.json`

当前检查结果摘要：

- `nq = 6`
- `nv = 6`
- `nu = 6`
- joint names: `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`
- site names: `attachment_site`
- actuator names: `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3`
- keyframe names: `home`
- end-effector candidates: `attachment_site=True`, `tool0=False`, `ee_link=False`, `wrist_3_link=True`

## 6. TODO 清单与脚本编号说明

当前 `scripts/01_model_inspect.py` 采用 legacy step-script 风格，并保留中文教学 TODO 注释：

- **TODO 1-6 是脚本结构说明**：路径常量、默认输入输出、末端候选、CLI、输出路径规划、最终目标说明。
- **TODO 7-13 已补最小实现**：读取配置、解析 MJCF、加载模型、枚举对象、检查末端、保存 JSON、保存 Markdown。

这样做的原因是：先把脚本结构放在代码中，再把未来要补的核心功能按位置展开，避免只在文档里列需求。

### TODO 1: 路径常量

- 推荐 API: `Path(__file__).resolve()`, `Path.parents`, `sys.path.insert`
- 输入: 当前脚本路径 `__file__`
- 输出: `A_ROOT`, `REPO_ROOT`, `SRC_ROOT`
- 验证: 运行脚本，确认日志中的 A_ROOT / REPO_ROOT 正确

### TODO 2: 默认输入与输出路径

- 推荐 API: `pathlib.Path`, 路径 `/` 运算符
- 输入: `A_ROOT`
- 输出: `DEFAULT_CONFIG`, `DEFAULT_OUTPUT_DIR`, `DEFAULT_REPORT_FILE`, `DEFAULT_SUMMARY_FILE`
- 验证: 打印路径，确认位于 `projects/A_self_baseline/`

### TODO 3: 末端候选名称

- 推荐 API: Python list
- 输入: A01 约定的末端候选名称
- 输出: `END_EFFECTOR_CANDIDATES`
- 验证: 后续 TODO 11 中检查 `attachment_site` 和 `wrist_3_link`

### TODO 4: 解析 CLI 参数

- 推荐 API: `argparse.ArgumentParser`, `add_argument`
- 输入: `--config`, `--mjcf`, `--output-dir`, `--log-level`
- 输出: `argparse.Namespace`
- 验证: 运行 `python projects/A_self_baseline/scripts/01_model_inspect.py --help`

### TODO 5: 计算输出路径

- 推荐 API: `Path.expanduser`, `Path.is_absolute`, pathlib 路径拼接
- 输入: CLI `--output-dir`
- 输出: `{"report": Path, "summary_json": Path}`
- 验证: 默认路径应落在 `projects/A_self_baseline/outputs/`

### TODO 6: 打印 A01 最终目标

- 推荐 API: `logging.info`
- 输入: 无
- 输出: 日志说明
- 验证: 运行脚本能看到 nq/nv/nu、对象名称和末端候选目标

### TODO 7: 读取 robot.yaml

- 推荐 API: `model_loader.load_yaml_config`, `yaml.safe_load`, `Path.read_text`
- 输入: `args.config`
- 输出: `config: dict`
- 验证: 打印 config keys，确认包含 `mjcf_path` 和 `end_effector_candidates`

### TODO 8: 解析 mjcf_path

- 推荐 API: `model_loader.resolve_path`, `pathlib.Path`, `expanduser`, `resolve`
- 输入: `args.mjcf` 或 `config["mjcf_path"]`
- 输出: `mjcf_path: Path`
- 验证: 打印 `mjcf_path`，确认 `mjcf_path.exists()` 为 True

### TODO 9: 加载 MuJoCo model

- 推荐 API: `model_loader.load_mujoco_model`, `mujoco.MjModel.from_xml_path`
- 输入: `mjcf_path`
- 输出: MuJoCo `MjModel`
- 验证: 打印 `model.nq`, `model.nv`, `model.nu`，UR5e 预期为 `6/6/6`

### TODO 10: 枚举模型对象

- 推荐 API: `model_loader.get_mujoco_names`, `mujoco.mj_id2name`, `mujoco.mjtObj`
- 输入: MuJoCo `model`
- 输出: joint/body/site/actuator/keyframe 名称列表
- 验证: 名称列表长度与 `model.njnt`, `model.nbody`, `model.nsite`, `model.nu`, `model.nkey` 一致

### TODO 11: 检查末端候选

- 推荐 API: `model_loader.summarize_mujoco_model`, Python set/list membership
- 输入: `model`, `end_effector_candidates`
- 输出: `summary`, `end_effector_check`
- 验证: `attachment_site` 应命中 site，`wrist_3_link` 应命中 body

### TODO 12: 保存 JSON

- 推荐 API: `model_loader.write_json_summary`, `json.dumps`, `Path.write_text`
- 输入: 模型 summary dict
- 输出: `outputs/cache/A01_model_summary.json`
- 验证: 打开 JSON，确认字段完整

### TODO 13: 保存 Markdown

- 推荐 API: `model_loader.write_model_report`, `Path.write_text`, Markdown 列表
- 输入: 模型 summary dict
- 输出: `outputs/reports/A01_model_inspect_report.md`
- 验证: 报告可读，包含维度、对象名称和末端候选检查

## 7. 当前不做什么

当前 A01 明确不做：

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

下一步是 **A02：configuration / site pose**。

A02 只应补 `q -> MuJoCo data -> site pose` 数据流，推荐 API 包括 `mujoco.MjData`、`mujoco.mj_forward`、`data.site_xpos` 和 `data.site_xmat`。不要在 A02 扩展到 Jacobian、IK 或 QP。
