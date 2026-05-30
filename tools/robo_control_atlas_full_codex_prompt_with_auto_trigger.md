# RoboControl Atlas Codex Prompt

> 用途：把本文档交给 Codex，用于实现或维护 `tools/robot_atlas/`。
> 目标：构建一个面向 `Robot_Dynamics_Control` 的本地静态项目理解工具，服务机器人运动控制学习、B 项目推进、外部论文开源项目审读与复现规划。
> 当前版本：v0.35，包含 v0.1 内部项目管理、v0.2 外部 repo 审读、v0.3 Understand Anything 可选增强、v0.35 Auto Trigger / Command Router。

---

## 1. 仓库背景

当前仓库是机器人运动控制学习与求职项目仓库，主线包括：

```text
Robot_Dynamics_Control
├── projects/
│   ├── A_self_baseline
│   ├── B_mujoco_mpc_study
│   ├── C_openloong_dyn_control_study
│   └── D_legged_control_study
├── docs/00_project_management
├── shared/
│   ├── env
│   └── robot_assets
├── external/open_source_repos
└── tools/robot_atlas
```

`RoboControl Atlas` 的 CLI 入口为：

```bash
robot-atlas
```

第一版只做静态扫描、文本报告、提示词生成、验证计划生成、demo 产物检查和外部 repo 只读审读。不接入 LLM API，不运行长时间仿真，不自动修改主项目源码。

---

## 2. 最高优先级边界

### 2.1 只允许修改的区域

默认只允许新增或修改：

```text
tools/robot_atlas/
```

除非用户明确要求，不要修改：

```text
projects/A_self_baseline/
projects/B_mujoco_mpc_study/
projects/C_openloong_dyn_control_study/
projects/D_legged_control_study/
external/open_source_repos/
shared/robot_assets/
```

### 2.2 禁止动作

不要执行：

```bash
git add
git commit
git push
```

不要自动运行：

```text
长时间 MuJoCo 仿真
Pinocchio 大规模测试
外部 repo 训练脚本
外部 repo 安装脚本
下载网络资源
```

不要实现：

```text
外部 LLM API 接入
复杂图数据库
网页 Dashboard
MCP Server
自动代码修改 Agent
论文 PDF 自动解析
```

---

## 3. 版本范围

```text
v0.1 Owned Project Mode
- scan
- status
- next
- explain
- generate-codex
- validate
- demo-assets
- guards

v0.2 External Reproduction Mode
- read-repo
- entrypoints
- reproduce-plan
- port-to-B
- generate-codex-reproduce

v0.3 Understand Anything Optional Adapter
- import-understand
- understand-summary
- understand-entrypoints
- understand-port-to-B
- generate-codex-understand

v0.35 Auto Trigger / Command Router
- route "<user_request>"
```

v0.4 论文到代码映射暂不实现。

---

## 4. 目录结构

目标结构：

```text
tools/robot_atlas/
├── README.md
├── pyproject.toml
├── configs/
│   └── robot_dynamics_control.yaml
├── src/
│   └── robot_atlas/
│       ├── __init__.py
│       ├── cli.py
│       ├── models.py
│       ├── scanner.py
│       ├── status.py
│       ├── next_step.py
│       ├── explain.py
│       ├── codex_prompt.py
│       ├── validate.py
│       ├── demo_assets.py
│       ├── guards.py
│       ├── external_repo.py
│       ├── understand_adapter.py
│       └── router.py
└── tests/
    ├── test_scanner.py
    ├── test_status.py
    ├── test_next_step.py
    ├── test_explain.py
    ├── test_codex_prompt.py
    ├── test_validate.py
    ├── test_demo_assets.py
    ├── test_guards.py
    ├── test_external_repo.py
    ├── test_understand_adapter.py
    └── test_router.py
```

---

## 5. 包与配置

`pyproject.toml` 要求：

```toml
[build-system]
requires = ["setuptools>=68"]
build-backend = "setuptools.build_meta"

[project]
name = "robot-atlas"
version = "0.1.0"
description = "Local project understanding and reproduction planning tool for Robot_Dynamics_Control."
requires-python = ">=3.10"
dependencies = []

[project.optional-dependencies]
test = ["pytest"]

[project.scripts]
robot-atlas = "robot_atlas.cli:main"

[tool.setuptools.packages.find]
where = ["src"]
```

配置文件保留人类可读 YAML：`tools/robot_atlas/configs/robot_dynamics_control.yaml`。如果不引入 PyYAML，代码可以使用内置默认配置。

---

## 6. 核心数据模型

使用 `dataclasses.dataclass` 定义：

- `ProjectLine`：A/B/C/D 项目线扫描结果。
- `Stage`：内部阶段，如 `B03-R4C`。
- `ScanResult`：仓库扫描总结果。
- `DemoAssetReport`：demo 产物检查结果。
- `ExternalRepoReport`：外部 repo 静态审读结果。
- `UnderstandGraphSummary`：Understand Anything 图谱摘要。
- `RouteResult`：自然语言路由结果。

每个模型应提供简单文本输出方法，如 `summary()` 或 `as_text()`，用于 CLI 输出。

---

## 7. Owned Project Mode

### 7.1 `scan`

命令：

```bash
robot-atlas scan
```

职责：

- 扫描 A/B/C/D 项目线是否存在。
- 统计 Python 文件、测试文件、Markdown 文档。
- 统计 `TODO`、`todo`、`待实现`、`NotImplementedError`。
- 识别可能入口脚本，如 `run_*.py`、`demo_*.py`、`train_*.py`、`scripts/`、`examples/`。
- 忽略 `.git`、`__pycache__`、`.pytest_cache`、`.venv`、`external/open_source_repos`。

### 7.2 `status`

命令：

```bash
robot-atlas status B
```

B 项目固定语义：

- 当前阶段：`B03-R4C`
- 下一步：`B03-R4C-1B state tracking smoke run`
- 重点风险：仍偏 TODO 骨架，缺少 tracking error / cost / torque / video 等求职展示证据链。

其它项目线可以输出扫描摘要和可推进方向。

### 7.3 `next`

命令：

```bash
robot-atlas next B03
```

输出应说明：

- 当前阶段。
- 已完成内容。
- 下一步任务。
- 输入、输出、验收产物。
- 不应跳到 task-space tracking、WBC、RL 或完整控制器架构。

### 7.4 `explain`

命令：

```bash
robot-atlas explain iLQR
robot-atlas explain MPC
robot-atlas explain MPPI
robot-atlas explain CEM
robot-atlas explain QP
robot-atlas explain task-space tracking
robot-atlas explain two-link dynamics
```

输出结构：

```text
任务目标
算法原理
输入输出
核心公式
在当前项目中的位置
可视化产物
验证标准
TODO 骨架
禁止事项
```

### 7.5 `generate-codex`

命令：

```bash
robot-atlas generate-codex B03-R4C-1B
```

提示词必须强调：

- 只做当前任务。
- 保留学习型 TODO 和中文注释。
- 说明输入、输出、数学逻辑是否变化、参数变化和风险。
- 不修改外部 repo、模型资产或无关项目线。
- 不执行 git add/commit/push。

### 7.6 `validate` 和 `demo-assets`

命令：

```bash
robot-atlas validate B03
robot-atlas demo-assets B03
```

验证计划应包含：

- 单元测试。
- smoke run。
- tracking error / cost / torque 检查。
- 输出文件检查。
- 不能声称 demo 完成，除非图、日志、视频和复现实验命令都存在。

### 7.7 `guards`

命令：

```bash
robot-atlas guards
```

输出禁止事项和只读边界。

---

## 8. External Reproduction Mode

命令：

```bash
robot-atlas read-repo <repo_path>
robot-atlas entrypoints <repo_path>
robot-atlas reproduce-plan <repo_path>
robot-atlas port-to-B <repo_path>
robot-atlas generate-codex-reproduce <repo_path>
```

职责：

- 只读扫描外部 repo。
- 识别 README、入口脚本、配置文件、依赖文件、测试文件。
- 判断最小复现路线。
- 判断可迁移到 `projects/B_mujoco_mpc_study` 的模块。
- 生成只读 Codex 审读提示词。

如果用户只给 GitHub URL 而没有本地路径，不自动 clone，只提示需要先放到 `external/open_source_repos/`。

---

## 9. Understand Anything 可选增强

命令：

```bash
robot-atlas import-understand <repo_path>
robot-atlas understand-summary <repo_path>
robot-atlas understand-entrypoints <repo_path>
robot-atlas understand-port-to-B <repo_path>
robot-atlas generate-codex-understand <repo_path>
```

只读取已有文件：

```text
<repo_path>/.understand-anything/knowledge-graph.json
```

边界：

- 不自动安装 Understand Anything。
- 不自动运行 `/understand`。
- 不把图谱作为强依赖。
- 图谱缺失时返回清晰提示，不 traceback。
- 图谱 schema 不稳定时使用启发式解析，不能保证绝对正确。

启发式支持字段：

```text
nodes
edges
graph.nodes
graph.edges
knowledge_graph.nodes
knowledge_graph.edges
files
relationships
links
```

---

## 10. Auto Trigger / Command Router

新增命令：

```bash
robot-atlas route "<user_request>"
```

`route` 默认只推荐命令，不自动执行命令。

### 10.1 路由输出格式

```text
识别到的需求类型：
...

推荐命令：
robot-atlas ...

理由：
...

注意事项：
...
```

### 10.2 路由规则

```text
项目状态 / 进度                 -> robot-atlas scan 或 robot-atlas status B
B03 下一步                      -> robot-atlas next B03
明确任务 ID 的 Codex 提示词      -> robot-atlas generate-codex <task_id>
算法解释                        -> robot-atlas explain <topic>
验证计划                        -> robot-atlas validate B03
demo 产物检查                   -> robot-atlas demo-assets B03
安全边界 / 禁止事项             -> robot-atlas guards
外部 repo 审读                  -> read-repo / entrypoints / reproduce-plan <repo_path>
外部 repo 迁移到 B              -> robot-atlas port-to-B <repo_path>
外部 repo Codex 审读提示词       -> robot-atlas generate-codex-reproduce <repo_path>
Understand Anything 图谱审读     -> import-understand / understand-summary <repo_path>
Understand 迁移建议             -> robot-atlas understand-port-to-B <repo_path>
Understand Codex 提示词          -> robot-atlas generate-codex-understand <repo_path>
```

匹配多个场景时，优先级：

```text
P0 安全边界
P1 明确任务 ID 的 Codex 提示词
P2 当前项目状态 / 下一步
P3 验证计划 / demo assets
P4 外部 repo 审读 / 复现路线
P5 Understand Anything 图谱增强
P6 算法解释
```

缺少 `<repo_path>` 时，必须提示用户提供本地路径，不要假装已经扫描。

---

## 11. README 要求

`tools/robot_atlas/README.md` 至少包含：

- 工具定位。
- 安装方式。
- 所有 CLI 命令示例。
- Understand Anything 可选增强说明。
- Auto Trigger / Command Router 说明。
- 安全边界。
- 当前限制。

---

## 12. 测试要求

至少覆盖：

```text
test_scanner.py
test_status.py
test_next_step.py
test_explain.py
test_codex_prompt.py
test_validate.py
test_demo_assets.py
test_guards.py
test_external_repo.py
test_understand_adapter.py
test_router.py
```

`test_router.py` 需要覆盖：

```python
route_request("B03 下一步做什么")
route_request("生成 B03-R4C-1B 的 Codex 提示词")
route_request("iLQR 原理是什么")
route_request("这个 repo 怎么复现")
route_request("用 Understand Anything 图谱分析这个项目")
route_request("检查 B03 demo 产物")
route_request("检查禁止事项")
```

---

## 13. 推荐验证命令

```bash
pytest tools/robot_atlas/tests -q
```

安装后可手动验证：

```bash
cd tools/robot_atlas
pip install -e .

cd ../..
robot-atlas scan
robot-atlas status B
robot-atlas next B03
robot-atlas explain iLQR
robot-atlas generate-codex B03-R4C-1B
robot-atlas validate B03
robot-atlas demo-assets B03
robot-atlas guards
robot-atlas route "B03 下一步做什么？"
```

---

## 14. 最终汇报格式

Codex 完成后按以下格式汇报：

```text
1. 新增文件
2. 修改文件
3. 未修改的关键区域
4. 已实现命令
5. 各模块功能说明
6. Understand Anything 接入方式
7. Auto Trigger / Command Router 规则
8. 测试命令和结果
9. 当前限制
10. 下一步建议
11. 是否修改 external/open_source_repos，必须明确说明没有
12. 是否修改 shared/robot_assets，必须明确说明没有
13. 是否修改 projects/A-D 主项目源码，必须明确说明没有
14. 是否执行 git add/commit/push，必须明确说明没有
```

必须明确写：

```text
没有执行 git add。
没有执行 git commit。
没有执行 git push。
没有修改 external/open_source_repos。
没有修改 shared/robot_assets。
没有修改 projects/A-D 主项目源码。
```
