# RoboControl Atlas 完整 Codex 实现提示词

> 本文档合并了两个任务文档：
>
> 1. `RoboControl Atlas v0.1 + v0.2` 主工具实现提示词
> 2. `RoboControl Atlas × Understand Anything` 可选增强模块提示词
>
> 目标：让 Codex 在 `/home/ubuntu/Robot_Dynamics_Control` 中实现一个面向机器人运动控制学习 / 求职项目 / 外部论文开源项目复现的本地项目理解工具。
>
> 合并日期：2026-05-27 08:55:18

---

## 总体版本规划

```text
RoboControl Atlas
├── v0.1 内部项目管理
│   ├── scan
│   ├── status
│   ├── next
│   ├── explain
│   ├── generate-codex
│   ├── validate
│   ├── demo-assets
│   └── guards
│
├── v0.2 外部论文 / 开源项目快速审读
│   ├── read-repo
│   ├── entrypoints
│   ├── reproduce-plan
│   ├── port-to-B
│   └── generate-codex-reproduce
│
├── v0.3 Understand Anything 可选增强
│   ├── import-understand
│   ├── understand-summary
│   ├── understand-entrypoints
│   ├── understand-port-to-B
│   └── generate-codex-understand
│
└── v0.4 论文到代码映射，后续路线，本次不实现
    ├── paper-map
    ├── algorithm-map
    ├── equation-map
    └── experiment-map
```

---

## 使用方式建议

把本文档完整交给 Codex。建议先实现 v0.1 + v0.2，如果主工具测试通过，再实现 v0.3 Understand Anything 可选增强模块。

也可以一次性交给 Codex，但必须要求它：

```text
1. 只修改 tools/robot_atlas/
2. 不修改 projects/A-D 主项目源码
3. 不修改 external/open_source_repos
4. 不修改 shared/robot_assets
5. 不执行 git add / git commit / git push
6. 不运行长时间仿真
7. 不下载网络资源
8. 不自动安装或运行 Understand Anything
```

---

# Part 1：RoboControl Atlas v0.1 + v0.2 主工具实现提示词

# RoboControl Atlas v0.1 + v0.2 Codex 实现提示词

> 用途：把本文件完整复制给 Codex，让 Codex 在 `Robot_Dynamics_Control` 仓库内实现一个专用工具：`RoboControl Atlas`。
> 目标：服务机器人运动控制学习 / 求职项目，同时支持外部论文开源项目快速审读和复现路线生成。
> 重要边界：本次只实现工具骨架、静态扫描、报告生成、提示词生成、验证计划生成；不接入 LLM API，不做 Dashboard，不运行长时间仿真，不修改第三方源码。

---

## 0. 当前仓库与任务背景

你现在位于仓库：

```bash
Robot_Dynamics_Control
```

这是一个机器人运动控制学习和求职项目仓库，当前重点包括：

```text
Robot_Dynamics_Control
├── projects/
│   ├── A_self_baseline
│   ├── B_mujoco_mpc_study
│   ├── C_openloong_dyn_control_study
│   └── D_legged_control_study
├── docs/
│   └── 00_project_management
├── shared/
│   ├── env
│   └── robot_assets
└── external/
    └── open_source_repos
```

本次要实现的工具名称：

```text
RoboControl Atlas
```

命令行入口名称：

```bash
robot-atlas
```

工具有两个模式：

```text
Mode A：Owned Project Mode
用于管理当前仓库中的 A/B/C/D 机器人运动控制项目线。

Mode B：External Reproduction Mode
用于快速审读外部论文开源项目，生成入口分析、复现路线、可迁移模块分析和 Codex 复现提示词。
```

---

## 1. 总体目标

实现一个本地静态项目理解工具，第一版不追求复杂知识图谱，而是优先解决以下实际问题：

### 1.1 对内部项目线的支持

能够回答：

```bash
robot-atlas scan
robot-atlas status B
robot-atlas next B03
robot-atlas explain iLQR
robot-atlas generate-codex B03-R4C-1B
robot-atlas validate B03
robot-atlas demo-assets B03
robot-atlas guards
```

对应能力：

1. 扫描 `/home/ubuntu/Robot_Dynamics_Control`。
2. 识别 A/B/C/D 四条项目线是否存在。
3. 统计 Python 文件、测试文件、Markdown 文档、TODO、`NotImplementedError`。
4. 判断 B 项目当前状态。
5. 判断 B03 下一步任务。
6. 按固定学习笔记格式解释 iLQR、MPC、MPPI 等机器人控制算法。
7. 生成适合 Codex 执行的任务提示词。
8. 生成验证计划。
9. 检查 demo 产物是否完整。
10. 提醒禁止修改第三方源码、禁止 git add/commit/push 等边界。

### 1.2 对外部论文 / 开源项目复现的支持

能够回答：

```bash
robot-atlas read-repo <external_repo_path>
robot-atlas entrypoints <external_repo_path>
robot-atlas reproduce-plan <external_repo_path>
robot-atlas port-to-B <external_repo_path>
robot-atlas generate-codex-reproduce <external_repo_path>
```

对应能力：

1. 快速读取外部 repo 的 README、入口脚本、配置文件、依赖文件、测试文件。
2. 判断项目可能的运行入口。
3. 生成复现路线。
4. 判断哪些模块适合迁移到 `B_mujoco_mpc_study`。
5. 生成只读分析外部 repo 的 Codex 提示词。
6. 明确外部 repo 默认只读，不直接修改第三方源码。

---

## 2. 最高优先级安全边界

严格遵守以下边界。

### 2.1 禁止执行的动作

不要执行：

```bash
git add
git commit
git push
```

不要自动运行：

```bash
长时间 MuJoCo 仿真
Pinocchio 大规模测试
下载网络资源
外部仓库训练脚本
外部仓库安装脚本
```

不要接入：

```text
OpenAI API
Claude API
Gemini API
任何外部 LLM API
```

不要实现：

```text
复杂图数据库
网页 Dashboard
MCP Server
自动代码修改 Agent
云端同步
论文 PDF 自动解析
```

### 2.2 禁止修改的目录

本次只允许新增或修改：

```text
tools/robot_atlas/
```

默认不要修改根 README。
如确实需要修改根 README，只能追加一小段工具说明，但本任务默认不需要。

禁止修改：

```text
external/open_source_repos/
shared/robot_assets/
projects/A_self_baseline/
projects/B_mujoco_mpc_study/
projects/C_openloong_dyn_control_study/
projects/D_legged_control_study/
```

特别说明：

```text
projects/A-D 是主项目线，只有明确任务要求时才能修改。
external/open_source_repos 是第三方源码，默认只读。
shared/robot_assets 是第三方模型资源，默认只读。
```

### 2.3 输出边界

本次工具只能做：

```text
静态扫描
文本报告
提示词生成
验证计划生成
demo assets 缺失检查
外部 repo 审读报告
```

不能做：

```text
实际修改主项目源码
自动修复项目
自动运行仿真
自动下载数据
自动提交 git
```

---

## 3. 必须创建的目录结构

请创建：

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
│       └── external_repo.py
└── tests/
    ├── test_scanner.py
    ├── test_status.py
    ├── test_next_step.py
    ├── test_explain.py
    ├── test_codex_prompt.py
    ├── test_validate.py
    ├── test_demo_assets.py
    ├── test_guards.py
    └── test_external_repo.py
```

---

## 4. `pyproject.toml` 要求

文件路径：

```text
tools/robot_atlas/pyproject.toml
```

要求：

1. Python 版本 `>=3.10`。
2. 包名使用：

```text
robot-atlas
```

3. CLI 入口：

```toml
[project.scripts]
robot-atlas = "robot_atlas.cli:main"
```

4. 尽量使用标准库，不要引入不必要依赖。
5. 不要强依赖 PyYAML。如果为了配置读取需要 YAML，可以实现一个非常简单的配置 fallback，或者将配置同时支持 JSON。
6. 测试依赖只需 pytest。

推荐结构：

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

---

## 5. 配置文件要求

文件路径：

```text
tools/robot_atlas/configs/robot_dynamics_control.yaml
```

必须包含下面信息：

```yaml
repo:
  root: /home/ubuntu/Robot_Dynamics_Control

index_paths:
  project_lines:
    - projects/A_self_baseline
    - projects/B_mujoco_mpc_study
    - projects/C_openloong_dyn_control_study
    - projects/D_legged_control_study
  docs:
    - docs/00_project_management
  shared:
    - shared/env
    - shared/robot_assets

ignore_paths:
  - .git
  - __pycache__
  - .pytest_cache
  - .venv
  - external/open_source_repos

project_lines:
  A:
    name: A_self_baseline
    goal: 自研 baseline 学习与 demo
  B:
    name: B_mujoco_mpc_study
    goal: MuJoCo MPC / iLQR / iLQG demo
  C:
    name: C_openloong_dyn_control_study
    goal: OpenLoong-Dyn-Control study
  D:
    name: D_legged_control_study
    goal: legged-control study

required_demo_assets:
  B03:
    figures:
      - tracking_error.png
      - cost_history.png
      - torque_profile.png
    videos:
      - state_tracking_smoke.mp4
    logs:
      - smoke_summary.json
    docs:
      - stepB03R4C1B_state_tracking_smoke_run.md

forbidden_actions:
  - git add
  - git commit
  - git push

forbidden_paths:
  - external/open_source_repos
  - shared/robot_assets/vendor
```

如果不使用 PyYAML，可以：

1. 保留此 YAML 作为人类可读配置。
2. 在代码中使用内置默认配置。
3. 后续再扩展 YAML parser。

---

## 6. `models.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/models.py
```

使用 `dataclasses.dataclass` 定义数据模型。

### 6.1 `ProjectLine`

表示 A/B/C/D 中的一条项目线。

字段：

```python
id: str
name: str
path: Path
goal: str
exists: bool
python_files: list[Path]
test_files: list[Path]
markdown_files: list[Path]
todo_count: int
not_implemented_count: int
entrypoint_files: list[Path]
```

含义：

- `id`：A、B、C、D。
- `name`：项目目录名，如 `B_mujoco_mpc_study`。
- `path`：项目路径。
- `goal`：项目目标。
- `exists`：目录是否存在。
- `python_files`：扫描到的 `.py` 文件。
- `test_files`：`test_*.py` 或 `*_test.py`。
- `markdown_files`：`.md` 文件。
- `todo_count`：TODO / todo / 待实现数量。
- `not_implemented_count`：`NotImplementedError` 数量。
- `entrypoint_files`：可能的运行入口脚本。

建议方法：

```python
def summary(self) -> str:
    ...
```

输出一段简短摘要。

---

### 6.2 `Stage`

表示内部项目阶段，例如 B03-R4C。

字段：

```python
id: str
project_line: str
title: str
status: str
docs: list[Path]
related_files: list[Path]
next_step: str
```

含义：

- `id`：阶段 ID，如 `B03-R4C`。
- `project_line`：所属项目线，如 `B`。
- `title`：阶段标题。
- `status`：`completed` / `in_progress` / `pending` / `unknown`。
- `docs`：相关文档。
- `related_files`：相关代码文件。
- `next_step`：下一步建议。

---

### 6.3 `ScanResult`

表示整个仓库扫描结果。

字段：

```python
repo_root: Path
project_lines: dict[str, ProjectLine]
docs_count: int
stage_docs: list[Path]
total_todos: int
total_not_implemented: int
warnings: list[str]
```

含义：

- `repo_root`：仓库根目录。
- `project_lines`：A/B/C/D 扫描结果。
- `docs_count`：`docs/00_project_management` 下 Markdown 数量。
- `stage_docs`：阶段文档列表。
- `total_todos`：总 TODO 数。
- `total_not_implemented`：总 `NotImplementedError` 数。
- `warnings`：扫描警告。

建议方法：

```python
def as_text(self) -> str:
    ...
```

---

### 6.4 `DemoAssetReport`

表示 demo 产物检查结果。

字段：

```python
stage: str
required: list[str]
existing: list[Path]
missing: list[str]
conclusion: str
```

用途：

- 检查 B03 是否有视频、图、日志、README reproduction command 等。
- 即使文件缺失，也不能抛异常，应输出 missing list。

---

### 6.5 `ExternalRepoReport`

表示外部 repo 审读结果。

字段：

```python
path: Path
name: str
main_language_guess: str
readme_files: list[Path]
entrypoints: list[Path]
config_files: list[Path]
dependency_files: list[Path]
test_files: list[Path]
robot_related_dirs: list[Path]
reproduce_risks: list[str]
porting_candidates: list[str]
```

含义：

- `path`：外部 repo 路径。
- `name`：repo 名称。
- `main_language_guess`：语言初判，如 Python / C++ / TypeScript / Mixed / Unknown。
- `readme_files`：README 文件。
- `entrypoints`：入口脚本。
- `config_files`：配置文件。
- `dependency_files`：依赖文件。
- `test_files`：测试文件。
- `robot_related_dirs`：机器人相关目录。
- `reproduce_risks`：复现风险。
- `porting_candidates`：可迁移候选模块。

建议方法：

```python
def as_text(self) -> str:
    ...
```

---

## 7. `scanner.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/scanner.py
```

### 7.1 模块职责

负责对当前主仓库做静态扫描。

不要 import 或运行主项目源码。
不要运行 MuJoCo、Pinocchio、pytest 或任何仿真脚本。
只做文件层扫描和文本统计。

### 7.2 必须实现函数

#### 7.2.1 `default_repo_root() -> Path`

返回默认仓库根目录：

```python
Path("/home/ubuntu/Robot_Dynamics_Control")
```

如果当前工作目录就是仓库，也可以优先用当前目录判断：

```python
Path.cwd()
```

建议逻辑：

1. 如果当前目录下存在 `projects/` 和 `docs/00_project_management/`，使用当前目录。
2. 否则使用 `/home/ubuntu/Robot_Dynamics_Control`。

---

#### 7.2.2 `is_ignored(path: Path, ignore_names: set[str]) -> bool`

判断路径是否应该忽略。

忽略：

```text
.git
__pycache__
.pytest_cache
.venv
external/open_source_repos
```

注意：

- 如果路径任意父级包含这些名字，也应该忽略。
- 不要进入外部仓库源码扫描内部项目。

---

#### 7.2.3 `collect_files(root: Path, suffixes: tuple[str, ...], ignore_names: set[str]) -> list[Path]`

递归收集文件。

要求：

- 使用 `Path.rglob` 或手写递归。
- 忽略 ignore path。
- 如果目录不存在，返回空列表。
- 不抛异常。

---

#### 7.2.4 `count_text_markers(files: list[Path], markers: list[str]) -> int`

统计文本标记出现次数。

markers 包括：

```text
TODO
todo
待实现
NotImplementedError
```

要求：

- 使用 `encoding="utf-8"`。
- 遇到编码错误时使用 `errors="ignore"`。
- 不因为单个文件读取失败而中断。

---

#### 7.2.5 `is_test_file(path: Path) -> bool`

判断测试文件：

```text
test_*.py
*_test.py
```

---

#### 7.2.6 `is_entrypoint_file(path: Path) -> bool`

判断入口脚本。

规则：

文件名匹配：

```text
main.py
run.py
run_*.py
demo.py
demo_*.py
train.py
train_*.py
eval.py
eval_*.py
test.py
```

或位于目录：

```text
scripts/
examples/
```

并且后缀是 `.py`。

---

#### 7.2.7 `scan_project_line(repo_root: Path, project_id: str, relative_path: str, goal: str) -> ProjectLine`

扫描单条项目线。

对于每个项目线：

1. 判断目录是否存在。
2. 收集 `.py` 文件。
3. 收集 `.md` 文件。
4. 找测试文件。
5. 找入口脚本。
6. 统计 TODO。
7. 统计 `NotImplementedError`。
8. 返回 `ProjectLine`。

---

#### 7.2.8 `scan_repo(repo_root: Path | None = None) -> ScanResult`

扫描整个仓库。

必须扫描：

```text
projects/A_self_baseline
projects/B_mujoco_mpc_study
projects/C_openloong_dyn_control_study
projects/D_legged_control_study
docs/00_project_management
```

内置 project lines 配置：

```python
{
    "A": ("A_self_baseline", "projects/A_self_baseline", "自研 baseline 学习与 demo"),
    "B": ("B_mujoco_mpc_study", "projects/B_mujoco_mpc_study", "MuJoCo MPC / iLQR / iLQG demo"),
    "C": ("C_openloong_dyn_control_study", "projects/C_openloong_dyn_control_study", "OpenLoong-Dyn-Control study"),
    "D": ("D_legged_control_study", "projects/D_legged_control_study", "legged-control study"),
}
```

返回 `ScanResult`。

---

### 7.3 scan 命令输出内容

`robot-atlas scan` 应输出：

```text
RoboControl Atlas Scan

仓库根目录：
...

项目线：
A - ...
B - ...
C - ...
D - ...

统计：
- docs/00_project_management 文档数量
- TODO 总数
- NotImplementedError 总数

警告：
...
```

---

## 8. `status.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/status.py
```

### 8.1 模块职责

根据扫描结果和内置项目知识，生成项目线状态报告。

第一版重点支持：

```bash
robot-atlas status B
```

其它项目线 A/C/D 可以输出基础扫描摘要。

### 8.2 必须实现函数

#### 8.2.1 `build_status(project_id: str, scan_result: ScanResult) -> str`

输入：

```python
project_id: "A" | "B" | "C" | "D"
scan_result: ScanResult
```

输出字符串。

### 8.3 B 项目内置状态

当 `project_id == "B"`，必须输出以下内容：

```text
项目目标：
MuJoCo MPC / iLQR / iLQG 学习与求职 demo，要求有真实可运行仿真、视频、图表和复现实验命令。

当前阶段：
B03-R4C

已完成内容：
- B03-R4C-1A 已完成
- 已保留 TODO 教学骨架
- 已新增 two-link dynamics adapter contract
- 已有 smoke script 和 skeleton test

未完成内容：
- iLQR-lite 尚未真正接入 B02 two-link dynamics
- state tracking smoke run 尚未完成
- task-space tracking 尚未完成
- CEM/MPPI warm-start 尚未完成
- MP4/GIF demo 产物不足

核心文件：
- projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py
- projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py
- docs/00_project_management/stepB03R4C1A_preserve_skeleton_adapter_contract.md

当前风险：
- 当前仍偏 TODO 骨架，不是完整 demo
- 还不能作为求职展示项目最终版本
- 如果没有 tracking error / cost / torque / video，则 B03 证据链不足

下一步建议：
进入 B03-R4C-1B：实现最小 state tracking smoke run。
```

同时结合扫描结果补充：

```text
扫描补充：
- Python 文件数量
- 测试文件数量
- Markdown 文档数量
- TODO 数量
- NotImplementedError 数量
```

如果关键文件不存在，输出：

```text
需要人工确认：
- 未扫描到 xxx
```

### 8.4 A/C/D 项目输出结构

对于 A/C/D，输出：

```text
项目目标
扫描摘要
可能入口文件
测试文件
TODO / NotImplementedError 数量
当前风险
下一步建议
```

如果没有内置阶段知识，就写：

```text
当前阶段需要根据 docs/00_project_management 进一步确认。
```

---

## 9. `next_step.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/next_step.py
```

### 9.1 模块职责

根据阶段 ID，生成下一步任务说明。

第一版重点支持：

```bash
robot-atlas next B03
```

### 9.2 必须实现函数

#### 9.2.1 `build_next_step(stage_id: str) -> str`

输入：

```python
stage_id: str
```

输出：阶段下一步报告。

### 9.3 B03 固定输出结构

当输入 `B03` 时，输出结构必须为：

```text
当前阶段定位
上一阶段结果
下一步任务
为什么做这一步
输入输出
需要修改文件
需要新增文件
验证标准
禁止事项
```

### 9.4 B03 内容要求

必须包含：

```text
当前阶段：
B03 = iLQR / iLQG-lite 控制器接入 MuJoCo demo。

上一阶段结果：
B03-R4C-1A 已完成，只做了最小接口契约，没有完整实现控制闭环。

下一步任务：
B03-R4C-1B：state tracking smoke run。

为什么做这一步：
因为在进入 task-space tracking 和 CEM/MPPI warm-start 之前，必须先证明：
- two-link dynamics 可以被 iLQR-lite 调用
- state trajectory 可以被 rollout
- tracking error / cost / torque 可以被记录
- 最小 demo 可以稳定运行

输入：
- two-link 初始状态 x0
- 目标状态轨迹 x_ref
- 初始控制序列 u_init
- dynamics function f(x, u)
- cost weights Q, R, Qf
- horizon T
- time step dt

输出：
- optimized state trajectory
- optimized control trajectory
- cost history
- tracking error figure
- torque figure
- smoke run log
- 可选 MP4/GIF 仿真视频

需要修改文件：
- projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py
- projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py

可能新增文件：
- docs/00_project_management/stepB03R4C1B_state_tracking_smoke_run.md
- projects/B_mujoco_mpc_study/outputs/B03_R4C_1B/...

验证标准：
- pytest 通过
- smoke run 可以完成
- cost 不出现 NaN
- tracking error 可计算
- torque 曲线可输出
- 图表文件存在
- summary 文件存在

禁止事项：
- 不要 git add / commit / push
- 不要删除 TODO 教学注释
- 不要直接跳到 task-space tracking
- 不要引入真实硬件接口
- 不要修改 external/open_source_repos
- 不要修改 shared/robot_assets 第三方模型资源
```

### 9.5 未知 stage 输出

如果输入不是 B03，输出：

```text
当前暂未内置该阶段的详细路线。
建议先使用 robot-atlas scan 和 robot-atlas status <ProjectLine> 查看项目状态。
```

---

## 10. `explain.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/explain.py
```

### 10.1 模块职责

按用户学习机器人运动控制的固定格式解释算法概念。

第一版支持：

```text
iLQR
iLQG
iLQG-lite
MPC
CEM
MPPI
QP
task-space tracking
two-link dynamics
```

### 10.2 必须实现函数

#### 10.2.1 `explain_algorithm(name: str) -> str`

输入算法名，大小写不敏感，支持别名。

例如：

```python
explain_algorithm("ilqr")
explain_algorithm("iLQR")
explain_algorithm("iLQG-lite")
```

输出固定结构：

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

---

### 10.3 iLQR 解释要求

必须包含以下关键词：

```text
rollout
dynamics linearization
cost quadratic approximation
Riccati backward pass
forward rollout update
state trajectory
control trajectory
cost history
feedback gains
feedforward terms
tracking error
```

内容示例要求：

```text
任务目标：
在有限时间范围内，为非线性动力学系统寻找一组控制序列，使状态轨迹尽量跟踪目标轨迹，同时控制输入不要过大。

算法原理：
iLQR 每一轮做三件事：
1. 用当前控制序列 rollout 得到 nominal trajectory。
2. 沿 nominal trajectory 对动力学做局部线性化，对代价做二次近似。
3. 通过 Riccati backward pass 求局部反馈控制律，再 forward rollout 更新轨迹。

输入：
- x0
- u_init[0:T-1]
- f(x, u)
- x_ref[0:T]
- Q, R, Qf
- max_iter

输出：
- x_traj[0:T]
- u_traj[0:T-1]
- cost_history
- feedback gains K_t
- feedforward terms k_t
```

核心公式至少写：

```text
x_{t+1} = f(x_t, u_t)
δx_{t+1} ≈ A_t δx_t + B_t δu_t
l_t = (x_t - x_ref)^T Q (x_t - x_ref) + u_t^T R u_t
```

说明公式含义，不要求复杂推导。

---

### 10.4 MPC 解释要求

必须说明：

```text
receding horizon
每个控制周期重新优化
只执行第一步控制
与 iLQR 的关系
适合在线控制
```

输出中必须包含：

```text
预测窗口
滚动优化
控制频率
仿真频率
warm-start
```

---

### 10.5 CEM 解释要求

必须说明：

```text
sampling-based optimization
从高斯分布采样控制序列
选择 elite samples
更新均值和方差
可作为 MPC 或 iLQR warm-start
```

---

### 10.6 MPPI 解释要求

必须说明：

```text
sampling-based MPC
path integral control
噪声扰动控制序列
按轨迹 cost 加权
适合非线性系统
```

---

### 10.7 QP 解释要求

必须说明：

```text
quadratic programming
二次目标
线性约束
机器人控制中的 WBC / inverse dynamics / MPC 常用
OSQP / qpOASES / HPIPM
```

---

### 10.8 task-space tracking 解释要求

必须说明：

```text
关节空间 tracking 和任务空间 tracking 的区别
末端位置 / 姿态 / CoM / foot tracking
需要 FK / Jacobian
误差从 task space 映射到 joint/control
```

---

### 10.9 two-link dynamics 解释要求

必须说明：

```text
二连杆系统状态
q, qdot
控制输入 torque
连续动力学
离散动力学
MuJoCo env 与 iLQR dynamics function 的接口
```

---

## 11. `codex_prompt.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/codex_prompt.py
```

### 11.1 模块职责

生成适合用户当前工作流的 Codex 提示词。

要求生成的提示词：

1. 可以直接复制给 Codex。
2. 明确当前仓库路径。
3. 明确任务目标。
4. 明确允许修改文件。
5. 明确禁止修改文件。
6. 强调中文 TODO 教学注释。
7. 强调不要 git add / commit / push。
8. 强调不要跳阶段。
9. 强调输出测试结果和最终汇报格式。

### 11.2 必须实现函数

#### 11.2.1 `generate_codex_prompt(task_id: str) -> str`

第一版至少支持：

```text
B03-R4C-1B
```

未知 task 输出：

```text
当前暂未内置该任务的 Codex 提示词。
请先使用 robot-atlas next <stage> 查看任务路线。
```

---

### 11.3 B03-R4C-1B 提示词必须包含的章节

输出内容必须包含以下标题：

```text
任务背景
当前状态
本次目标
允许修改文件
禁止修改文件
实现要求
代码风格要求
中文 TODO 注释要求
测试要求
可视化要求
文档要求
验收标准
禁止事项
最终汇报格式
```

---

### 11.4 B03-R4C-1B 提示词内容要求

必须明确：

```text
任务名称：
B03-R4C-1B：将 iLQR-lite 接入 B02 two-link dynamics，完成最小 state tracking smoke run。
```

必须包含当前背景：

```text
B03-R4C-1A 已完成。它保留了 TODO 教学骨架，只新增了最小 adapter contract。
当前目标不是重构整个项目，也不是直接进入 task-space tracking，而是完成一个最小可运行的 state tracking smoke run。
```

必须包含本次目标：

```text
1. 使用已有 TwoLinkEnv / two-link dynamics 接口构造离散动力学函数。
2. 构造一个简单 state tracking problem。
3. 调用 ILQGLiteSolver.solve(problem) 或当前项目中已有的 solver 接口。
4. 输出 state trajectory、control trajectory、cost history。
5. 生成 tracking error、cost、torque 图。
6. 保存 smoke run summary。
7. 增加或更新 pytest，验证最小流程可运行。
```

允许修改：

```text
- projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py
- projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py
- docs/00_project_management/stepB03R4C1B_state_tracking_smoke_run.md
```

如确有必要，可以最小修改：

```text
- projects/B_mujoco_mpc_study/simulator/planners/ilqg_solver.py
- projects/B_mujoco_mpc_study/simulator/envs/ 与 two-link 相关文件
```

禁止修改：

```text
- external/open_source_repos/
- shared/robot_assets/ 中的第三方模型文件
- 与 B03 无关的 A/C/D 项目文件
- git 历史、远程仓库、commit 信息
```

必须包含可视化要求：

```text
至少输出：
- tracking_error.png
- cost_history.png
- torque_profile.png

如果当前环境支持渲染，可以输出：
- state_tracking_smoke.mp4

如果暂时不能渲染视频，需要在 summary 中明确说明原因，并保留视频输出接口。
```

测试要求：

```text
新增或更新 pytest，至少检查：
1. smoke script 可以 import。
2. dynamics adapter 返回维度正确。
3. state tracking problem 字段完整。
4. smoke run 在很小 horizon 下可以执行。
5. 输出 summary 文件存在。
6. 图表文件存在或在 headless 模式下被合理跳过。
```

运行验证：

```bash
pytest projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py -q
```

禁止事项必须包括：

```text
- 不要 git add
- 不要 git commit
- 不要 git push
- 不要删除 TODO 教学注释
- 不要重构整个 B 项目
- 不要改 external/open_source_repos
- 不要声称完成完整 MPC
- 不要声称完成求职最终 demo
```

最终汇报格式必须包括：

```text
1. 修改 / 新增文件
2. 保留的 TODO 骨架
3. 本次实现的最小功能
4. 生成的图表 / 日志 / 视频
5. 运行的测试命令和结果
6. 未完成内容
7. 下一步建议
8. 是否执行 git add/commit/push，必须明确说明没有执行
```

---

## 12. `validate.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/validate.py
```

### 12.1 模块职责

生成阶段验证计划。
注意：只输出验证命令，不自动执行。

### 12.2 必须实现函数

#### 12.2.1 `build_validation_plan(stage_id: str) -> str`

第一版至少支持：

```text
B03
```

输出结构固定：

```text
基础检查
单元测试
脚本 smoke run
图表检查
视频检查
文档检查
禁止事项检查
验收结论
```

### 12.3 B03 验证计划内容

必须包含：

#### 基础检查

```text
- 检查 projects/B_mujoco_mpc_study 是否存在
- 检查 simulator/scripts 是否有 B03 入口脚本
- 检查 simulator/planners 是否有 iLQR/iLQG solver
- 检查 tests 是否有 B03 测试
```

#### 单元测试

```bash
pytest projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py -q
```

#### 脚本 smoke run

```bash
python projects/B_mujoco_mpc_study/simulator/scripts/run_B03_ilqr_lite_two_link_smoke.py \
  --config projects/B_mujoco_mpc_study/configs/B03_ilqg_lite.yaml \
  --output-dir projects/B_mujoco_mpc_study/outputs/B03_R4C_1B
```

#### 图表检查

```text
应该存在：
- tracking_error.png
- cost_history.png
- torque_profile.png
```

#### 视频检查

```text
推荐存在：
- state_tracking_smoke.mp4

如果没有视频，必须说明是 headless 渲染问题还是尚未实现。
```

#### 文档检查

```text
应该存在：
- docs/00_project_management/stepB03R4C1B_state_tracking_smoke_run.md
```

#### 禁止事项检查

```text
确认：
- 没有 git add/commit/push
- 没有修改 external/open_source_repos
- 没有删除 TODO 教学注释
- 没有跳到 task-space tracking
- 没有引入硬件接口
```

#### 验收结论

```text
只有测试、日志、图表、文档都存在，才算 B03-R4C-1B 完成。
```

---

## 13. `demo_assets.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/demo_assets.py
```

### 13.1 模块职责

检查某个阶段是否已有求职展示所需 demo 产物。

### 13.2 必须实现函数

#### 13.2.1 `build_demo_assets_report(stage_id: str, repo_root: Path) -> str`

第一版至少支持：

```text
B03
```

输出结构固定：

```text
必需产物
当前已有
当前缺失
求职展示可用性
下一步补齐建议
```

### 13.3 B03 必需产物

必需：

```text
- MP4 或 GIF 仿真视频
- tracking error figure
- cost figure
- torque figure
- runtime summary
- smoke run log
- README reproduction command
- pytest 结果
```

推荐：

```text
- 参数对比表
- 不同 horizon 对比
- 不同权重 Q/R 对比
- baseline vs iLQR 对比
```

### 13.4 文件检查规则

检查路径可以包括：

```text
projects/B_mujoco_mpc_study/outputs/
projects/B_mujoco_mpc_study/outputs/videos/
projects/B_mujoco_mpc_study/outputs/figures/
projects/B_mujoco_mpc_study/outputs/logs/
projects/B_mujoco_mpc_study/outputs/B03_R4C_1B/
docs/00_project_management/
```

通过文件名关键词判断：

视频：

```text
*.mp4
*.gif
```

tracking error 图：

```text
tracking
error
```

cost 图：

```text
cost
```

torque 图：

```text
torque
control
u_profile
```

runtime summary：

```text
runtime
summary
```

smoke run log：

```text
smoke
log
summary
json
```

pytest 结果：

```text
pytest
test
```

文档：

```text
stepB03R4C1B
state_tracking_smoke
```

### 13.5 结论规则

如果缺少视频、tracking error 图、cost 图、torque 图：

```text
当前不可作为完整求职 demo，只能作为开发中项目骨架展示。
```

如果全部必需产物存在：

```text
当前具备初步求职 demo 展示条件，但仍建议补充参数对比和 baseline 对比。
```

---

## 14. `guards.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/guards.py
```

### 14.1 模块职责

输出安全边界检查报告。
不做实际 git 操作，不修改任何文件。

### 14.2 必须实现函数

#### 14.2.1 `run_guard_checks(repo_root: Path) -> str`

检查并输出：

1. `external/open_source_repos` 是否存在。
2. `shared/robot_assets` 是否存在。
3. `tools/robot_atlas` 是否存在。
4. `.git` 是否存在。
5. 禁止事项清单。
6. 外部仓库默认只读。
7. `shared/robot_assets` 默认只读。
8. `projects/A-D` 是主项目线，修改要通过明确任务。
9. 本工具不会执行 `git add/commit/push`。
10. 本工具不会自动运行长时间仿真。

### 14.3 输出结构

固定为：

```text
仓库边界检查
关键目录状态
禁止动作
默认只读区域
主项目线修改规则
工具自身行为边界
建议
```

---

## 15. `external_repo.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/external_repo.py
```

### 15.1 模块职责

实现外部论文 / 开源项目快速审读模式。

注意：

- 外部 repo 默认只读。
- 不修改外部 repo。
- 不执行外部 repo 的安装脚本。
- 不运行外部 repo 的训练或 demo。
- 只做静态扫描和报告生成。

### 15.2 必须实现函数

#### 15.2.1 `read_external_repo(path: Path) -> ExternalRepoReport`

扫描外部 repo，并返回 `ExternalRepoReport`。

必须识别：

README：

```text
README
README.md
readme.md
docs/README.md
```

入口文件：

```text
main.py
run.py
run_*.py
demo.py
demo_*.py
train.py
train_*.py
eval.py
eval_*.py
test.py
```

入口目录：

```text
scripts/
examples/
demo/
demos/
```

配置文件：

```text
configs/
config/
*.yaml
*.yml
*.json
*.toml
```

依赖文件：

```text
requirements.txt
environment.yml
environment.yaml
pyproject.toml
setup.py
setup.cfg
package.json
Dockerfile
docker-compose.yml
```

测试文件：

```text
tests/
test_*.py
*_test.py
```

机器人相关目录：

```text
envs
controllers
planners
models
robots
assets
urdf
mjcf
sim
mujoco
pinocchio
isaac
gym
tasks
policies
```

复现风险：

```text
缺 README
缺依赖文件
缺 demo
缺配置文件
缺测试
可能需要 GPU
可能需要大数据集
可能需要第三方仿真资产
可能需要真实机器人硬件
```

语言初判：

- `.py` 多：Python
- `.cpp/.hpp/.cc` 多：C++
- `.ts/.js` 多：TypeScript/JavaScript
- 混合：Mixed
- 否则：Unknown

---

#### 15.2.2 `build_entrypoints_report(path: Path) -> str`

输出结构：

```text
入口文件总览
训练入口
评估入口
demo 入口
可视化入口
配置入口
测试入口
建议优先阅读顺序
```

分类规则：

训练入口：

```text
train.py
train_*.py
scripts/train*.py
```

评估入口：

```text
eval.py
evaluate.py
test.py
scripts/eval*.py
scripts/test*.py
```

demo 入口：

```text
demo.py
demo_*.py
examples/*.py
scripts/demo*.py
```

可视化入口：

```text
visualize.py
render.py
plot.py
viewer.py
```

配置入口：

```text
configs/*.yaml
configs/*.json
config/*.yaml
```

测试入口：

```text
tests/
test_*.py
```

建议优先阅读顺序：

```text
1. README
2. requirements / environment
3. configs
4. demo / examples
5. train / eval
6. core algorithm modules
7. tests
```

---

#### 15.2.3 `build_reproduce_plan(path: Path) -> str`

输出结构：

```text
项目基本判断
环境安装
数据准备
最小 demo
训练复现
评估复现
指标对比
视频输出
常见失败点
建议执行顺序
```

必须强调：

```text
不要直接运行未知安装脚本。
先阅读 README 和依赖文件。
优先找最小 demo，而不是直接训练。
如果需要 GPU / 大数据集 / 第三方资产，需要单独标记。
```

---

#### 15.2.4 `build_port_to_b_report(path: Path) -> str`

输出结构：

```text
可迁移到 B 的模块
不建议直接迁移的模块
需要适配的接口
建议新增到 Robot_Dynamics_Control 的位置
下一步 Codex 任务
风险与边界
```

可迁移模块候选包括：

```text
cost design
trajectory generator
MPC loop structure
iLQR solver structure
MPPI / CEM sampler
visualization script
config organization
logging format
benchmark script
```

不建议直接迁移：

```text
真实硬件接口
闭源依赖
绑定特定机器人资产的代码
需要大数据集的训练逻辑
复杂未理解的 monolithic framework
```

建议迁移位置示例：

```text
projects/B_mujoco_mpc_study/simulator/controllers/
projects/B_mujoco_mpc_study/simulator/planners/
projects/B_mujoco_mpc_study/simulator/utils/
projects/B_mujoco_mpc_study/configs/
projects/B_mujoco_mpc_study/tests/
docs/00_project_management/
```

---

#### 15.2.5 `generate_codex_reproduce_prompt(path: Path) -> str`

生成一个可复制给 Codex 的外部 repo 审读提示词。

必须包含：

```text
任务目标
外部 repo 路径
只读边界
禁止事项
审读内容
入口文件识别
论文/算法模块识别
复现路线
可迁移到 B 项目的模块
输出报告要求
最终汇报格式
```

提示词必须要求 Codex：

```text
- 只读分析外部项目
- 不修改第三方源码
- 不执行 git add/commit/push
- 不运行长时间训练
- 不直接运行未知安装脚本
- 生成审读报告
- 列出入口文件
- 列出复现命令候选
- 列出依赖文件
- 列出和 B_mujoco_mpc_study 的可迁移点
```

输出报告建议路径：

```text
tools/robot_atlas/reviews/<repo_name>_review.md
```

注意：本函数只生成提示词，不实际创建该报告。

---

## 16. `cli.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/cli.py
```

### 16.1 模块职责

实现命令行入口：

```bash
robot-atlas
```

建议使用标准库：

```python
argparse
```

不要强制依赖 typer/click。

### 16.2 必须支持命令

#### 16.2.1 `scan`

```bash
robot-atlas scan
```

执行：

```python
scan_repo()
print(scan_result.as_text())
```

---

#### 16.2.2 `status`

```bash
robot-atlas status B
```

执行：

```python
scan_result = scan_repo()
print(build_status(project_id, scan_result))
```

---

#### 16.2.3 `next`

```bash
robot-atlas next B03
```

执行：

```python
print(build_next_step(stage_id))
```

---

#### 16.2.4 `explain`

```bash
robot-atlas explain iLQR
```

执行：

```python
print(explain_algorithm(name))
```

---

#### 16.2.5 `generate-codex`

```bash
robot-atlas generate-codex B03-R4C-1B
```

执行：

```python
print(generate_codex_prompt(task_id))
```

---

#### 16.2.6 `validate`

```bash
robot-atlas validate B03
```

执行：

```python
print(build_validation_plan(stage_id))
```

---

#### 16.2.7 `demo-assets`

```bash
robot-atlas demo-assets B03
```

执行：

```python
print(build_demo_assets_report(stage_id, repo_root))
```

---

#### 16.2.8 `guards`

```bash
robot-atlas guards
```

执行：

```python
print(run_guard_checks(repo_root))
```

---

#### 16.2.9 `read-repo`

```bash
robot-atlas read-repo external/open_source_repos/some_repo
```

执行：

```python
report = read_external_repo(path)
print(report.as_text())
```

---

#### 16.2.10 `entrypoints`

```bash
robot-atlas entrypoints external/open_source_repos/some_repo
```

执行：

```python
print(build_entrypoints_report(path))
```

---

#### 16.2.11 `reproduce-plan`

```bash
robot-atlas reproduce-plan external/open_source_repos/some_repo
```

执行：

```python
print(build_reproduce_plan(path))
```

---

#### 16.2.12 `port-to-B`

```bash
robot-atlas port-to-B external/open_source_repos/some_repo
```

执行：

```python
print(build_port_to_b_report(path))
```

---

#### 16.2.13 `generate-codex-reproduce`

```bash
robot-atlas generate-codex-reproduce external/open_source_repos/some_repo
```

执行：

```python
print(generate_codex_reproduce_prompt(path))
```

---

### 16.3 CLI 错误处理要求

1. 未知命令：显示 help。
2. 缺参数：显示 help。
3. 路径不存在：不要 traceback，输出清晰错误。
4. 扫描失败：输出 warning，不中断整个程序。
5. 所有命令都应返回人类可读文本。

---

## 17. `README.md` 详细要求

文件路径：

```text
tools/robot_atlas/README.md
```

必须包含以下章节：

```markdown
# RoboControl Atlas

## 1. 工具定位
## 2. 两种模式
### 2.1 Owned Project Mode
### 2.2 External Reproduction Mode
## 3. 当前支持范围
## 4. 当前不支持内容
## 5. 安装方式
## 6. 命令示例
## 7. 安全边界
## 8. 和 Robot_Dynamics_Control 的关系
## 9. 推荐工作流
## 10. 后续路线
```

### 17.1 工具定位

写清楚：

```text
RoboControl Atlas 是一个面向机器人运动控制学习和求职项目的本地项目理解工具。
它既能管理自己的 Robot_Dynamics_Control 项目进度，也能快速审读外部论文开源仓库，生成复现路线和迁移计划。
```

### 17.2 安装方式

写：

```bash
cd /home/ubuntu/Robot_Dynamics_Control/tools/robot_atlas
pip install -e .
```

如果没有安装，也可运行：

```bash
PYTHONPATH=src python -m robot_atlas.cli scan
```

### 17.3 命令示例

内部项目：

```bash
robot-atlas scan
robot-atlas status B
robot-atlas next B03
robot-atlas explain iLQR
robot-atlas generate-codex B03-R4C-1B
robot-atlas validate B03
robot-atlas demo-assets B03
robot-atlas guards
```

外部项目：

```bash
robot-atlas read-repo external/open_source_repos/some_repo
robot-atlas entrypoints external/open_source_repos/some_repo
robot-atlas reproduce-plan external/open_source_repos/some_repo
robot-atlas port-to-B external/open_source_repos/some_repo
robot-atlas generate-codex-reproduce external/open_source_repos/some_repo
```

### 17.4 当前不支持内容

必须写：

```text
- 不接入 LLM API
- 不自动修改主项目源码
- 不自动运行仿真
- 不自动安装外部 repo
- 不自动下载数据
- 不做网页 Dashboard
- 不做论文 PDF 自动解析
```

---

## 18. 测试要求

测试目录：

```text
tools/robot_atlas/tests/
```

使用 pytest。

### 18.1 `test_scanner.py`

至少测试：

1. `scan_repo` 返回 `ScanResult`。
2. 能识别 B 项目。
3. `ProjectLine` 中有 `todo_count` 字段。
4. 不存在的目录不会报错。
5. 能识别 `test_*.py`。
6. 能识别入口脚本。

---

### 18.2 `test_status.py`

至少测试：

1. `build_status("B", scan_result)` 输出包含：

```text
B03-R4C
B03-R4C-1A
B03-R4C-1B
state tracking smoke run
```

2. A/C/D 输出不会报错。

---

### 18.3 `test_next_step.py`

至少测试：

```text
build_next_step("B03")
```

输出包含：

```text
B03-R4C-1B
state tracking smoke run
two-link dynamics
tracking error
cost
torque
```

---

### 18.4 `test_explain.py`

至少测试：

```text
explain_algorithm("iLQR")
```

输出包含：

```text
rollout
Riccati
tracking error
state trajectory
control trajectory
```

测试 `MPC` 输出包含：

```text
receding horizon
warm-start
```

测试未知算法不会报错。

---

### 18.5 `test_codex_prompt.py`

至少测试：

```text
generate_codex_prompt("B03-R4C-1B")
```

输出包含：

```text
不要 git add
不要 git commit
不要 git push
tracking_error.png
cost_history.png
torque_profile.png
stepB03R4C1B_state_tracking_smoke_run.md
```

---

### 18.6 `test_validate.py`

至少测试：

```text
build_validation_plan("B03")
```

输出包含：

```text
pytest projects/B_mujoco_mpc_study/tests/test_B03_ilqr_two_link_smoke_skeleton.py -q
run_B03_ilqr_lite_two_link_smoke.py
tracking_error.png
cost_history.png
torque_profile.png
```

---

### 18.7 `test_demo_assets.py`

至少测试：

1. 对临时空 repo 调用不会报错。
2. 缺文件时输出“当前缺失”。
3. 输出包含：

```text
MP4
tracking error
cost
torque
求职展示可用性
```

---

### 18.8 `test_guards.py`

至少测试：

```text
run_guard_checks(tmp_path)
```

输出包含：

```text
external/open_source_repos
shared/robot_assets
git add
git commit
git push
默认只读
```

---

### 18.9 `test_external_repo.py`

创建临时 fake repo：

```text
fake_repo/
├── README.md
├── demo.py
├── requirements.txt
├── configs/default.yaml
└── tests/test_demo.py
```

测试：

1. `read_external_repo(fake_repo)` 能识别 README。
2. 能识别 demo.py。
3. 能识别 requirements.txt。
4. 能识别 configs/default.yaml。
5. `build_entrypoints_report` 输出包含 demo。
6. `build_reproduce_plan` 输出包含环境安装、最小 demo。
7. `build_port_to_b_report` 输出包含 `B_mujoco_mpc_study`。
8. `generate_codex_reproduce_prompt` 输出包含只读、不修改第三方源码。

---

## 19. 运行验证要求

完成代码后，请运行：

```bash
cd /home/ubuntu/Robot_Dynamics_Control
pytest tools/robot_atlas/tests -q
```

如支持安装，请运行：

```bash
cd /home/ubuntu/Robot_Dynamics_Control/tools/robot_atlas
pip install -e .
```

然后回到仓库根目录运行：

```bash
cd /home/ubuntu/Robot_Dynamics_Control

robot-atlas scan
robot-atlas status B
robot-atlas next B03
robot-atlas explain iLQR
robot-atlas generate-codex B03-R4C-1B
robot-atlas validate B03
robot-atlas demo-assets B03
robot-atlas guards
```

外部 repo 模式可以用临时 fake repo 测试，不要下载网络项目。

---

## 20. 最终汇报格式

完成后必须按以下格式汇报：

```text
1. 新增文件
2. 未修改的关键区域
3. 已实现命令
4. 各模块功能说明
5. 测试命令和结果
6. 当前限制
7. 下一步建议
8. 是否执行 git add/commit/push
```

其中第 8 点必须明确写：

```text
没有执行 git add。
没有执行 git commit。
没有执行 git push。
```

如果某些测试失败，要如实汇报失败原因，不要假装通过。

---

## 21. 验收标准

本任务完成后，至少满足：

1. `tools/robot_atlas/` 目录结构完整。
2. `pyproject.toml` 可安装本地包。
3. `robot-atlas scan` 可运行。
4. `robot-atlas status B` 输出 B03-R4C 当前状态。
5. `robot-atlas next B03` 输出 B03-R4C-1B 下一步。
6. `robot-atlas explain iLQR` 输出学习笔记式解释。
7. `robot-atlas generate-codex B03-R4C-1B` 输出完整 Codex 提示词。
8. `robot-atlas validate B03` 输出验证计划。
9. `robot-atlas demo-assets B03` 输出 demo 产物缺失情况。
10. `robot-atlas guards` 输出安全边界。
11. `robot-atlas read-repo <path>` 可审读 fake 外部 repo。
12. `robot-atlas reproduce-plan <path>` 可生成复现路线。
13. `robot-atlas port-to-B <path>` 可生成迁移到 B 项目的建议。
14. `pytest tools/robot_atlas/tests -q` 尽量通过。
15. 不修改 A/B/C/D 主项目源码。
16. 不修改 external/open_source_repos。
17. 不修改 shared/robot_assets。
18. 不执行 git add/commit/push。

---

## 22. 后续路线，仅写入 README，不在本次实现

在 README 中写清楚后续路线：

### v0.1 内部项目管理

```text
- scan
- status
- next
- explain
- generate-codex
- validate
- demo-assets
- guards
```

### v0.2 外部项目快速审读

```text
- read-repo
- entrypoints
- reproduce-plan
- port-to-B
- generate-codex-reproduce
```

### v0.3 论文到代码映射

未来再做，不在本次实现：

```text
- paper-map
- algorithm-map
- equation-map
- experiment-map
- paper-to-code table
```

注意：v0.3 不要在本次实现，只在 README 中作为路线说明。

---

## 23. 再次强调

请严格遵守：

```text
不要 git add
不要 git commit
不要 git push
不要修改 external/open_source_repos
不要修改 shared/robot_assets
不要重构 projects/A-D
不要删除 TODO 教学注释
不要运行长时间仿真
不要下载网络资源
不要接入外部 LLM API
```

本次只实现：

```text
tools/robot_atlas/
```

并完成轻量测试。

---

# Part 2：RoboControl Atlas × Understand Anything 可选增强模块提示词

# RoboControl Atlas × Understand Anything 可选增强模块 Codex 提示词

> 用途：把本文件交给 Codex，让 Codex 在已有 `RoboControl Atlas` 工具基础上，增加一个 **Understand Anything / Lum1104/Understand-Anything 可选增强接入模块**。
> 核心思想：不要把 Understand Anything 作为强依赖；只在外部 repo 已经存在 `.understand-anything/knowledge-graph.json` 时，读取它并增强外部项目审读、入口识别、复现路线和迁移到 B 项目的建议。
> 严格边界：不要自动安装 Understand Anything，不要自动运行远程安装脚本，不要强制调用 `/understand`，不要修改第三方源码，不要执行 git add/commit/push。

---

## 0. 当前背景

当前仓库：

```bash
/home/ubuntu/Robot_Dynamics_Control
```

当前已有或计划实现的工具：

```text
tools/robot_atlas/
```

工具名称：

```text
RoboControl Atlas
```

命令行入口：

```bash
robot-atlas
```

RoboControl Atlas 的已有定位：

```text
面向机器人运动控制学习和求职项目的本地项目理解工具。
它既能管理自己的 Robot_Dynamics_Control 项目进度，也能快速审读外部论文开源仓库，生成复现路线和迁移计划。
```

已有两个模式：

```text
Mode A：Owned Project Mode
用于管理 /home/ubuntu/Robot_Dynamics_Control 中的 A/B/C/D 项目线。

Mode B：External Reproduction Mode
用于快速审读外部论文开源项目，生成入口分析、复现路线、可迁移模块分析和 Codex 复现提示词。
```

本次新增模式：

```text
Mode C：Understand Anything Optional Enhancement Mode
读取外部 repo 中已有的 Understand Anything 图谱结果，增强 External Reproduction Mode。
```

---

## 1. 为什么要接入 Understand Anything

用户在复现新的论文和开源项目时，需要快速理解陌生代码库。

普通静态扫描只能识别：

```text
README
main.py
run.py
demo.py
train.py
eval.py
configs
requirements
tests
```

但复杂机器人控制项目还需要理解：

```text
代码依赖关系
核心算法模块
控制器入口
planner 入口
environment / simulator 接口
cost function
trajectory generator
配置如何连接实验
demo 如何调用算法
哪些文件适合迁移到自己的 B_mujoco_mpc_study
```

Understand Anything 可以把代码库分析成知识图谱，因此适合作为 RoboControl Atlas 的可选增强后端。

但是，RoboControl Atlas 不能依赖它才能运行。

正确关系是：

```text
RoboControl Atlas 负责用户自己的学习 / 复现工作流。
Understand Anything 负责陌生代码库图谱理解。
Codex 负责根据提示词执行具体实现或审读任务。
```

---

## 2. 接入原则

### 2.1 不做强依赖

即使没有 Understand Anything，以下命令仍必须能正常运行：

```bash
robot-atlas scan
robot-atlas status B
robot-atlas next B03
robot-atlas explain iLQR
robot-atlas generate-codex B03-R4C-1B
robot-atlas validate B03
robot-atlas demo-assets B03
robot-atlas read-repo <repo>
robot-atlas entrypoints <repo>
robot-atlas reproduce-plan <repo>
robot-atlas port-to-B <repo>
```

Understand Anything 只能作为增强功能：

```bash
robot-atlas import-understand <repo>
robot-atlas understand-summary <repo>
robot-atlas understand-entrypoints <repo>
robot-atlas understand-port-to-B <repo>
robot-atlas generate-codex-understand <repo>
```

### 2.2 不自动运行 `/understand`

本模块不负责执行：

```text
/understand
/understand-dashboard
/understand-chat
/understand-diff
/understand-explain
```

原因：

1. `/understand` 通常是 AI coding platform 内部命令，不一定能被普通 Python CLI 稳定调用。
2. 不同平台环境不同，如 Claude Code、Codex、Cursor、Gemini CLI 等。
3. 自动调用外部插件可能有安全风险。
4. 本工具应保持稳定、可控、离线可用。

### 2.3 只读取已有图谱

本模块只寻找并读取：

```text
<repo_path>/.understand-anything/knowledge-graph.json
```

如果不存在，就输出提示：

```text
未发现 Understand Anything 图谱。
请先在目标 repo 中使用 Understand Anything 生成图谱，例如运行 /understand。
本命令不会自动安装或自动运行 Understand Anything。
```

### 2.4 外部 repo 默认只读

不得修改：

```text
external/open_source_repos/
任何外部 repo 源码
.understand-anything/knowledge-graph.json
```

不得把分析结果写回第三方 repo。

如果需要生成报告，只能写到 RoboControl Atlas 自己的报告目录，例如：

```text
tools/robot_atlas/reviews/
```

但本次优先只输出到 stdout，不强制落盘。

---

## 3. 本次新增文件

请在已有 `tools/robot_atlas/` 中新增：

```text
tools/robot_atlas/src/robot_atlas/understand_models.py
tools/robot_atlas/src/robot_atlas/understand_adapter.py
tools/robot_atlas/tests/test_understand_adapter.py
```

并修改：

```text
tools/robot_atlas/src/robot_atlas/cli.py
tools/robot_atlas/README.md
```

只允许修改这些工具内部文件。

不要修改：

```text
projects/A_self_baseline/
projects/B_mujoco_mpc_study/
projects/C_openloong_dyn_control_study/
projects/D_legged_control_study/
external/open_source_repos/
shared/robot_assets/
```

---

## 4. 新增 CLI 命令

必须新增以下命令：

```bash
robot-atlas import-understand <repo_path>
robot-atlas understand-summary <repo_path>
robot-atlas understand-entrypoints <repo_path>
robot-atlas understand-port-to-B <repo_path>
robot-atlas generate-codex-understand <repo_path>
```

### 4.1 `robot-atlas import-understand <repo_path>`

用途：

```text
检查目标 repo 是否存在 Understand Anything 图谱文件，并输出基础摘要。
```

行为：

1. 检查 `<repo_path>` 是否存在。
2. 检查 `<repo_path>/.understand-anything/knowledge-graph.json` 是否存在。
3. 如果不存在，输出清晰提示，不抛异常。
4. 如果存在，读取 JSON。
5. 输出：
   - repo 路径
   - graph 路径
   - graph 是否可读
   - 顶层字段
   - 节点数量
   - 边数量
   - 文件节点候选
   - 函数节点候选
   - 类节点候选
   - 警告信息

示例输出结构：

```text
Understand Anything 图谱导入检查

Repo:
...

Graph:
...

状态:
已发现 knowledge-graph.json

图谱摘要:
- 顶层字段: ...
- 节点数量: ...
- 边数量: ...
- 文件节点候选数量: ...
- 函数节点候选数量: ...
- 类节点候选数量: ...

提示:
本命令只读取图谱，不修改外部 repo。
```

---

### 4.2 `robot-atlas understand-summary <repo_path>`

用途：

```text
基于 Understand Anything 图谱生成外部 repo 的结构摘要。
```

输出结构：

```text
项目图谱摘要
核心文件候选
核心类 / 函数候选
依赖热点
建议优先阅读顺序
与静态扫描结果的互补关系
风险与限制
```

要求：

1. 如果图谱不存在，提示用户先运行 `/understand`。
2. 如果图谱存在但字段结构未知，也要尽量从 JSON 中启发式提取。
3. 不因字段不符合预期而崩溃。
4. 对无法识别的字段输出 warning。

---

### 4.3 `robot-atlas understand-entrypoints <repo_path>`

用途：

```text
结合 Understand Anything 图谱和文件命名规则，辅助判断外部 repo 的运行入口。
```

输出结构：

```text
入口判断结果
README / 文档入口
demo 入口
train 入口
eval 入口
main / run 入口
高连接度入口候选
建议最小复现入口
建议阅读顺序
```

入口识别策略：

1. 文件名规则：
   - `main.py`
   - `run.py`
   - `run_*.py`
   - `demo.py`
   - `demo_*.py`
   - `train.py`
   - `train_*.py`
   - `eval.py`
   - `eval_*.py`
   - `examples/*.py`
   - `scripts/*.py`
2. 图谱节点规则：
   - 节点名称中包含 main / run / demo / train / eval / launch / script
   - 节点连接度较高
   - 被多个模块引用
3. 如果图谱无法提供边信息，则退回文件名规则。

---

### 4.4 `robot-atlas understand-port-to-B <repo_path>`

用途：

```text
基于 Understand Anything 图谱，判断外部 repo 中哪些模块适合迁移到 B_mujoco_mpc_study。
```

输出结构：

```text
可迁移模块总览
controller 候选
planner 候选
cost function 候选
trajectory generator 候选
environment / simulator 候选
visualization 候选
config system 候选
logging / benchmark 候选
建议迁移目标位置
不建议直接迁移的内容
需要适配的接口
风险与边界
下一步 Codex 任务
```

目标项目线：

```text
projects/B_mujoco_mpc_study
```

建议目标目录：

```text
projects/B_mujoco_mpc_study/simulator/controllers/
projects/B_mujoco_mpc_study/simulator/planners/
projects/B_mujoco_mpc_study/simulator/envs/
projects/B_mujoco_mpc_study/simulator/utils/
projects/B_mujoco_mpc_study/simulator/scripts/
projects/B_mujoco_mpc_study/configs/
projects/B_mujoco_mpc_study/tests/
docs/00_project_management/
```

可迁移模块关键词：

```text
controller
control
mpc
ilqr
ilqg
mppi
cem
planner
planning
trajectory
rollout
cost
loss
reward
dynamics
env
environment
mujoco
pinocchio
visualize
render
plot
logger
benchmark
config
```

不建议直接迁移：

```text
真实硬件接口
闭源依赖
绑定特定机器人资产的代码
需要大数据集的训练逻辑
复杂未理解的 monolithic framework
平台专用 launch 脚本
需要 GPU 训练才能工作的模块
需要 Isaac / ROS / proprietary simulator 的模块
```

---

### 4.5 `robot-atlas generate-codex-understand <repo_path>`

用途：

```text
生成一个让 Codex 基于 Understand Anything 图谱审读外部 repo 的详细提示词。
```

输出的 Codex 提示词必须包含：

```text
任务目标
外部 repo 路径
Understand Anything 图谱路径
只读边界
禁止事项
审读内容
入口文件识别
核心算法模块识别
复现路线
迁移到 B 项目分析
输出报告要求
最终汇报格式
```

提示词必须要求 Codex：

```text
- 只读分析外部项目
- 优先参考 .understand-anything/knowledge-graph.json
- 不修改第三方源码
- 不执行 git add/commit/push
- 不运行长时间训练
- 不直接运行未知安装脚本
- 生成 repo overview
- 生成 entrypoint map
- 生成 algorithm map
- 生成 reproduce plan
- 生成 port-to-B plan
```

建议报告输出路径：

```text
tools/robot_atlas/reviews/<repo_name>_understand_review.md
```

但本函数只生成提示词，不实际创建报告。

---

## 5. `understand_models.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/understand_models.py
```

### 5.1 必须定义 dataclass：`UnderstandGraphSummary`

```python
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

@dataclass
class UnderstandGraphSummary:
    repo_path: Path
    graph_path: Path | None
    graph_exists: bool
    graph_loaded: bool
    top_level_keys: list[str] = field(default_factory=list)
    node_count: int = 0
    edge_count: int = 0
    file_nodes: list[str] = field(default_factory=list)
    function_nodes: list[str] = field(default_factory=list)
    class_nodes: list[str] = field(default_factory=list)
    entrypoint_candidates: list[str] = field(default_factory=list)
    algorithm_candidates: list[str] = field(default_factory=list)
    dependency_hotspots: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
```

### 5.2 推荐方法

```python
def as_text(self) -> str:
    ...
```

输出内容：

```text
UnderstandGraphSummary
- repo_path
- graph_path
- graph_exists
- graph_loaded
- top_level_keys
- node_count
- edge_count
- file_nodes
- function_nodes
- class_nodes
- entrypoint_candidates
- algorithm_candidates
- dependency_hotspots
- warnings
```

### 5.3 设计注意

字段必须允许为空。
图谱格式可能变化，因此不能假设固定 JSON schema。

---

## 6. `understand_adapter.py` 详细要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/understand_adapter.py
```

### 6.1 模块职责

读取 `.understand-anything/knowledge-graph.json`，尽力提取：

```text
节点
边
文件节点
函数节点
类节点
入口候选
算法候选
依赖热点
迁移到 B 项目的候选模块
```

注意：

```text
不要写入外部 repo。
不要修改图谱文件。
不要运行 Understand Anything。
```

---

### 6.2 必须实现函数

#### 6.2.1 `find_understand_graph(repo_path: Path) -> Path | None`

查找：

```text
<repo_path>/.understand-anything/knowledge-graph.json
```

如果存在返回路径，否则返回 None。

---

#### 6.2.2 `load_understand_graph(graph_path: Path) -> dict[str, Any]`

读取 JSON。

要求：

1. 使用 `encoding="utf-8"`。
2. JSON decode error 时返回空 dict 或抛出受控异常。
3. 不让 CLI 产生长 traceback。
4. 如果文件过大，也要尽量安全读取。
5. 不修改文件。

---

#### 6.2.3 `extract_graph_lists(graph: dict[str, Any]) -> tuple[list[Any], list[Any]]`

目标：

```text
尽量从未知 JSON schema 中提取 nodes 和 edges。
```

启发式支持以下常见字段：

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

如果找不到，返回空列表。

---

#### 6.2.4 `node_to_text(node: Any) -> str`

把图谱节点转成可搜索文本。

支持：

1. node 是 str。
2. node 是 dict。
3. node 是其它对象。

如果 node 是 dict，优先拼接字段：

```text
id
name
label
type
kind
path
file
symbol
summary
description
```

---

#### 6.2.5 `edge_to_text(edge: Any) -> str`

把边转成可搜索文本。

支持：

```text
source
target
from
to
type
kind
label
relationship
```

---

#### 6.2.6 `classify_node_text(text: str) -> dict[str, bool]`

根据文本判断节点类型。

返回：

```python
{
    "is_file": bool,
    "is_function": bool,
    "is_class": bool,
    "is_entrypoint": bool,
    "is_algorithm": bool,
    "is_robot_related": bool,
}
```

文件节点关键词：

```text
.py
.cpp
.hpp
.h
.ts
.js
.yaml
.yml
.json
.toml
.md
```

函数节点关键词：

```text
function
def
method
callable
```

类节点关键词：

```text
class
```

入口关键词：

```text
main
run
demo
train
eval
evaluate
launch
script
example
```

算法关键词：

```text
controller
control
mpc
ilqr
ilqg
mppi
cem
planner
planning
trajectory
rollout
cost
loss
reward
dynamics
env
environment
mujoco
pinocchio
qp
wbc
ik
fk
jacobian
```

机器人相关关键词：

```text
robot
mujoco
pinocchio
urdf
mjcf
controller
planner
dynamics
trajectory
torque
joint
task-space
end-effector
```

---

#### 6.2.7 `summarize_understand_graph(repo_path: Path) -> UnderstandGraphSummary`

主函数。

流程：

1. 查找图谱。
2. 不存在则返回 `graph_exists=False` 的 summary，warnings 中加入提示。
3. 存在则读取 JSON。
4. 提取 top-level keys。
5. 尝试提取 nodes / edges。
6. 统计 node_count / edge_count。
7. 分类 file_nodes / function_nodes / class_nodes。
8. 提取 entrypoint_candidates。
9. 提取 algorithm_candidates。
10. 提取 dependency_hotspots。
11. 返回 summary。

注意：

- 列表长度可以限制，例如最多显示 30 个，避免输出过长。
- 但 node_count / edge_count 要显示总数。

---

#### 6.2.8 `build_import_understand_report(repo_path: Path) -> str`

对应 CLI：

```bash
robot-atlas import-understand <repo_path>
```

输出结构：

```text
Understand Anything 图谱导入检查
Repo
Graph
状态
图谱摘要
候选节点
警告
安全边界
```

---

#### 6.2.9 `build_understand_summary_report(repo_path: Path) -> str`

对应 CLI：

```bash
robot-atlas understand-summary <repo_path>
```

输出结构：

```text
项目图谱摘要
核心文件候选
核心类 / 函数候选
依赖热点
建议优先阅读顺序
与静态扫描结果的互补关系
风险与限制
```

---

#### 6.2.10 `build_understand_entrypoints_report(repo_path: Path) -> str`

对应 CLI：

```bash
robot-atlas understand-entrypoints <repo_path>
```

输出结构：

```text
入口判断结果
README / 文档入口
demo 入口
train 入口
eval 入口
main / run 入口
高连接度入口候选
建议最小复现入口
建议阅读顺序
```

如果可行，可以复用 `external_repo.py` 中的静态 entrypoint 检测函数；不要复制大量重复逻辑。

---

#### 6.2.11 `build_understand_port_to_b_report(repo_path: Path) -> str`

对应 CLI：

```bash
robot-atlas understand-port-to-B <repo_path>
```

输出结构：

```text
可迁移模块总览
controller 候选
planner 候选
cost function 候选
trajectory generator 候选
environment / simulator 候选
visualization 候选
config system 候选
logging / benchmark 候选
建议迁移目标位置
不建议直接迁移的内容
需要适配的接口
风险与边界
下一步 Codex 任务
```

映射规则：

```text
controller/control → projects/B_mujoco_mpc_study/simulator/controllers/
planner/mpc/ilqr/ilqg/mppi/cem → projects/B_mujoco_mpc_study/simulator/planners/
env/environment/mujoco → projects/B_mujoco_mpc_study/simulator/envs/
trajectory/rollout/utils → projects/B_mujoco_mpc_study/simulator/utils/
demo/run/script → projects/B_mujoco_mpc_study/simulator/scripts/
config/yaml/json → projects/B_mujoco_mpc_study/configs/
test → projects/B_mujoco_mpc_study/tests/
doc/readme → docs/00_project_management/
```

---

#### 6.2.12 `generate_codex_understand_prompt(repo_path: Path) -> str`

对应 CLI：

```bash
robot-atlas generate-codex-understand <repo_path>
```

输出完整 Codex 提示词。

提示词章节：

```text
任务背景
外部 repo 路径
Understand Anything 图谱路径
只读边界
禁止事项
审读目标
入口文件识别
核心算法模块识别
复现路线
迁移到 B 项目分析
输出报告要求
最终汇报格式
```

必须包含禁止事项：

```text
不要修改外部 repo 源码
不要执行 git add
不要执行 git commit
不要执行 git push
不要运行长时间训练
不要直接运行未知安装脚本
不要下载大数据集
不要把外部 repo 代码直接复制进 B 项目
不要声称已经完成复现，除非实际运行并验证
```

最终汇报格式：

```text
1. 是否发现 Understand Anything 图谱
2. 图谱路径
3. 项目主入口候选
4. 核心算法模块候选
5. 最小复现路线
6. 迁移到 B_mujoco_mpc_study 的候选模块
7. 不建议迁移的内容
8. 风险与缺失信息
9. 是否修改外部源码，必须说明没有
10. 是否执行 git add/commit/push，必须说明没有
```

---

## 7. `cli.py` 修改要求

文件路径：

```text
tools/robot_atlas/src/robot_atlas/cli.py
```

在已有 CLI 中新增子命令：

```bash
robot-atlas import-understand <repo_path>
robot-atlas understand-summary <repo_path>
robot-atlas understand-entrypoints <repo_path>
robot-atlas understand-port-to-B <repo_path>
robot-atlas generate-codex-understand <repo_path>
```

### 7.1 CLI 行为要求

如果路径不存在，输出：

```text
路径不存在：<repo_path>
```

不要 traceback。

如果图谱不存在，输出：

```text
未发现 Understand Anything 图谱：
<repo_path>/.understand-anything/knowledge-graph.json

请先在目标 repo 中运行 Understand Anything，例如 /understand。
本工具不会自动安装或自动运行 Understand Anything。
```

如果图谱存在但解析失败，输出：

```text
发现图谱，但解析失败。
可能是 JSON 格式变化或文件损坏。
```

不要长 traceback。

---

## 8. `README.md` 修改要求

文件路径：

```text
tools/robot_atlas/README.md
```

新增章节：

```markdown
## Understand Anything 可选增强

### 作用

RoboControl Atlas 可以读取外部 repo 中已有的 Understand Anything 图谱：

```text
<repo_path>/.understand-anything/knowledge-graph.json
```

用于增强：

- 外部 repo 审读
- 入口识别
- 核心算法模块候选
- 迁移到 B_mujoco_mpc_study 的建议
- Codex 审读提示词生成

### 边界

- 不自动安装 Understand Anything
- 不自动运行 /understand
- 不修改外部 repo
- 不把 Understand Anything 作为强依赖
- 图谱不存在时，普通 external repo 审读功能仍然可用

### 命令

```bash
robot-atlas import-understand <repo_path>
robot-atlas understand-summary <repo_path>
robot-atlas understand-entrypoints <repo_path>
robot-atlas understand-port-to-B <repo_path>
robot-atlas generate-codex-understand <repo_path>
```

### 推荐工作流

```bash
cd external/open_source_repos/some_repo

# 在支持 Understand Anything 的 AI coding 环境中手动运行：
/understand

# 回到 Robot_Dynamics_Control：
cd /home/ubuntu/Robot_Dynamics_Control

robot-atlas import-understand external/open_source_repos/some_repo
robot-atlas understand-summary external/open_source_repos/some_repo
robot-atlas understand-entrypoints external/open_source_repos/some_repo
robot-atlas understand-port-to-B external/open_source_repos/some_repo
robot-atlas generate-codex-understand external/open_source_repos/some_repo
```

```

---

## 9. 测试要求

新增测试文件：

```text
tools/robot_atlas/tests/test_understand_adapter.py
```

### 9.1 测试 1：图谱不存在时不报错

创建临时 repo：

```text
fake_repo/
├── README.md
└── demo.py
```

调用：

```python
summary = summarize_understand_graph(fake_repo)
```

断言：

```python
summary.graph_exists is False
summary.graph_loaded is False
summary.warnings
```

调用：

```python
build_import_understand_report(fake_repo)
```

断言输出包含：

```text
未发现
knowledge-graph.json
不会自动安装
```

---

### 9.2 测试 2：能读取简单 knowledge-graph.json

创建：

```text
fake_repo/
└── .understand-anything/
    └── knowledge-graph.json
```

内容：

```json
{
  "nodes": [
    {"id": "file:demo.py", "type": "file", "path": "demo.py"},
    {"id": "func:main", "type": "function", "name": "main"},
    {"id": "class:MPCController", "type": "class", "name": "MPCController"},
    {"id": "file:controllers/mpc.py", "type": "file", "path": "controllers/mpc.py"}
  ],
  "edges": [
    {"source": "file:demo.py", "target": "func:main", "type": "defines"},
    {"source": "func:main", "target": "class:MPCController", "type": "uses"}
  ]
}
```

断言：

```python
summary.graph_exists is True
summary.graph_loaded is True
summary.node_count == 4
summary.edge_count == 2
```

输出包含：

```text
demo.py
MPCController
controllers/mpc.py
```

---

### 9.3 测试 3：entrypoints 报告

调用：

```python
report = build_understand_entrypoints_report(fake_repo)
```

断言输出包含：

```text
demo
main
建议最小复现入口
```

---

### 9.4 测试 4：port-to-B 报告

调用：

```python
report = build_understand_port_to_b_report(fake_repo)
```

断言输出包含：

```text
B_mujoco_mpc_study
controllers
planners
需要适配
风险
```

---

### 9.5 测试 5：Codex 提示词

调用：

```python
prompt = generate_codex_understand_prompt(fake_repo)
```

断言输出包含：

```text
只读
不要修改外部 repo 源码
不要执行 git add
不要执行 git commit
不要执行 git push
Understand Anything
knowledge-graph.json
port-to-B
```

---

### 9.6 测试 6：兼容未知 JSON schema

创建图谱：

```json
{
  "knowledge_graph": {
    "nodes": [
      {"label": "scripts/run_demo.py", "kind": "file"},
      {"label": "planner_ilqr", "kind": "function"}
    ],
    "edges": []
  }
}
```

断言：

```python
summary.node_count == 2
summary.graph_loaded is True
```

不允许崩溃。

---

## 10. 更新验收标准

完成后请运行：

```bash
cd /home/ubuntu/Robot_Dynamics_Control
pytest tools/robot_atlas/tests/test_understand_adapter.py -q
```

如果全量测试可行：

```bash
pytest tools/robot_atlas/tests -q
```

安装后测试 CLI：

```bash
cd /home/ubuntu/Robot_Dynamics_Control/tools/robot_atlas
pip install -e .

cd /home/ubuntu/Robot_Dynamics_Control
robot-atlas import-understand external/open_source_repos/some_repo
robot-atlas understand-summary external/open_source_repos/some_repo
robot-atlas understand-entrypoints external/open_source_repos/some_repo
robot-atlas understand-port-to-B external/open_source_repos/some_repo
robot-atlas generate-codex-understand external/open_source_repos/some_repo
```

如果没有真实 external repo，可用测试中的 fake repo 验证。

---

## 11. 最终汇报格式

完成后请按以下格式汇报：

```text
1. 新增文件
2. 修改文件
3. 新增命令
4. Understand Anything 接入方式
5. 每个命令的功能说明
6. 测试命令和结果
7. 当前限制
8. 后续建议
9. 是否修改 external/open_source_repos，必须明确说明没有
10. 是否执行 git add/commit/push，必须明确说明没有
```

第 9 点必须写：

```text
没有修改 external/open_source_repos。
没有修改 shared/robot_assets。
没有修改 projects/A-D 主项目源码。
```

第 10 点必须写：

```text
没有执行 git add。
没有执行 git commit。
没有执行 git push。
```

---

## 12. 当前限制，需要写入 README

本模块不是 Understand Anything 的替代品。

当前限制：

```text
1. 不自动安装 Understand Anything。
2. 不自动运行 /understand。
3. 不保证 knowledge-graph.json 的 schema 永远稳定。
4. 只做启发式解析。
5. 不能保证自动识别的入口一定正确。
6. 不能保证自动迁移建议一定可直接执行。
7. 不直接修改外部 repo。
8. 不直接复制外部源码到 B 项目。
9. 不自动验证复现结果。
10. 不自动读取论文 PDF。
```

---

## 13. 推荐工作流，需要写入 README

### 13.1 外部项目快速复现工作流

```bash
cd /home/ubuntu/Robot_Dynamics_Control/external/open_source_repos/some_repo

# 在支持 Understand Anything 的 AI coding 环境中手动运行
/understand

cd /home/ubuntu/Robot_Dynamics_Control

robot-atlas read-repo external/open_source_repos/some_repo
robot-atlas import-understand external/open_source_repos/some_repo
robot-atlas understand-summary external/open_source_repos/some_repo
robot-atlas understand-entrypoints external/open_source_repos/some_repo
robot-atlas reproduce-plan external/open_source_repos/some_repo
robot-atlas understand-port-to-B external/open_source_repos/some_repo
robot-atlas generate-codex-understand external/open_source_repos/some_repo
```

### 13.2 判断是否值得复现

优先看：

```text
README 是否清楚
依赖是否可安装
是否有 demo
是否有 configs
是否有 eval
是否有测试
是否有预训练模型
是否需要 GPU
是否需要大数据集
是否需要专用仿真器
Understand 图谱是否能识别核心算法
是否能迁移到 B_mujoco_mpc_study
```

### 13.3 判断是否适合迁移到 B

优先迁移：

```text
cost design
trajectory generator
MPC loop structure
iLQR solver structure
MPPI / CEM sampler
visualization script
config organization
logging format
benchmark script
```

谨慎迁移：

```text
真实硬件接口
ROS launch
Isaac-only module
大规模训练框架
闭源依赖封装
特定机器人资产绑定代码
```

---

## 14. 不要做的事情

本次不要实现：

```text
自动安装 Understand Anything
自动执行 /understand
自动调用 AI coding platform
自动生成图谱
自动改外部 repo
自动复制外部代码进 B 项目
自动运行外部 repo demo
自动跑外部 repo 训练
自动解析论文 PDF
自动构建网页 Dashboard
```

只实现：

```text
读取已有 knowledge-graph.json
生成摘要
生成入口判断
生成 port-to-B 分析
生成 Codex 审读提示词
新增 CLI 命令
新增测试
更新 README
```

---

## 15. 再次强调安全边界

请严格遵守：

```text
不要 git add
不要 git commit
不要 git push
不要修改 external/open_source_repos
不要修改 shared/robot_assets
不要重构 projects/A-D
不要运行长时间仿真
不要下载网络资源
不要自动安装 Understand Anything
不要自动执行 /understand
不要接入外部 LLM API
```

本次只允许修改：

```text
tools/robot_atlas/
```

---

## 16. 简短版本总结

本次任务一句话：

```text
为 RoboControl Atlas 增加 Understand Anything 可选增强模块：只读取外部 repo 已有的 .understand-anything/knowledge-graph.json，用于增强外部项目审读、入口识别、复现路线和迁移到 B_mujoco_mpc_study 的建议；不自动安装、不自动运行、不修改第三方源码。
```

---

# 合并后最终验收清单

完成后至少应满足：

```text
1. tools/robot_atlas/ 目录结构完整。
2. pyproject.toml 可安装本地包。
3. robot-atlas scan 可运行。
4. robot-atlas status B 输出 B03-R4C 当前状态。
5. robot-atlas next B03 输出 B03-R4C-1B 下一步。
6. robot-atlas explain iLQR 输出学习笔记式解释。
7. robot-atlas generate-codex B03-R4C-1B 输出完整 Codex 提示词。
8. robot-atlas validate B03 输出验证计划。
9. robot-atlas demo-assets B03 输出 demo 产物缺失情况。
10. robot-atlas guards 输出安全边界。
11. robot-atlas read-repo <path> 可审读 fake 外部 repo。
12. robot-atlas reproduce-plan <path> 可生成复现路线。
13. robot-atlas port-to-B <path> 可生成迁移到 B 项目的建议。
14. robot-atlas import-understand <path> 可检查 .understand-anything/knowledge-graph.json。
15. robot-atlas understand-summary <path> 可基于已有图谱生成摘要。
16. robot-atlas understand-entrypoints <path> 可基于图谱辅助判断入口。
17. robot-atlas understand-port-to-B <path> 可基于图谱生成迁移建议。
18. robot-atlas generate-codex-understand <path> 可生成基于 Understand Anything 图谱的 Codex 审读提示词。
19. pytest tools/robot_atlas/tests -q 尽量通过。
20. 不修改 A/B/C/D 主项目源码。
21. 不修改 external/open_source_repos。
22. 不修改 shared/robot_assets。
23. 不执行 git add / git commit / git push。
```

---

# 合并后推荐 Codex 执行命令

```bash
cd /home/ubuntu/Robot_Dynamics_Control

codex --auto-edit < /path/to/robo_control_atlas_full_codex_prompt.md
```

如果当前 Codex 不支持 stdin：

```bash
cd /home/ubuntu/Robot_Dynamics_Control

codex exec - < /path/to/robo_control_atlas_full_codex_prompt.md
```

---

# 合并后推荐验证命令

```bash
cd /home/ubuntu/Robot_Dynamics_Control

pytest tools/robot_atlas/tests -q
```

如果支持安装：

```bash
cd /home/ubuntu/Robot_Dynamics_Control/tools/robot_atlas
pip install -e .

cd /home/ubuntu/Robot_Dynamics_Control

robot-atlas scan
robot-atlas status B
robot-atlas next B03
robot-atlas explain iLQR
robot-atlas generate-codex B03-R4C-1B
robot-atlas validate B03
robot-atlas demo-assets B03
robot-atlas guards
```

Understand Anything 可选增强验证：

```bash
robot-atlas import-understand external/open_source_repos/some_repo
robot-atlas understand-summary external/open_source_repos/some_repo
robot-atlas understand-entrypoints external/open_source_repos/some_repo
robot-atlas understand-port-to-B external/open_source_repos/some_repo
robot-atlas generate-codex-understand external/open_source_repos/some_repo
```

---

# 最终汇报格式

Codex 完成后必须按以下格式汇报：

```text
1. 新增文件
2. 修改文件
3. 未修改的关键区域
4. 已实现命令
5. 各模块功能说明
6. Understand Anything 接入方式
7. 测试命令和结果
8. 当前限制
9. 下一步建议
10. 是否修改 external/open_source_repos，必须明确说明没有
11. 是否修改 shared/robot_assets，必须明确说明没有
12. 是否修改 projects/A-D 主项目源码，必须明确说明没有
13. 是否执行 git add/commit/push，必须明确说明没有
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
