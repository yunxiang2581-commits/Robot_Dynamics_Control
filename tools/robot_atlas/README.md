# RoboControl Atlas

RoboControl Atlas 是 `Robot_Dynamics_Control` 仓库的本地静态项目理解工具。它只做文件扫描、文本报告、提示词生成、验证计划生成和外部 repo 只读审读，不接入外部 LLM API，不运行长时间仿真。

## 安装

```bash
cd tools/robot_atlas
pip install -e .
```

## 命令

```bash
robot-atlas scan
robot-atlas status B
robot-atlas next B03
robot-atlas explain iLQR
robot-atlas generate-codex B03-R4C-1B
robot-atlas validate B03
robot-atlas demo-assets B03
robot-atlas guards
robot-atlas read-repo external/open_source_repos/some_repo
robot-atlas entrypoints external/open_source_repos/some_repo
robot-atlas reproduce-plan external/open_source_repos/some_repo
robot-atlas port-to-B external/open_source_repos/some_repo
robot-atlas generate-codex-reproduce external/open_source_repos/some_repo
robot-atlas route "B03 下一步做什么？"
```

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

cd /home/ubuntu/Robot_Dynamics_Control

robot-atlas read-repo external/open_source_repos/some_repo
robot-atlas import-understand external/open_source_repos/some_repo
robot-atlas understand-summary external/open_source_repos/some_repo
robot-atlas understand-entrypoints external/open_source_repos/some_repo
robot-atlas reproduce-plan external/open_source_repos/some_repo
robot-atlas understand-port-to-B external/open_source_repos/some_repo
robot-atlas generate-codex-understand external/open_source_repos/some_repo
```

## Auto Trigger / Command Router

`route` 用于把自然语言需求转换成推荐的 `robot-atlas` 命令。它只推荐命令，不自动执行扫描、安装、仿真、下载或外部 repo 脚本。

### 示例

```bash
robot-atlas route "B03 下一步做什么？"
robot-atlas route "生成 B03-R4C-1B 的 Codex 提示词"
robot-atlas route "iLQR 原理是什么？"
robot-atlas route "这个 repo 怎么复现？"
robot-atlas route "用 Understand Anything 图谱分析这个项目"
robot-atlas route "检查 B03 demo 产物"
robot-atlas route "检查禁止事项"
```

输出会包含：

```text
识别到的需求类型
推荐命令
理由
注意事项
```

### 路由规则

- 项目状态 / 进度：推荐 `robot-atlas scan` 或 `robot-atlas status B`
- B03 下一步：推荐 `robot-atlas next B03`
- 明确任务 ID 的 Codex 提示词：推荐 `robot-atlas generate-codex <task_id>`
- 算法解释：推荐 `robot-atlas explain <topic>`
- 验证计划：推荐 `robot-atlas validate B03`
- demo 产物检查：推荐 `robot-atlas demo-assets B03`
- 安全边界 / 禁止事项：推荐 `robot-atlas guards`
- 外部 repo 审读：推荐 `read-repo` / `entrypoints` / `reproduce-plan`
- Understand Anything 图谱：推荐 `import-understand` / `understand-summary`

如果需求涉及外部 repo 但没有提供本地路径，`route` 会提示需要提供 `<repo_path>`。RoboControl Atlas 不会自动 clone GitHub repo。

## 当前限制

1. 不自动安装 Understand Anything。
2. 不自动运行 /understand。
3. 不保证 `knowledge-graph.json` 的 schema 永远稳定。
4. 只做启发式解析。
5. 不能保证自动识别的入口一定正确。
6. 不能保证自动迁移建议一定可直接执行。
7. 不直接修改外部 repo。
8. 不直接复制外部源码到 B 项目。
9. 不自动验证复现结果。
10. 不自动读取论文 PDF。

## 安全边界

- 不执行 `git add` / `git commit` / `git push`
- 不修改 `external/open_source_repos`
- 不修改 `shared/robot_assets`
- 不重构 `projects/A-D`
- 不运行长时间仿真
- 不下载网络资源
