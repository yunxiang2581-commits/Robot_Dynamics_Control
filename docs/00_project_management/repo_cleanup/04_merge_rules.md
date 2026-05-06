# 仓库整理合并规则

## 外部源码规则

- 不直接合并外部开源仓库源码。
- 不把 `external/open_source_repos/` 提交到主仓库。
- 外部仓库仅作为只读源码参考。
- Project B/C/D 的文档可以引用外部仓库路径，但不复制大段源码。

## A 项目保护规则

- 不删除 `projects/A_self_baseline/`。
- 不移动 `projects/A_self_baseline/`。
- 不重命名 A 项目脚本。
- 不在仓库整理任务中修改 A 项目 Python/C++ 控制代码。

## Markdown 合并规则

- 重复 Markdown 先合并成规范主文档。
- 合并时保留有价值的历史信息。
- 合并完成后，旧文档先标记为 archive candidate。
- 不直接删除旧 Markdown。

## 旧 step 文档规则

- `docs/00_project_management/step*.md` 先归档，不直接删除。
- 若某个 step 文档仍解释当前目录结构或历史决策，应保留或迁移摘要。

## 输出成果规则

- 输出视频 demo 和 metrics 属于正式成果，不作为缓存删除。
- `outputs/videos/`、metrics JSON / CSV、图表和报告需要按项目阶段人工确认。
- 不把正式 demo 结果与缓存文件混为一类。

## 低风险缓存规则

以下属于低风险缓存，可在单独步骤清理：

- `__pycache__/`
- `*.pyc`
- `.pytest_cache/`
- `.mypy_cache/`
- `.ruff_cache/`

清理缓存前仍应先输出候选列表，并确认不会删除源码、文档、配置、测试或正式成果。
