# 仓库整理执行 TODO

## Step 1：提交 .gitignore 和 repo_cleanup 文档

目标：

- 固化外部仓库隔离规则。
- 提交本目录下的整理方案文档。

允许修改范围：

- `.gitignore`
- `docs/00_project_management/repo_cleanup/`

禁止修改范围：

- `projects/A_self_baseline/`
- `projects/B_mujoco_mpc_study/`
- `projects/C_openloong_dyn_control_study/`
- `projects/D_legged_control_study/`
- `external/open_source_repos/`
- Python/C++ 源码。

验证命令：

```bash
git status --short
git diff -- .gitignore docs/00_project_management/repo_cleanup
```

回滚方式：

```bash
git restore .gitignore
```

对于新增文档，如尚未提交，可人工确认后删除新增文档目录；不要影响其他目录。

## Step 2：规范 B/C/D README 和 simulator 目录

目标：

- 确保 B/C/D README 都清楚说明 simulation-only 目标。
- 确保 simulator 目录只包含规划和 TODO skeleton，不实现复杂逻辑。

允许修改范围：

- `projects/B_mujoco_mpc_study/`
- `projects/C_openloong_dyn_control_study/`
- `projects/D_legged_control_study/`

禁止修改范围：

- `projects/A_self_baseline/`
- `external/open_source_repos/`
- 编译脚本或外部源码。

验证命令：

```bash
git status --short
git diff -- projects/B_mujoco_mpc_study projects/C_openloong_dyn_control_study projects/D_legged_control_study
```

回滚方式：

```bash
git restore <被修改的 tracked 文件>
```

新增文件需人工确认后处理。

## Step 3：合并重复 Markdown 为主文档

目标：

- 将重复或相似 Markdown 的有效内容合并到规范主文档。
- 避免多个文档表达同一规则导致维护困难。

允许修改范围：

- 目标主文档。
- 被合并文档的归档标记说明。

禁止修改范围：

- Python/C++ 源码。
- 外部仓库。
- A 项目核心脚本。

验证命令：

```bash
git diff --stat
git diff -- docs projects
```

回滚方式：

```bash
git restore <被修改的 tracked markdown>
```

## Step 4：只清理低风险缓存

目标：

- 清理 `.pytest_cache/`、`__pycache__/`、`*.pyc` 等可再生成缓存。

允许修改范围：

- 缓存目录。
- 字节码文件。
- 明确临时日志。

禁止修改范围：

- `.md`
- `.py`
- `.cpp/.h`
- configs
- tests
- outputs/videos
- metrics
- external 源码仓库。

验证命令：

```bash
git status --short
find . -name __pycache__ -o -name "*.pyc"
```

回滚方式：

- 若误删 tracked 文件，使用 `git restore <path>`。
- 若误删 untracked 缓存，通常无需恢复；但执行前仍需人工确认候选列表。

## Step 5：人工确认 archive candidates

目标：

- 判断旧目录和旧文档是否归档。

允许修改范围：

- archive 目标目录。
- 被归档的历史文档或历史目录。

禁止修改范围：

- A 项目核心代码。
- 外部源码仓库内部文件。
- 正式成果输出。

验证命令：

```bash
git status --short
git diff --stat
```

回滚方式：

- 若使用 git 管理移动，使用 git 反向操作恢复。
- 未提交前可根据 `git status` 和人工记录恢复。

## Step 6：最后才执行归档或删除

目标：

- 在合并、确认、备份和审阅完成后，才执行归档或删除。

允许修改范围：

- 已确认的 archive candidates。
- 已确认的低风险 deletion candidates。

禁止修改范围：

- 未确认内容。
- 外部源码仓库。
- A 项目核心代码。
- B/C/D 主项目成果。

验证命令：

```bash
git status --short
git diff --stat
```

回滚方式：

- 删除 tracked 文件前必须确保可用 `git restore` 恢复。
- 删除 untracked 文件前必须人工确认其确实无价值或已有备份。
