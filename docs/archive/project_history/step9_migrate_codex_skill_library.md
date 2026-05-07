# Step 9 - Migrate Codex Skill Library

## Step 9 目标

将旧仓库 `/home/ubuntu/robot_proj/Pinocchio_URDF` 中与 Codex 协作相关的规则完整迁入当前仓库，并转换为当前 monorepo 可提交、可复用的项目内 skill 技能库。

## 旧仓库检查结果

旧仓库中没有发现标准 `SKILL.md` 技能包，也没有 `.codex/skills/` 或 `.agents/skills/` 目录。

实际存在的规则文件是：

- `/home/ubuntu/robot_proj/Pinocchio_URDF/AGENTS.md`
- `/home/ubuntu/robot_proj/Pinocchio_URDF/AGENT.MD`
- `/home/ubuntu/robot_proj/Pinocchio_URDF/.codex`

其中 `.codex` 是 0 字节本地状态文件，不是技能目录。

## 本次迁移内容

新增项目内技能库：

```text
tools/codex_skills/pinocchio-learning/
├── SKILL.md
├── agents/openai.yaml
└── references/
    ├── AGENTS_from_Pinocchio_URDF.md
    └── AGENT_from_Pinocchio_URDF.md
```

`SKILL.md` 保存当前仓库可直接使用的精简规则，包括：

- A 项目当前 monorepo 路径；
- Pinocchio 学习主线；
- legacy_imported 只读规则；
- 路径处理规则；
- frame/joint 搜索规则；
- TODO 教学注释规则；
- 验证规则。

`references/` 保存旧仓库原始规则文本，便于需要完整上下文时查阅。

## 为什么不迁移到 `.codex/`

当前仓库的 `.gitignore` 明确忽略 `.codex/`，并且当前根目录存在一个 0 字节 `.codex` 本地文件。

为了让 skill 技能库能进入 Git 并随项目管理，本次将技能库放在：

```text
tools/codex_skills/
```

这样不会覆盖本地 `.codex` 状态，也不会受到 `.gitignore` 对 `.codex/` 的影响。

## 与 AGENTS.md 的关系

根目录 `AGENTS.md` 保留为面向 Codex 的项目级协作规则。

项目内 skill 技能库是更结构化的版本：

- `AGENTS.md`：仓库级常驻规则；
- `tools/codex_skills/pinocchio-learning/SKILL.md`：可迁移、可安装、可复用的技能包；
- `tools/codex_skills/pinocchio-learning/references/`：旧仓库规则原文。

## 验收清单

- [x] 检查旧仓库是否存在标准 skill 包。
- [x] 确认旧仓库没有 `SKILL.md`。
- [x] 将旧仓库 `AGENTS.md` 迁入 skill references。
- [x] 将旧仓库 `AGENT.MD` 迁入 skill references。
- [x] 创建 `tools/codex_skills/pinocchio-learning/SKILL.md`。
- [x] 创建 `agents/openai.yaml`。
- [x] 不修改 legacy_imported 脚本。
- [x] 不删除旧仓库或当前仓库本地文件。
