# Codex Workflow Rules

> Chinese version: [08_codex_workflow_rules.md](08_codex_workflow_rules.md)

## Reading Navigation

| Previous | Current | Next |
| --- | --- | --- |
| [07 Robot model and asset plan](07_robot_model_and_asset_plan_en.md) | Codex workflow rules | [09 Risk and fallback plan](09_risk_and_fallback_plan_en.md) |

## Purpose

This document constrains how Codex should modify the repository. The core principle is to build teaching skeletons first, then implement algorithms step by step with explanation and verification.

## Key Takeaways

| Rule | Meaning |
| --- | --- |
| Current phase | only docs, tables, directories, and templates |
| Algorithm phase | TODO teaching skeleton first, incremental implementation later |
| Comment requirement | core TODOs should explain inputs, outputs, recommended APIs, and validation in Chinese |
| External projects | do not download large repositories and do not heavily modify third-party source |

## Repository-Local Skill

The old `Pinocchio_URDF` repo did not contain a standard `SKILL.md` package, but it did contain `AGENTS.md` and `AGENT.MD`. The current repo migrated them into:

- `tools/codex_skills/pinocchio-learning/SKILL.md`
- `tools/codex_skills/pinocchio-learning/references/AGENTS_from_Pinocchio_URDF.md`
- `tools/codex_skills/pinocchio-learning/references/AGENT_from_Pinocchio_URDF.md`

## Core Rules

- Do not implement large complex algorithms in one pass.
- Keep core algorithms as TODO teaching skeletons first.
- Preserve Chinese teaching comments for interview review value.
- Do not download large third-party repositories into the repo.
- Do not commit large models, videos, logs, or weights.
- Every phase needs explicit acceptance criteria.

## TODO Skeleton Format

```python
# TODO: Implement this robot-control step.
# What to implement:
# Why it matters for job interviews:
# Recommended APIs:
# Inputs:
# Outputs:
# Validation:
```

## Development Order

- Preparation phase: docs, tables, structure, templates only
- A skeleton phase: interfaces, configs, script entry points, test skeletons
- A implementation phase: implement one module at a time with verification
- B breakdown phase: read docs and write architecture and source-code notes
- C breakdown phase: analyze observation/action/reward and runtime records
