---
name: pinocchio-learning
description: Use this skill when working on the A self baseline robotics learning project, especially for Pinocchio URDF loading, path handling, frame and joint search, FK, Jacobian, IK, MuJoCo playback, PD control learning scripts, TODO skeletons, and learning-oriented code reviews in Robot_Dynamics_Control.
---

# Pinocchio Learning

## Overview

This skill is migrated from the old `Pinocchio_URDF` repository rules. Use it to keep A project work focused on the learning path:

```text
URDF -> FK -> Jacobian -> IK -> dynamics -> control
```

The goal is not to generate a large finished control stack in one pass. The goal is to build readable, verifiable learning scripts and reusable modules step by step.

For the original migrated rules, read `references/AGENTS_from_Pinocchio_URDF.md` when a task needs the full policy text. `references/AGENT_from_Pinocchio_URDF.md` is the shorter old-agent variant.

## Repository Context

Current monorepo root:

```text
/home/ubuntu/Robot_Dynamics_Control
```

A project root:

```text
projects/A_self_baseline/
```

Standard A project code should live in:

- `projects/A_self_baseline/scripts/` for learning script entry points.
- `projects/A_self_baseline/src/robot_baseline/` for reusable Python modules.
- `projects/A_self_baseline/configs/` for YAML/TOML/JSON templates.
- `projects/A_self_baseline/docs/` for A project notes.
- `shared/robot_assets/` for shared robot model resources.

Treat `projects/A_self_baseline/scripts/legacy_imported/` as read-only reference material unless the user explicitly asks to edit legacy files.

## Before Editing

Run `git status --short` before changing project files.

Before writing or changing Pinocchio, IK, Jacobian, MuJoCo, or controller code, briefly state:

- What this step computes.
- Why it exists in the learning path.
- What the input state or model is.
- What the output object or file is.
- Whether the math logic changes.

Keep changes scoped to the requested learning step. Do not jump ahead to QP, WBC, dynamics, or RL unless the user explicitly asks.

## Coding Rules

Prefer simple Python with a clear `main()` function for scripts.

Use:

- `pathlib.Path` for paths.
- `argparse` for command-line options.
- `logging` for script progress.
- Chinese comments for teaching notes and TODOs.
- Small text, CSV, or plot outputs under task-specific output directories.

Avoid:

- Depending on the shell working directory.
- Hard-coding `/home/ubuntu/robot_proj/Pinocchio_URDF`.
- Rewriting whole files when a small patch is enough.
- Deleting teaching comments.
- Making learning code compact at the cost of readability.

For standard scripts in `projects/A_self_baseline/scripts/`, derive paths like:

```python
from pathlib import Path

A_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = A_ROOT.parents[1]
SHARED_MODELS = REPO_ROOT / "shared" / "robot_assets" / "models"
OUT_DIR = A_ROOT / "outputs" / "<task_name>"
```

For legacy scripts under `scripts/legacy_imported/`, remember that the old `parents[1]` pattern points to `projects/A_self_baseline/scripts/` in the new layout. Do not copy that path logic into standard code without adapting it.

## Learning Workflow

Use this order for A project implementation:

1. Inspect URDF and Pinocchio model dimensions.
2. Compute frame poses with FK.
3. Compute frame Jacobian and validate it with finite differences.
4. Implement DLS position IK.
5. Add QP-IK only after DLS IK is verified.
6. Add MuJoCo PD tracking after Pinocchio kinematics are stable.
7. Add Mini-WBC QP only after QP-IK and PD tracking are understood.

Each step should be independently runnable and should print or save enough information for verification.

## Model and Target Search

Do not rely on one guessed frame or joint name. Prefer this order:

1. Exact match.
2. Lowercase substring match.
3. Clear error listing useful candidates.

Useful APIs:

- `model.names`
- `model.frames`
- `[frame.name for frame in model.frames]`
- `model.getFrameId(name)`
- `model.getJointId(name)`

For humanoid learning, default attention is pelvis, legs, feet, and floating-base behavior. Arms are secondary unless the task says otherwise.

## Pinocchio Concepts To Explain

When relevant, explain:

- Fixed base vs floating base.
- `nq` vs `nv`.
- `pin.JointModelFreeFlyer()`.
- `pin.neutral(model)`.
- `forwardKinematics`.
- `updateFramePlacements`.
- `computeJointJacobians`.
- `getFrameJacobian`.
- `pin.integrate`.

For IK and Jacobian tasks, also state risks such as singularity, frame mismatch, floating-base indexing, quaternion integration, step size, damping, and non-convergence.

## TODO Style

For learning skeletons, TODO comments should say:

- 要实现什么。
- 为什么这一步对机器人运动控制求职重要。
- 推荐使用哪些 API。
- 输入是什么。
- 输出是什么。
- 如何验证。

If the user asks to fill TODOs:

- Fill only the requested TODOs.
- Explain each filled TODO.
- Preserve file structure and teaching comments.
- Do not silently expand into later learning tasks.

## Verification

After project edits, run at least:

```bash
git diff --stat
```

For Python edits, prefer:

```bash
python -m py_compile <changed_python_file>
```

For runnable learning scripts, verify with a small command and report key outputs: selected URDF, frame or joint, `nq`, `nv`, matrix shape, norm error, convergence status, or output file path.

Do not run `git push`. Do not change Docker socket permissions. If Docker is blocked by sandboxing, ask for approval rather than changing system permissions.
