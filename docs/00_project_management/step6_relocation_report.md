# Step 6 Relocation Report

## 1. Directories Actually Moved

Root A project directories moved into `projects/A_self_baseline/`:

```text
configs/ -> projects/A_self_baseline/configs/root_imported/
controllers/ -> projects/A_self_baseline/controllers/root_imported/
dynamics/ -> projects/A_self_baseline/dynamics/root_imported/
envs/ -> projects/A_self_baseline/envs/root_imported/
experiments/ -> projects/A_self_baseline/experiments/root_imported/
notes/ -> projects/A_self_baseline/notes/root_imported/
plots/ -> projects/A_self_baseline/plots/root_imported/
utils/ -> projects/A_self_baseline/src/root_imported_utils/
tests/ -> projects/A_self_baseline/tests/root_imported/
src/ -> projects/A_self_baseline/src/root_imported_src/
```

Tracked files were moved with `git mv`. Empty or untracked directories were moved without changing source content.

## 2. Files Actually Moved

Root Python entry files moved into A experiments:

```text
init_project.py -> projects/A_self_baseline/experiments/root_imported/init_project.py
main.py -> projects/A_self_baseline/experiments/root_imported/main.py
test_mujoco.py -> projects/A_self_baseline/experiments/root_imported/test_mujoco.py
```

Root export helper files moved into `tools/export/`:

```text
scripts/README.md -> tools/export/README.md
scripts/export_md_to_docx.ps1 -> tools/export/export_md_to_docx.ps1
scripts/export_md_to_docx.sh -> tools/export/export_md_to_docx.sh
```

Root external study notes moved into B/C project containers:

```text
external/legged_control_README.md -> projects/B_legged_control_study/external/legged_control_README.md
external/unitree_rl_mjlab_README.md -> projects/C_unitree_rl_mjlab_study/external/unitree_rl_mjlab_README.md
```

## 3. Files Kept At Repository Root

Root content intentionally kept:

```text
README.md
.gitignore
docs/
projects/
shared/
tools/
exports/
external/
outputs/
requirements.txt
```

Local or ignored root entries still present on disk:

```text
.codex/
.idea/
.vscode/
debug.log
scripts/
```

`external/` and `scripts/` currently have no remaining files from this step's scan.

## 4. Conflicts

No relocation conflicts were recorded.

Conflict tracking file:

```text
docs/00_project_management/step6_relocation_conflicts.md
```

## 5. Pending Manual Confirmation

No unknown root `external/` files remained after moving the B/C notes, so `step6_external_mapping_todo.md` was not created.

Pending follow-up decisions:

- Decide whether root `requirements.txt` should stay global or move into `shared/env/`.
- Decide whether root `outputs/` should remain a global output index or be split by project.
- Decide whether ignored local folders `.idea/`, `.vscode/`, `.codex/`, and `debug.log` should remain local-only.
- Review `root_imported/`, `root_imported_src/`, and `root_imported_utils/` before refactoring into standard A modules.

## 6. Current Root Directory Remaining Structure

```text
.
.codex
.git
.gitignore
.idea
.vscode
README.md
debug.log
docs
exports
external
outputs
projects
requirements.txt
scripts
shared
tools
```

## 7. Next Step Suggestions

- Update path rules and `.gitignore` if project-local outputs need more precise ignore behavior.
- Review `projects/A_self_baseline/*/root_imported/` content and decide what becomes standard A code.
- Keep `legacy_imported/` unchanged as old project reference material.
- Add project-level run instructions only after the TODO skeletons begin to receive real implementations.
