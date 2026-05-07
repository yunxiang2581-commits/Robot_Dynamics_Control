# Step 6 Relocate Root A Assets

## Step 6 Goal

Move early A self baseline assets out of the repository root and into `projects/A_self_baseline/`, while keeping root-level files focused on project management, shared resources, tools, and top-level documentation.

This step does not migrate anything from `/home/ubuntu/robot_proj/Pinocchio_URDF`, does not modify `scripts/legacy_imported/`, and does not implement robot control algorithms.

## Root Content That Belongs To A

The following root-level content is treated as early A self baseline material:

- `controllers/`
- `dynamics/`
- `envs/`
- `experiments/`
- `notes/`
- `plots/`
- `utils/`
- `configs/`
- `tests/`
- `init_project.py`
- `main.py`
- `test_mujoco.py`
- `src/`

These should move under `projects/A_self_baseline/` using `root_imported/` containers so they do not overwrite the standard Step 5 skeleton.

## Root Content To Keep

The following content stays at repository root:

- `README.md`
- `.gitignore`
- `docs/`
- `projects/`
- `shared/`
- `tools/`
- `exports/`
- `external/` when it still contains overall external notes
- `outputs/` as a temporary top-level output entry
- `requirements.txt` until a later environment decision

## Move Rules

- Use `git mv` for tracked files and directories whenever possible.
- Do not overwrite existing target files or directories.
- Move root A directories into `projects/A_self_baseline/<area>/root_imported/`.
- Move root `src/` into `projects/A_self_baseline/src/root_imported_src/` to avoid overwriting standard modules.
- Move export helper scripts from root `scripts/` into `tools/export/`.
- Move B/C external notes into the matching project `external/` directory.

## Conflict Rules

If a destination already exists, do not overwrite it. Record the source and target in:

```text
docs/00_project_management/step6_relocation_conflicts.md
```

If an external file cannot be confidently assigned to B or C, record it in:

```text
docs/00_project_management/step6_external_mapping_todo.md
```

## Acceptance Checklist

- [ ] Root A code, experiments, configs, tests, notes, plots, and early entry files are under `projects/A_self_baseline/`.
- [ ] Root export helper scripts are under `tools/export/`.
- [ ] B and C external notes are under their project containers.
- [ ] No legacy imported script is modified.
- [ ] No file is overwritten.
- [ ] Conflicts and unknown external files are recorded.
- [ ] Root directory is closer to a monorepo shell.
