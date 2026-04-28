# Final Merge Report

## 1. New Project Working Directory

The only active working directory for the migrated monorepo is:

```text
/home/ubuntu/Robot_Dynamics_Control
```

## 2. Old Project Source

The old project source used for audit and lightweight A asset import was:

```text
/home/ubuntu/robot_proj/Pinocchio_URDF
```

No file was migrated, moved, or deleted in Step 7.

## 3. Completed Stages

- Step 0.5: Git cleanup checkpoint and ignore rules.
- Step 1: old/new project audit.
- Step 2: A/B/C monorepo skeleton.
- Step 3: lightweight A legacy asset import.
- Step 4: existing Markdown docs reorganized into A/B/C.
- Step 5: A standard TODO learning script and module skeletons.
- Step 6: root A assets relocated into `projects/A_self_baseline/`.
- Step 7: independence checks and final report.

## 4. A Project Assets

`projects/A_self_baseline/` now contains:

- Standard TODO learning scripts under `scripts/`.
- Standard module skeletons under `src/robot_baseline/`.
- Config templates under `configs/`.
- Legacy old-project references under `legacy_imported/`.
- Early root-project assets under `root_imported/`, `root_imported_src/`, and `root_imported_utils/`.
- A docs, legacy mapping, and learning script plan.
- Root-imported envs, controllers, experiments, notes, plots, and tests.
- Small legacy output samples under `outputs/legacy_samples/`.

## 5. B Project Assets

`projects/B_legged_control_study/` now contains:

- Project README.
- Project docs.
- `external/legged_control_README.md`.
- Empty containers for configs, scripts, reproduce logs, and outputs.

## 6. C Project Assets

`projects/C_unitree_rl_mjlab_study/` now contains:

- Project README.
- Project docs.
- `external/unitree_rl_mjlab_README.md`.
- Empty containers for configs, scripts, reproduce logs, and outputs.

## 7. shared/ Assets

`shared/` now contains:

- `shared/env/`: imported Pinocchio_URDF environment files and Docker templates.
- `shared/robot_assets/`: placeholder model asset container.
- `shared/scripts/`: shared script container.
- `shared/templates/`: shared template container.

## 8. tools/ Assets

`tools/` now contains:

- `tools/export/`: Markdown to Word export scripts and README.
- `tools/migration/`: migration helper container.

## 9. Root Directory Retained Content

The root directory currently retains:

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

`scripts/` and `external/` are empty directories in the current scan.

## 10. Old Path Dependency Check

Search target:

```text
/home/ubuntu/robot_proj/Pinocchio_URDF
```

Findings:

- Allowed occurrences exist in migration/project-management documents.
- Additional occurrences exist in imported legacy output sample text files under `projects/A_self_baseline/outputs/legacy_samples/`.
- No disallowed occurrence was found in standard A scripts, standard A configs, standard A modules, root README run commands, or B/C project run commands.

The legacy output sample paths are historical text records, not runtime configuration. They should be treated as reference artifacts.

## 11. Large File Risk Check

Scanned suffixes:

```text
*.mp4 *.avi *.mov *.bag *.db3 *.pt *.pth *.onnx *.ckpt *.npy *.csv
```

Found local CSV files under:

```text
outputs/exp_pd_joint_control/*/data/pd_joint_log.csv
```

These files are ignored by `.gitignore` through:

```text
outputs/exp_pd_joint_control/
```

`git ls-files` did not report tracked files with the scanned large-output suffixes.

## 12. Root A Directory Residue Check

The following root A directories were checked and were not present:

```text
controllers/
dynamics/
envs/
experiments/
notes/
plots/
src/
tests/
utils/
configs/
```

No root A project directory residue was found.

## 13. Follow-Up Suggestions

- Step 8: clean empty root directories such as `scripts/` and `external/` if they are no longer needed, and finalize root README wording.
- Step 8: decide whether `requirements.txt`, `outputs/`, and ignored local files should remain at root.
- Step 9: start implementing A project `projects/A_self_baseline/scripts/01_inspect_urdf.py`.
- Before implementation, confirm robot model source under `shared/robot_assets/models/`.

## 14. Acceptance Checklist

- [x] No files were migrated, moved, or deleted in Step 7.
- [x] Old path hardcoding was scanned.
- [x] Standard A scripts/configs/modules do not contain the old absolute path.
- [x] Root A project directories were checked.
- [x] Root `scripts/` and `external/` were checked.
- [x] Large-output suffixes were scanned outside `.git/`.
- [x] A/B/C project boundaries were checked.
- [x] `shared/env`, `shared/robot_assets`, and `tools/export` were checked.
- [x] Final merge report was created.
- [x] Root README was updated at documentation level only.
