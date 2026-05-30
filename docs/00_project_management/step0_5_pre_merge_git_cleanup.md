# Step 0.5 Pre-Merge Git Cleanup

## 1. Current Task Goal

This checkpoint prepares the repository for the later migration steps without moving project assets or changing robot control code. The goal is to make the Git working tree easier to review by separating source files, documentation, and configuration from local IDE metadata, Python caches, and generated experiment outputs.

This step does not create the future `projects/A`, `projects/B`, or `projects/C` structure.

## 2. Current Git Status Issues

The working tree contains a mix of source changes, documentation additions, local tool metadata, Python bytecode, and generated run artifacts. The main issues are:

- IDE settings under `.idea/` are tracked or modified.
- Python cache files such as `__pycache__/` and `*.pyc` are tracked.
- Experiment run outputs under `outputs/exp_pd_joint_control/` include CSV data, metadata, plots, and generated reports.
- Runtime output folders such as `outputs/logs/`, `outputs/videos/`, and `outputs/runs/` should stay local.
- New project preparation folders such as `docs/`, `configs/`, `scripts/`, `src/`, `external/`, and `exports/` need to remain visible for review.

## 3. Files That Should Be Kept

The following categories should remain available for Git review and future commits:

- Repository documentation: `README.md`, `docs/`, `notes/*.md`, `outputs/README.md`
- Dependency and setup files: `requirements.txt`, `init_project.py`
- Source code: `main.py`, `controllers/*.py`, `experiments/*.py`, `src/`
- Robot and simulation assets: `envs/*.xml`, `configs/`
- Tests and test documentation: `tests/*.py`, `tests/README.md`
- Helper scripts: `scripts/`
- External reference notes and export structure: `external/`, `exports/`
- Small reviewable examples in `outputs/figures/` or `outputs/reports/` when they are intentionally curated

## 4. Files That Should Be Removed From Git Tracking

If already tracked, these paths should be removed from the Git index with `git rm --cached` while keeping the local files on disk:

- `.idea/`
- `.vscode/`
- `controllers/__pycache__/`
- `outputs/exp_pd_joint_control/`

These are not deleted locally in this checkpoint.

## 5. Local Generated Artifacts

The following paths are local generated artifacts and should not be committed as routine source material:

- Python bytecode and caches: `__pycache__/`, `*.pyc`, `*.pyo`, `*.pyd`
- IDE metadata: `.idea/`, `.vscode/`
- Runtime logs and videos: `outputs/logs/`, `outputs/videos/`, `outputs/runs/`
- Experiment run data and metadata: `outputs/exp_pd_joint_control/*/data/`, `outputs/exp_pd_joint_control/*/meta/`, generated CSV files, generated log files
- Model checkpoints and large binary artifacts: `*.pt`, `*.pth`, `*.onnx`, `*.ckpt`
- Robotics recording files: `*.bag`, `*.db3`

## 6. Why Cleanup Comes Before Migration

The migration should operate on intentional project assets, not on local caches or generated run outputs. Cleaning the Git index first reduces review noise, prevents accidental commits of large or machine-specific files, and creates a clear checkpoint before the repository is reorganized into the later multi-project layout.

This also makes Step 1 auditing more reliable because source files, documentation, and configuration can be reviewed separately from local execution artifacts.

## 7. Conditions To Enter Step 1

Step 1 can begin when:

- `.gitignore` contains rules for IDE metadata, Python caches, runtime logs, videos, run folders, checkpoints, and large model or recording artifacts.
- Already tracked local artifacts have been removed from the Git index with `git rm --cached`.
- Important source, documentation, test, configuration, and script directories remain visible in `git status`.
- The user has reviewed the Step 0.5 status output and decided whether to create the recommended checkpoint commit.
