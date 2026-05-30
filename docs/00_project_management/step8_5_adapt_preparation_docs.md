# Step 8.5 Adapt Preparation Docs

## 1. Step 8.5 Goal

Adapt the preparation-stage Markdown documents to the current monorepo structure:

```text
Robot_Dynamics_Control/
├── projects/
│   ├── A_self_baseline/
│   ├── B_legged_control_study/
│   └── C_unitree_rl_mjlab_study/
├── shared/
├── tools/
├── docs/
├── exports/
├── outputs/
└── requirements.txt
```

This step only updates Markdown documentation.

## 2. Why Adapt Preparation Docs

The preparation documents were created before the final A/B/C monorepo layout was established. Some descriptions still pointed to earlier root-level or `docs/01_*` placeholders. Updating them prevents future implementation steps from following stale paths.

## 3. Modified Documents

- `README.md`
- `docs/00_preparation/00_preparation_overview.md`
- `docs/00_preparation/03_preparation_task_table.md`
- `docs/00_preparation/05_repository_structure_plan.md`
- `docs/00_preparation/06_external_project_plan.md`
- `docs/00_preparation/07_robot_model_and_asset_plan.md`
- `docs/00_preparation/08_codex_workflow_rules.md`
- `docs/00_preparation/10_preparation_acceptance_checklist.md`
- `projects/A_self_baseline/README.md`
- `projects/B_legged_control_study/README.md`
- `projects/C_unitree_rl_mjlab_study/README.md`

The other preparation documents were checked and did not need theme-level rewrites.

## 4. Main Path Adjustments

- A project paths now point to `projects/A_self_baseline/`.
- B project paths now point to `projects/B_legged_control_study/`.
- C project paths now point to `projects/C_unitree_rl_mjlab_study/`.
- Project documentation now points to `projects/*/docs/`.
- B/C external notes now point to `projects/*/external/`.
- Shared model and environment resources now point to `shared/`.
- Repository tools now point to `tools/`.

Root-level project management paths remain valid:

- `docs/00_preparation/`
- `docs/00_project_management/`
- `docs/04_compare/`
- `docs/interview/`

## 5. Markdown Display Rules Applied

- Replaced the root README preparation-stage wide table with grouped lists.
- Replaced the preparation task table's 7-column table with grouped task lists.
- Kept smaller tables where they remain readable.
- Kept each document's original topic and intent.

## 6. Code Modification Scope

No Python code, algorithm implementation, legacy imported file, or project asset was modified.

This step did not implement FK, Jacobian, IK, QP, WBC, MuJoCo control, or RL logic.

## 7. Next Step

Enter A project implementation from:

```text
projects/A_self_baseline/scripts/01_model_inspect.py
```

The next technical task should be A-01 inspect URDF.

## 8. Validation Notes

Checks performed:

- The three previous root project-doc paths for A/B/C no longer appear in the adapted Markdown scope: root README, `docs/00_preparation/`, A/B/C README files, and B/C external Markdown notes.
- The same old doc paths still appear in historical migration records under `docs/00_project_management/` and in non-Markdown export scripts under `tools/export/`; those files were not changed in this Markdown-only step.
- The previous old-project absolute path does not appear in the adapted preparation docs or project README files. It remains in migration records and legacy sample output text as historical provenance, not as a runtime dependency.

## 9. Acceptance Checklist

- [x] README preparation entry is a grouped list instead of a wide table.
- [x] README points to A/B/C project README files.
- [x] Preparation docs prefer `projects/A_self_baseline/`, `projects/B_legged_control_study/`, and `projects/C_unitree_rl_mjlab_study/`.
- [x] Root project management docs remain under `docs/00_preparation/` and `docs/00_project_management/`.
- [x] A/B/C README files include project document entry sections.
- [x] No Python code was modified.
- [x] No legacy imported content was modified.
