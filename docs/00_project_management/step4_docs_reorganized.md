# Step 4 Docs Reorganized

## 1. Step 4 Goal

Organize existing Markdown documents into the independent A, B, and C project containers while keeping root-level project management and cross-project documents in place.

This step only copies Markdown documents. It does not migrate old project source code, move root documents, delete files, or implement algorithms.

## 2. Documents Copied Into A

From `docs/01_self_baseline/` to `projects/A_self_baseline/docs/`:

```text
README.md
```

Existing A legacy documents remain under:

```text
projects/A_self_baseline/docs/legacy_imported/
```

## 3. Documents Copied Into B

From `docs/02_legged_control/` to `projects/B_legged_control_study/docs/`:

```text
README.md
```

## 4. Documents Copied Into C

From `docs/03_unitree_rl_mjlab/` to `projects/C_unitree_rl_mjlab_study/docs/`:

```text
README.md
```

## 5. Documents Kept As Root Project Documents

The following directories remain under root `docs/` and were not moved:

```text
docs/00_preparation/
docs/00_project_management/
docs/04_compare/
docs/interview/
```

Root docs stay as overall planning, project management, comparison, and interview preparation material.

## 6. File Conflicts

No file conflicts occurred in this step.

If a future document copy finds a target file with the same relative path, the copy should be skipped and the conflict should be recorded in:

```text
docs/00_project_management/step4_doc_merge_conflicts.md
```

## 7. Future Documentation Maintenance Rules

- A self-developed baseline documents should live under `projects/A_self_baseline/docs/`.
- B legged_control reproduction documents should live under `projects/B_legged_control_study/docs/`.
- C unitree_rl_mjlab reproduction documents should live under `projects/C_unitree_rl_mjlab_study/docs/`.
- Cross-project planning and migration records should remain under `docs/00_project_management/`.
- Preparation documents should remain under `docs/00_preparation/` unless a later step explicitly creates project-specific copies.
- Do not overwrite same-name project documents without recording and reviewing the conflict.

## 8. Acceptance Checklist

- [x] Markdown from `docs/01_self_baseline/` was copied to A project docs.
- [x] Markdown from `docs/02_legged_control/` was copied to B project docs.
- [x] Markdown from `docs/03_unitree_rl_mjlab/` was copied to C project docs.
- [x] Root project documents were not deleted or moved.
- [x] No source code was migrated.
- [x] No algorithms were implemented.
- [x] A/B/C README files now include document entry sections.
- [x] File conflict status is documented.
