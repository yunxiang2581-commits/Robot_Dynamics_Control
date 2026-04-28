# Preparation Acceptance Checklist

> Chinese version: [10_preparation_acceptance_checklist.md](10_preparation_acceptance_checklist.md)

## Reading Navigation

| Previous | Current | Next |
| --- | --- | --- |
| [09 Risk and fallback plan](09_risk_and_fallback_plan_en.md) | Preparation acceptance checklist | Next: A/B/C development phases |

## Purpose

This checklist is used to decide whether the preparation phase is complete and whether the repository can move into A/B/C skeleton and implementation work.

## Key Takeaways

| Dimension | Standard |
| --- | --- |
| Documentation | preparation docs are complete and internally linked |
| Repository | README, `.gitignore`, `projects/`, `shared/`, `tools/`, and `outputs/` exist |
| Boundary | preparation docs clearly say algorithms are not implemented yet |
| Next-step readiness | the repository can move into A/B/C code skeleton work |

## Documentation Checklist

- [x] `docs/00_preparation/` exists
- [x] preparation overview exists
- [x] job target and skill matrix exists
- [x] three-track scope exists
- [x] preparation task table exists
- [x] environment requirements exist
- [x] repository structure plan exists
- [x] external project plan exists
- [x] robot model and asset plan exists
- [x] Codex workflow rules exist
- [x] risk and fallback plan exists
- [x] preparation acceptance checklist exists

## Repository Checklist

- [x] `README.md` links to preparation docs
- [x] A/B/C directories exist
- [x] A legacy assets are under `legacy_imported/`
- [x] A root-relocated assets are under `root_imported/`
- [x] `shared/env/` stores environment files
- [x] `tools/export/` stores export scripts
- [x] root-level A code has been relocated out of the repository root
- [x] `.gitignore` contains large-file rules
- [x] the next step points to `projects/A_self_baseline/scripts/01_inspect_urdf.py`

## Conditions To Start Development

- Documentation is complete.
- Boundaries are clear.
- Monorepo structure is usable.
- Risks have fallbacks.
- The Codex workflow is fixed to TODO teaching skeletons first.
