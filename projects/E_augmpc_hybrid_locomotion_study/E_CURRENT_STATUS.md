# Project E Current Status

Project E 已从旧的 `isaac-quad-loco` / Orbit 方向完全 retarget 到 AugMPC / LRHControl / IBRIDO 复现路线。

| Stage | Status | Note |
|---|---|---|
| E00 retarget cleanup | completed | old `isaac-quad-loco` direction removed; new Project E skeleton established |
| E01 upstream static audit | completed | `LRHControl` / `IBRIDO` / `ibrido-containers` cloned and statically audited; Hugging Face metadata only |
| E02 full-process reproducibility learning roadmap | completed | dual-view roadmap written: management flow + learning flow, with stage dependencies and exit criteria |
| E03 public resource completeness audit | completed | public metadata and file-tree evidence consolidated for bundles, rosbags, configs, and future-stage handoff |
| E04 public resource acquisition | completed | public resource acquisition reached 151/151 target files locally, including the final zero-byte public file |
| E05 delayed reproduction command plan | pending | write future execution commands only; do not run |
| E06 learning report | pending | summarize AugMPC RL + MPC architecture, reproduction boundaries, and learning value |

## Next Recommended Step

`E05 delayed reproduction command plan`

## E04 Snapshot

- `LRHControl` cloned at branch `ibrido`
- `IBRIDO` cloned at branch `main`
- `ibrido-containers` cloned at branch `main`
- `AugMPCModels` public metadata and file tree inspected
- best-effort `wget` retry covered P0-P3 public targets, followed by targeted curl+wget repair on the remaining failures
- public resources downloaded: yes
- first downloaded robot: `Centauro`
- local target files present: `151/151`
- bundle coverage:
  - `centauro/d2026_02_21_h14_m01_s10-CentauroCloopPartialUbNoWheels_FakePosTrackingEnv`: `38/38`
  - `centauro/d2026_03_07_h19_m22_s30-CentauroCloopPartialNoYawUb_FakePosTrackingEnv`: `37/37`
  - `b2w/d2026_03_28_h11_m11_s07-B2WPartialCloopWheels_FakePosTrackingEnv`: `38/38`
  - `centauro/d2026_01_19_h14_m54_s32-CentauroCloopPartialNoYawUbPercep_FakePosTrackEnvPhaseControl`: `38/38`
- remaining failed files: `0`
- rosbag downloaded: yes
- checkpoint downloaded: yes
- `Kyon` remains excluded from first public route because of private-resource risk
- no upstream code executed
- no upstream Python or shell entrypoint executed
- no container started
- visualization: not run
- eval: not run
- training: not run
