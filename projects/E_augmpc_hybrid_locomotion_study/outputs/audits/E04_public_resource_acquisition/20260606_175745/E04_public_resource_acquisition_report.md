# Project E E04 Public Resource Acquisition And Integrity Audit Report

## 1. Scope

- 本次只做公开资源下载、路径整理、hash 记录、manifest 生成。
- 不运行上游代码，不启动容器，不跑 visualization，不跑 eval，不训练。
- 最后一个剩余文件被确认是 Hugging Face 上的 0-byte `__init__.py`，并按远端 0-byte 内容补齐到本地。

## 2. Inputs

- E03 public resource completeness audit outputs
- Hugging Face source: `https://huggingface.co/AndrePatri/AugMPCModels`

## 3. Download Tool Availability

- `git`: present
- `git-lfs`: missing
- `huggingface-cli` / `hf`: missing
- `curl`: present
- `wget`: present

## 4. Download Plan

- P0-P3 public targets were fully attempted.
- Final remaining file was validated as zero-byte and written locally.

## 5. Download Results

- remote target files across P0-P3: `151`
- local files now present under target bundles: `151`
- files still missing after final repair: `0`

### Bundle Coverage

- `centauro/d2026_02_21_h14_m01_s10-CentauroCloopPartialUbNoWheels_FakePosTrackingEnv`: `38/38` files present locally
- `centauro/d2026_03_07_h19_m22_s30-CentauroCloopPartialNoYawUb_FakePosTrackingEnv`: `37/37` files present locally
- `b2w/d2026_03_28_h11_m11_s07-B2WPartialCloopWheels_FakePosTrackingEnv`: `38/38` files present locally
- `centauro/d2026_01_19_h14_m54_s32-CentauroCloopPartialNoYawUbPercep_FakePosTrackEnvPhaseControl`: `38/38` files present locally

## 6. Resource Tree

- downloaded files are rooted at `/home/ubuntu/Robot_Dynamics_Control/projects/E_augmpc_hybrid_locomotion_study/outputs/reproduction/public_resources/AugMPCModels`
- tree listing: `/home/ubuntu/Robot_Dynamics_Control/projects/E_augmpc_hybrid_locomotion_study/outputs/audits/E04_public_resource_acquisition/20260606_175745/E04_downloaded_resource_tree.txt`

## 7. Integrity Manifest

- manifest json: `/home/ubuntu/Robot_Dynamics_Control/projects/E_augmpc_hybrid_locomotion_study/outputs/audits/E04_public_resource_acquisition/20260606_175745/E04_resource_manifest.json`
- manifest md: `/home/ubuntu/Robot_Dynamics_Control/projects/E_augmpc_hybrid_locomotion_study/outputs/audits/E04_public_resource_acquisition/20260606_175745/E04_resource_manifest.md`
- sha256 list: `/home/ubuntu/Robot_Dynamics_Control/projects/E_augmpc_hybrid_locomotion_study/outputs/audits/E04_public_resource_acquisition/20260606_175745/E04_resource_hashes.sha256`

## 8. Size And Storage Summary

- total bytes downloaded across target bundles: `104376741`
- size summary and top largest files are listed in `E04_resource_size_summary.md`

## 9. Remaining Gaps

- no remaining file gaps inside the P0-P3 public target set

## 10. Can Later Visualization / Eval Planning Proceed?

**Yes.**

All targeted public files for P0-P3 are now locally present. This still does not claim runtime validation.

## 11. What Was Not Done

- no AugMPC run
- no IBRIDO run
- no Isaac run
- no MuJoCo run
- no simulation
- no training
- no dependency install
- no container launch
- no upstream source modification
- no git add / commit / push
- no Kyon download
- no private resource download

## 12. Next Step

Recommended next step: `E05 delayed reproduction command plan`.
