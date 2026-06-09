# Project E E03 Public Resource Completeness Audit Report

## 1. Scope

本次只做公开资源完整性审查：

- 只检查 Hugging Face `AugMPCModels` 当前公开 metadata / 文件树
- 只静态读取本地已 clone 的 `ibrido-containers`、E01、E02 产物
- 不下载 bundle / rosbag / checkpoint
- 不运行 AugMPC / IBRIDO / Isaac / MuJoCo / 仿真 / 训练
- 不安装依赖，不启动容器

## 2. Inputs

### E01 inputs

- `E01_upstream_static_audit_report.md`
- `E01_public_bundle_and_rosbag_audit.md`
- `E01_private_resource_risk.md`
- `E01_reproduction_target_decision.md`
- `E01_container_route_audit.md`
- `E01_entrypoints_eval_training.md`

### E02 inputs

- `E02_full_process_reproducibility_learning_roadmap_report.md`
- `E02_full_process_roadmap_table.md`
- `E02_stage_dependencies.md`
- `E02_exit_criteria.md`

### Upstream/local paths

- `external/open_source_repos/LRHControl/`
- `external/open_source_repos/IBRIDO/`
- `external/open_source_repos/ibrido-containers/`
- public page: `https://huggingface.co/AndrePatri/AugMPCModels`

## 3. AugMPCModels Public Metadata Summary

当前公开 metadata 已能确认：

- repository name: `AndrePatri/AugMPCModels`
- anonymous public access is available for page and file tree
- reported license: `gpl-2.0`
- reported paper linkage: `arxiv: 2603.10878`
- visible public bundle root: `bundles/`
- visible robot directories: `b2w`, `centauro`
- visible bundle names include one `b2w` bundle and three `centauro` bundles
- model card and file tree both mention / imply rosbags, eval route, bundle manifests, configs, URDF/SRDF, checkpoints, helper scripts
- checkpoint files show `xet` markers, so large-file transfer considerations remain relevant for later stages

## 4. Public Bundle Candidates

当前最明确的公开候选是：

- `Centauro`
- `B2W`

`Kyon` 在当前 HF 公共文件树中没有看到公开 robot bundle 目录；它只在 `ibrido-containers` README 中作为文档路线出现，并明确伴随 private robot-description 风险。因此它只能作为风险对象，不应作为第一公开目标。

## 5. Centauro and B2W First Target Assessment

结论保持不变：`Centauro` 和 `B2W` 仍然是第一公开目标候选，其中 `Centauro` 证据最强。

原因：

- `Centauro` 有最完整的“三联证据”：
  - HF tree visible bundle
  - README explicit rosbag path example
  - model card explicit eval `MPATH` / `MNAME` example
- `B2W` 也有强证据：
  - HF tree visible bundle
  - HF tree visible rosbag entry
  - README explicitly says public B2W bag visualization remains available

## 6. Rosbag / Visualization Evidence

公开证据足以支持“未来存在 rosbag visualization smoke 的合理性”，但还不足以支持“当前本地已具备 visualization”。

当前能确认的只有：

- public rosbag metadata exists
- Centauro example rosbag path is documented in README
- B2W rosbag directory is visible in the HF tree
- README says `viz_bag.sh` eventually launches bag replay and `launch_mpcviz.py`
- visualization route is documented inside a container-centered workflow and therefore still depends on future E04 prerequisite checking

## 7. Config / Checkpoint / Asset Completeness

当前公开 metadata 已能看到或强烈暗示以下资源类型：

- `bundle.yaml`
- `*_model` checkpoint-like file
- `*.yaml` configs
- `*.urdf`
- `*.srdf`
- helper python scripts
- `ibrido_run__*` preserved run folders
- rosbag directories

但 E03 仍然不能把这些资源视作“已可用”，因为：

- none were downloaded locally
- payload integrity was not checked
- model / bag / config compatibility was not executed or validated

## 8. Missing / Private / External Resource Risks

本次最重要的缺口和风险有：

- `Kyon` private robot-description dependency
- `NGC / IsaacSim` related external setup risk
- Hugging Face large-file / xet transfer risk
- wandb run link exists but was not audited deeply
- `AugMPCEnvs` / `MPCHive` / `EigenIPC` / `MPCViz` are still referenced surfaces rather than locally audited repos
- bundle / rosbag / checkpoint are still not downloaded
- eval / training have still not been run

## 9. Resource-to-Stage Mapping

E03 的交接关系现在清楚：

- 交给 `E04` 的，是前置条件相关资源风险：container runtime, mounts, storage, GUI, GPU, external auth
- 交给 `E05` 的，是 bundle path, model naming, config path, rosbag naming and future command structure
- 交给 `E06` 的，是 remaining ecosystem gaps and learning interpretation

## 10. E03 Decision

### Public resource path completeness

**Decision: partially documented**

原因：

- metadata and file tree evidence are already strong enough to identify concrete public targets
- but resource usability is still incomplete until future download / prerequisite / command stages

### First future target

**Decision: keep `Centauro` / `B2W` public bundle + rosbag visualization / inspection as the preferred first public target, with `Centauro` first.**

### Can E04 proceed?

**Yes.**

E03 已经足够支撑 E04 去列：

- Apptainer / container requirements
- GUI / display requirements
- mount / storage / `training_data` path expectations
- large-file transfer awareness
- private-route exclusions such as Kyon

### Should actual download or run still be delayed?

**Yes.**

E03 之后仍然不应直接执行下载或运行；应先完成 `E04 execution prerequisites checklist`。

## 11. What Was Not Done

- no AugMPC run
- no IBRIDO run
- no Isaac run
- no MuJoCo run
- no simulation
- no training
- no dependency install
- no container launch
- no Hugging Face bundle download
- no rosbag download
- no checkpoint download
- no upstream source modification
- no git add / commit / push

## 12. Next Step

推荐下一步：

`E04 execution prerequisites checklist`

该阶段应只列：

- Apptainer / Singularity
- NGC / IsaacSim access assumptions
- GPU / display / GUI requirements
- host-side mount and storage expectations
- container-facing `training_data` path assumptions

不建议在 E04 之前直接运行任何 bundle / rosbag / eval 命令。
