# E01 Upstream Static Audit Report

## Scope

本报告覆盖 Project E 的 E01 阶段：
- clone `LRHControl`
- clone `IBRIDO`
- clone `ibrido-containers`
- 读取 `AugMPCModels` Hugging Face 公开元数据
- 不运行上游代码，不安装依赖，不启动容器，不下载公开 bundle

## High-Level Findings

1. 上游公开资料已经形成比较清晰的三层结构：
   - `IBRIDO` 是框架索引
   - `LRHControl` 是 AugMPC 公开算法 / 控制主仓库
   - `ibrido-containers` 是官方推荐执行入口
2. 公开材料明确偏向 `Apptainer / Singularity` 路线，而不是 Docker。
3. `AugMPCModels` 公开页更像 bundle 元数据与使用约定说明；真正消费这些 bundle 的推荐方式仍是 `ibrido-containers`。
4. 公开路径中确实存在私有资源风险，尤其是 Kyon 相关 robot-description 依赖与 IsaacSim/NGC 前置条件。
5. 因此，Project E 的第一个运行目标不应是训练，而应是更轻的公开 bundle / rosbag 可视化或检查链路。

## Cloned Repositories

- `LRHControl`: `25e1674188079fafee6a67b63dbd9b10211c876f` on `ibrido`
- `IBRIDO`: `2cb0ec5366b4f46cbb09933c35b91b9613f42071` on `main`
- `ibrido-containers`: `b673a74198db43530ddb6477d5c9cad1ccfdb8ad` on `main`

## Public Metadata Source

- Hugging Face: https://huggingface.co/AndrePatri/AugMPCModels
- Reported license: `gpl-2.0`
- Reported summary: `Demo bundles for AugMPC`
- Reported related paper id: `2603.10878`

## Recommended First Reproduction Target

`Centauro` or `B2W` public bundle / rosbag visualization-oriented route, before any IsaacSim-heavy or training-heavy attempt.

## Recommended Next Step

`E02 container readiness audit`

## Supporting Files

- `E01_clone_status.md`
- `E01_git_snapshots.md`
- `E01_repo_file_trees.txt`
- `E01_license_summary.md`
- `E01_ibrido_package_map.md`
- `E01_augmpc_architecture_map.md`
- `E01_container_route_audit.md`
- `E01_public_bundle_and_rosbag_audit.md`
- `E01_entrypoints_eval_training.md`
- `E01_private_resource_risk.md`
- `E01_reproduction_target_decision.md`

## Constraint Confirmation

- Upstream repositories were cloned but not modified.
- No upstream Python entrypoint was executed.
- No container was built or started.
- No public model bundle or ROS bag was downloaded.
- No Project A/B/C/D content was touched.
