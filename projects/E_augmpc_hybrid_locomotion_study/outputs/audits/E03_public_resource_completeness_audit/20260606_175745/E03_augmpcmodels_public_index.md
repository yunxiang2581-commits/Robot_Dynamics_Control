# E03 AugMPCModels Public Index

本文件只记录 `https://huggingface.co/AndrePatri/AugMPCModels` 当前可匿名看到的公开 metadata / 文件树证据，不下载任何 bundle、rosbag、checkpoint。

| Item | Public Evidence | Local Availability | Downloaded Now? | Notes |
|---|---|---|---|---|
| Repository name | Hugging Face page title shows `AndrePatri/AugMPCModels` | no local clone | no | page opens anonymously |
| Public visibility | page and file tree can be opened without authentication | metadata only | no | anonymous access observed |
| License | public page shows `License: gpl-2.0` | metadata only | no | current public metadata |
| Paper linkage | public page shows `arxiv: 2603.10878` | metadata only | no | model card links paper |
| Bundles directory | file tree shows `bundles/` | metadata only | no | directory size shown as `104 MB` |
| Visible robot directories | file tree shows `bundles/b2w` and `bundles/centauro` | metadata only | no | no public `kyon` robot directory observed in current tree |
| Visible bundle names | file tree shows one `b2w` bundle and three `centauro` bundles | metadata only | no | names are visible from tree listing |
| Rosbag evidence | bundle trees show `rosbag_*` directories; model card and `ibrido-containers` mention example rosbags | metadata only | no | supports future visualization path |
| Eval evidence | model card explains `EVAL=1`, `MPATH`, `MNAME`, `N_ENVS=1` | metadata only | no | still only documentation evidence |
| Visualization evidence | `ibrido-containers` README says public bundle behavior can be visualized without launching simulation | local README + public metadata | no | requires future container / GUI validation |
| Centauro mention | visible in public tree and example `MPATH` | metadata only | no | strong first-target candidate |
| B2W mention | visible in public tree | metadata only | no | strong first-target candidate |
| Kyon mention | not seen in current HF public tree; only appears in `ibrido-containers` README | local README only | no | treat as risk / private route, not public first target |
| Bundle manifest | bundle trees show `bundle.yaml` | metadata only | no | visible for centauro and b2w examples |
| Checkpoint-like file | bundle trees show `<bundle_name>_model` | metadata only | no | xet-backed large asset indicator shown |
| Config files | bundle trees show `*.yaml` | metadata only | no | includes controller and impedance config examples |
| URDF / SRDF files | bundle trees show `*.urdf` and `*.srdf` | metadata only | no | visible for centauro and b2w examples |
| Helper scripts / Python | bundle trees show `launch_*.py`, `*_rhc.py`, env helpers | metadata only | no | visible but not executed |
| Preserved run folder | bundle trees show `ibrido_run__*` directories | metadata only | no | aligns with training cfg preservation note |
| Git-LFS / xet signal | checkpoint files show `xet` markers | metadata only | no | treat as large-file / transfer risk |
| Auth required? | not required for page browsing | metadata only | no | later asset download may still hit size / transfer constraints |
| Direct download avoided | yes | yes | no | required by E03 boundary |

## Visible Public Tree Snapshot

- `bundles/b2w/d2026_03_28_h11_m11_s07-B2WPartialCloopWheels_FakePosTrackingEnv`
- `bundles/centauro/d2026_01_19_h14_m54_s32-CentauroCloopPartialNoYawUbPercep_FakePosTrackEnvPhaseControl`
- `bundles/centauro/d2026_02_21_h14_m01_s10-CentauroCloopPartialUbNoWheels_FakePosTrackingEnv`
- `bundles/centauro/d2026_03_07_h19_m22_s30-CentauroCloopPartialNoYawUb_FakePosTrackingEnv`

## E03 Reading

当前公开 metadata 已经足够证明：

- `AugMPCModels` 不是只有 README 级描述，而是确实公开暴露了 bundle 目录树。
- `Centauro` 和 `B2W` 至少各有公开 bundle 证据。
- 当前不能把“页面可见”误写成“本地已可用”，因为本次没有下载任何内容。
