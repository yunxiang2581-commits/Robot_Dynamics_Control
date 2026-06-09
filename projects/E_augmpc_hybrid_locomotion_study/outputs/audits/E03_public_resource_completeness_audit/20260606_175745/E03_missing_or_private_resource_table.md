# E03 Missing Or Private Resource Table

| Resource / Condition | Public? | Evidence | Impact | Blocks Which Stage? | Recommendation |
|---|---|---|---|---|---|
| Kyon private robot-description risk | partial | `ibrido-containers` README says Kyon visualization and evaluations still depend on private robot-description repositories | blocks public Kyon visualization/eval route | later visualization smoke; later eval smoke | keep Kyon out of first public target |
| NGC / IsaacSim access risk | external / conditional | container docs mention NGC-related setup and IsaacSim routes | blocks some Isaac-based eval or training routes | E04; later eval smoke; later training feasibility | list as prerequisite, do not install now |
| Hugging Face large file / xet risk | yes for metadata, no for local assets | HF bundle tree shows `xet` markers on model files | blocks assuming local asset availability | E04; E05; later visualization/eval | treat downloads as delayed future action |
| wandb run access | public link mentioned but not audited in E03 | model card links wandb runs | does not block base roadmap, but limits run-level confirmation | E06; later training feasibility | treat as optional external evidence |
| AugMPCEnvs repo not yet cloned | referenced only | E01 package map and LRHControl README references | limits deeper environment implementation audit | E06; later execution analysis | keep noted as unresolved dependency surface |
| MPCHive repo not yet cloned | referenced only | E01 package map references it | limits control-cluster depth audit | E06; later execution analysis | keep noted as unresolved dependency surface |
| EigenIPC repo not yet cloned | referenced only | E01 package map references it | limits shared-memory dependency audit | E04; E06 | keep noted as unresolved dependency surface |
| MPCViz repo not yet cloned | referenced only | E01 package map references it; README mentions `launch_mpcviz.py` | limits deeper visualization dependency audit | E04; later visualization smoke | keep noted as unresolved dependency surface |
| container runtime not yet audited | not yet | E04 still pending by roadmap | cannot yet declare execution readiness | E04 | do prerequisite checklist next |
| bundle not downloaded yet | no | E03 boundary forbids bundle download | cannot confirm payload integrity locally | later visualization/eval; E05 planning | keep delayed |
| rosbag not downloaded yet | no | E03 boundary forbids rosbag download | cannot replay now | later visualization smoke | keep delayed |
| checkpoint not downloaded yet | no | E03 boundary forbids checkpoint download | cannot eval now | later eval smoke | keep delayed |
| no eval run yet | no | status only | cannot claim runnable evaluation path | later eval smoke | keep delayed |
| no training run yet | no | status only | cannot claim training reproducibility | later training feasibility | keep delayed |
