# E03 Config Checkpoint Asset Check

| Resource Type | Public Evidence | Needed For Visualization | Needed For Eval | Needed For Training | Downloaded Now? | Risk |
|---|---|---|---|---|---|---|
| bundle.yaml | found in HF bundle trees | likely yes | likely yes | possibly useful | no | none in metadata stage |
| training config | model card mentions `ibrido_run_*/training_cfg_*.sh` preserved in bundle | not needed for minimal visualization understanding | yes | yes | no | cannot confirm exact files without download |
| eval config | model card + README describe `EVAL=1`, `MPATH`, `MNAME`, matching training cfg folders in containers | no | yes | no | no | requires later container/config alignment |
| checkpoint | visible `<bundle_name>_model` files with xet markers | not needed for bag-only visualization | yes | yes | no | large-file transfer risk |
| URDF | visible `*.urdf` in HF bundle trees | possibly useful for visualization tooling | yes | yes | no | not locally available |
| SRDF | visible `*.srdf` in HF bundle trees | possibly useful | yes | yes | no | not locally available |
| robot description | visible for Centauro/B2W bundle examples; private risk remains for Kyon | yes for some visualization routes | yes | yes | no | Kyon blocked by private repos |
| rosbag | visible `rosbag_*` directories in HF bundle trees and README | yes | no | no | no | download still required later |
| helper scripts | visible `launch_*.py`, `*_rhc.py`, env helpers in bundle trees | indirectly | indirectly | indirectly | no | should not be treated as executed now |
| terrain/world config | not explicitly isolated by name in current metadata | cannot confirm | cannot confirm | possibly | no | cannot confirm without download |
| MPC config | visible controller yaml / rhc python names | possibly | yes | yes | no | metadata only |
| policy config | bundle naming + training cfg preservation imply presence; exact file cannot always be isolated | no | yes | yes | no | cannot confirm without download |
| wandb metadata | model card links training runs on wandb | no | not required for base eval path | useful for study only | no | external web dependency |
| NGC/IsaacSim image access | documented in container docs, not in HF bundle itself | no | needed for Isaac-based routes | needed for some training/eval routes | no | external auth/setup risk |
| container definition | present locally in `ibrido-containers` repo, not in HF bundle | no | yes | yes | no | handled in E04/E05 |
