# E01 Entrypoints For Eval And Training

## LRHControl Script Entrypoints

- `aug_mpc/scripts/launch_world_interface.py`
- `aug_mpc/scripts/launch_control_cluster.py`
- `aug_mpc/scripts/launch_train_env.py`
- `aug_mpc/scripts/launch_mpcviz.py`
- `aug_mpc/scripts/replay_bag.bash`

## Static Reading Of Roles

- `launch_world_interface.py`
  - world interface bootstrapping
  - default public path points to `aug_mpc_envs.world_interfaces.isaac_world_interface`
- `launch_control_cluster.py`
  - MPC cluster client bootstrap
  - default client path points to `aug_mpc.controllers.rhc.hybrid_quad_client`
- `launch_train_env.py`
  - training / resume / eval unified entry
  - exposes SAC / PPO related switches and bundle/model path arguments
- `replay_bag.bash`
  - bag replay helper, not itself a simulator launcher

## Container-Level Entrypoints

- `ibrido_u20/singularity/execute.sh`
- `ibrido_u22/singularity/execute.sh`
- `ibrido_u24/singularity/execute.sh`
- `ibrido_u24/singularity/utils/launch_bundle.sh`

## Training Config Surface Seen In Containers

- `u20`: centauro, joy_cfg.sh, kyon02, kyon_simple, training_cfg.sh, unitree_b2w, zmq_cfg.sh
- `u22`: ablations, b2w, centauro, kyon02, kyon_simple, talos
- `u24`: README_profiles.md, common, robots, runs

## Audit Note

E01 只确认“入口存在且分层关系合理”，不对这些入口在当前主机上的实际可执行性做结论。
