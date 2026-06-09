# E01 AugMPC Architecture Map

## Why This Matters

这一步的目标是确定“公开复现路径到底由哪些层组成”。如果架构边界都没理清，后面的容器检查、bundle 检查、评估烟测都会误判。

## Static Architecture View

1. `LRHControl/aug_mpc/world_interfaces/*`
   - 世界接口层，负责把仿真器或真实系统状态暴露给 AugMPC。
   - README 指向 Isaac Sim、MuJoCo、XBot2 / hardware 的接口适配路线。
2. `LRHControl/aug_mpc/controllers/*`
   - MPC 集群与控制器客户端逻辑。
   - `launch_control_cluster.py` 默认入口连到 `hybrid_quad_client`。
3. `LRHControl/aug_mpc/training_envs/*`
   - 训练 / 评估环境抽象，维护观测、动作、奖励、终止、截断等定义。
4. `LRHControl/aug_mpc/training_algs/*`
   - RL 算法实现，公开可见至少包含 SAC 与 PPO。
5. `LRHControl/aug_mpc/agents/*`
   - 策略 / actor-critic 相关模型封装。
6. `ibrido-containers/*/singularity/*`
   - 负责把运行环境、容器镜像、bundle / cfg 路由串起来。
7. `AugMPCModels` public page
   - 公开 bundle 元数据与推荐使用路线，不直接等于“本地可运行结果”。

## Observed Public Entry Points

- `aug_mpc/scripts/launch_world_interface.py`
- `aug_mpc/scripts/launch_control_cluster.py`
- `aug_mpc/scripts/launch_train_env.py`
- `ibrido_u20/singularity/execute.sh`
- `ibrido_u22/singularity/execute.sh`
- `ibrido_u24/singularity/execute.sh`
- `ibrido_u24/singularity/utils/launch_bundle.sh`

## Practical Reading

- `LRHControl` 更像算法与控制主仓库。
- `ibrido-containers` 更像官方推荐运行入口。
- `IBRIDO` 更像项目导航总索引。
- 公开页面已经暗示：真正复现时，不应绕过容器路线直接猜测本地裸跑。
