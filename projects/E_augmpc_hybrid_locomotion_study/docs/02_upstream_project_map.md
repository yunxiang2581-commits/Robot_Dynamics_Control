# Upstream Project Map

Project E 后续优先静态审查以下上游：

- LRHControl / AugMPC
- IBRIDO
- `ibrido-containers`
- AugMPCEnvs
- AugMPCModels
- MPCHive
- EigenIPC
- MPCViz

## What To Look For

- README / LICENSE
- 容器入口
- public model bundle
- rosbag / dataset / eval 入口
- visualization 路径
- training 是否被拆成独立阶段
- high-level RL 与 low-level MPC 的接口划分
