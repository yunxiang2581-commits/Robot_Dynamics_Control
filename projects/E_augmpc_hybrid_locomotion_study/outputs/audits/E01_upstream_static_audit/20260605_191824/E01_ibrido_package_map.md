# E01 IBRIDO Package Map

`IBRIDO` 仓库本身更像顶层索引页，主要职责是说明整体框架组件，而不是承载所有实现代码。

| Package | Role in IBRIDO README | Public Availability in This Audit |
|---|---|---|
| AugMPC | RL-augmented MPC for non-gaited legged and hybrid locomotion | partially visible via `LRHControl` clone |
| AugMPCEnvs | world interfaces and training env implementations | referenced only, repo not cloned in E01 |
| AugMPCModels | demo bundles and configs | metadata only, no bundle download |
| MPCHive | parallel MPC cluster manager | referenced only |
| EigenIPC | shared-memory transport for states/commands/meta | referenced only |
| MPCViz | RViz-based MPC visualization | referenced only |
| ibrido-containers | containerized installation / execution route | cloned and statically inspected |

## Structural Reading

- `IBRIDO/README.md` 给的是框架总表，不是单仓库即完整复现实体。
- 真正和 AugMPC 训练/评估逻辑直接相关的公开代码入口，本次主要落在 `LRHControl`。
- 真正和容器化执行、bundle 路由、Isaac/MuJoCo 分流直接相关的公开入口，本次主要落在 `ibrido-containers`。
- `AugMPCEnvs`、`MPCHive`、`EigenIPC`、`MPCViz` 在 E01 中都只有引用关系，没有进一步拉取验证。
