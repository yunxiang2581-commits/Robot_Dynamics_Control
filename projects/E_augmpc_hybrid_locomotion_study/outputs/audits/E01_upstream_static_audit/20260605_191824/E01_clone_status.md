# E01 Clone Status

本次 E01 仅执行上游仓库克隆与静态读取，不运行上游脚本、不安装依赖、不启动容器、不下载公开 bundle。

| Target | Source | Local Path | Status | Note |
|---|---|---|---|---|
| LRHControl | https://github.com/AndrePatri/LRHControl | `projects/E_augmpc_hybrid_locomotion_study/external/open_source_repos/LRHControl/` | cloned | branch `ibrido` |
| IBRIDO | https://github.com/AndrePatri/IBRIDO | `projects/E_augmpc_hybrid_locomotion_study/external/open_source_repos/IBRIDO/` | cloned | branch `main` |
| ibrido-containers | https://github.com/AndrePatri/ibrido-containers | `projects/E_augmpc_hybrid_locomotion_study/external/open_source_repos/ibrido-containers/` | cloned | branch `main` |
| AugMPCModels | https://huggingface.co/AndrePatri/AugMPCModels | not downloaded | metadata only | Hugging Face public page only |

## Boundaries

- Did clone public GitHub repositories.
- Did read README, license, entrypoint, and file layout statically.
- Did not run `LRHControl`, `IBRIDO`, Isaac Sim, MuJoCo, ROS bag replay, evaluation, or training.
- Did not install Python packages, system packages, or container runtime dependencies.
- Did not modify upstream repository contents.
