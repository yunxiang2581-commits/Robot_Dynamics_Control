# Project E: AugMPC Hybrid Locomotion Reproduction

Project E 已完全从 `isaac-quad-loco` / Orbit 方向重构。新目标是复现和学习 AugMPC / LRHControl / IBRIDO 中的 RL-augmented MPC locomotion 结构。

## Core Focus

- high-level RL agent 如何输出 contact schedules 和 twist commands
- low-level MPC controllers 如何执行底层动力学控制
- 优先复现 public bundles / visualization / eval，而不是一开始训练

## Reproduction Priority

1. upstream static audit
2. container readiness audit
3. public model bundle / rosbag audit
4. visualization smoke
5. public model eval smoke
6. minimal training only after eval works

## Project Boundaries

- simulation-only
- no real robot deployment
- no hardware drivers
- no sim2real claim
- no full paper reproduction claim at this stage
- no private robot resources

## Environment Strategy

- container-first
- prioritize IBRIDO / `ibrido-containers`
- Apptainer / Singularity should be checked
- Docker can be evaluated but not assumed

## License Handling

- record upstream license
- do not copy GPL source into unrelated closed code

## Working Directories

- `docs/`: 复现范围、容器路线、上游映射、路线图
- `scripts/`: 可运行的只读审查和状态收集工具
- `outputs/`: audits、manifests、figures、videos、metrics、reproduction 产物
- `external/open_source_repos/`: 上游只读参考目录
