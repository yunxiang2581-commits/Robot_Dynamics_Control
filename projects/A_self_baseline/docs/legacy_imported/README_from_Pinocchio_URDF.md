# Robot Dynamics Control Skeleton

## 项目名称
Robot Dynamics Control

当前仓库名称：`Pinocchio_URDF`

## 项目目标
构建一个干净、可扩展的机器人动力学控制学习项目骨架，便于后续逐步加入仿真、建模与控制模块。

本仓库当前主要用于学习：

- 机器人 URDF 模型加载
- Pinocchio 正运动学、Jacobian、逆运动学
- 后续结合 MuJoCo 做可视化验证
- 服务于机械臂、四足、人形机器人运动控制学习

## 当前阶段
仅完成独立目录初始化与基础说明文档。

## 基础环境

- Python / Conda 环境：`robot311`
- Git：用于本地版本控制和后续远程同步
- Docker：用于固定运行环境，减少系统环境差异
- Codex：用于辅助代码修改、检查 diff、解释实现逻辑

## 常用命令

```bash
git status
git diff --stat
git diff
python -m pytest
docker compose config
docker compose build
```

## 安全提交注意事项

- 不提交 `.env`、密钥、token、账号配置等敏感信息。
- 不提交日志、缓存、仿真输出视频、ROS bag、大型生成文件。
- 大文件后续考虑使用 Git LFS 或单独存储，不直接放入普通 Git 历史。

## 目录结构说明
- `controllers/`: 控制器模块
- `experiments/`: 实验脚本模块
- `envs/`: 仿真环境文件（如 MuJoCo XML）
- `models/urdf/`: URDF 模型目录
- `models/meshes/`: 网格资源目录
- `models/mjcf/`: MJCF 模型目录
- `tests/`: 测试脚本目录
- `scripts/`: 工具脚本目录
- `notes/`: 学习与安装笔记
- `outputs/`: 输出结果目录
- `third_party/`: 第三方资源目录
- `src/`: 预留源码目录

## 后续计划（占位）
- 环境搭建
- MuJoCo
- Pinocchio
- URDF
- FK/IK
- Dynamics
- WBC

## Docker Development Environment

Docker is used to provide a reproducible and isolated development environment for this project.

The local `robot311` conda environment can still be used for fast daily development. Docker is mainly used for reproducibility, isolation, and future handoff.

### Build image

```bash
docker compose build
```

### Start an interactive shell

```bash
docker compose run --rm robot-dev
```

### Check installed Python packages

```bash
python scripts/check_docker_env.py
```

Notes:

- `environment.yml` uses `conda-forge` packages where possible.
- Pinocchio is installed as `pinocchio` from `conda-forge`.
- MuJoCo is installed through `pip` as `mujoco`.
- `unitree_ros/` is treated as a local third-party resource and is excluded from the Docker build context for now.
