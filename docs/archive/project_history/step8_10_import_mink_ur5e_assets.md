# Step 8.10 Import Mink UR5e Assets

## 1. 为什么要从 mink_upstream 中选择文件

A 项目当前主线是对标 mink UR5e 示例的教学版 6-DOF 机械臂控制 baseline。为了后续实现 A01-A07，需要本地可检查的 UR5e MuJoCo 模型资产，以及两个最小对照脚本。

本步骤只选择 A 项目当前需要的最小集合，不复制完整 mink 仓库。

## 2. mink_upstream 的用途

`external/mink_upstream/` 是本地上游完整仓库镜像，用于只读参考。它不是 A 项目标准入口，也不应整体提交为 A 项目实现。

## 3. 复制了哪些文件

模型资产：

- `external/mink_upstream/examples/universal_robots_ur5e/`
- 复制到 `shared/robot_assets/models/mink_universal_robots_ur5e/`

对照脚本：

- `external/mink_upstream/examples/arm_ur5e.py`
- 复制到 `projects/A_self_baseline/external/mink/examples/arm_ur5e.py`
- `external/mink_upstream/examples/arm_ur5e_actuators.py`
- 复制到 `projects/A_self_baseline/external/mink/examples/arm_ur5e_actuators.py`

引用信息：

- `external/mink_upstream/README.md`
- `external/mink_upstream/LICENSE`
- `external/mink_upstream/pyproject.toml`
- 复制到 `projects/A_self_baseline/external/mink/`

## 4. 没有复制哪些文件

- `.git/`
- `src/mink/`
- `tests/`
- `docs/`
- `benchmarks/`
- `uv.lock`
- 其他机器人 examples
- `__pycache__`
- `.venv` / `venv`
- 构建缓存
- 大型无关资产

## 5. 为什么不复制 src/mink

A 项目不是完整复刻 mink，也不是直接调用 mink 替代自己的实现。`src/mink/` 只作为上游源码参考保留在 `external/mink_upstream/` 中。

当前 copied files are reference assets/examples, not our implementation。标准实现仍然放在：

- `projects/A_self_baseline/scripts/`
- `projects/A_self_baseline/src/robot_baseline/`

## 6. 资产大小检查结果

- `external/mink_upstream/examples/universal_robots_ur5e/` 总大小：`32M`
- 超过 20MB 的文件：未发现

## 7. 冲突检查结果

复制前未发现目标同名文件冲突。

## 8. A 项目后续如何使用这些资产

- A01 使用 `shared/robot_assets/models/mink_universal_robots_ur5e/scene.xml` 做 model inspect。
- A02 使用同一模型做 configuration / site pose。
- A03 使用同一模型做 site Jacobian check。
- A04/A05 参考 `arm_ur5e.py` 的任务和 limit 组织方式，但不直接调用 mink 替代实现。
- A07 参考 `arm_ur5e_actuators.py` 的 actuator tracking 思路。

## 9. 未实现算法说明

本步骤只做审计、复制和文档更新。未实现 FK、Jacobian、IK、QP、WBC、MuJoCo 控制或 RL。

## 10. 下一步

下一步进入 A01 model inspect：读取 `scene.xml`，检查 `nq`、`nv`、`nu`、joint、body、site、actuator 和 keyframe。
