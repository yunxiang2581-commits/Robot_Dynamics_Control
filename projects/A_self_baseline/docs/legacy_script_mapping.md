# Legacy Script Mapping

| legacy 脚本                                                                          | 初步用途                 | 对应标准脚本                              | 是否已迁移 | 是否需要重构 | 备注                                         |
| ------------------------------------------------------------------------------------ | ------------------------ | ----------------------------------------- | ---------- | ------------ | -------------------------------------------- |
| `check_docker_env.py`                                                              | 环境检查                 | `01_model_inspect.py` 或独立环境检查脚本 | 是         | 是           | 先作为环境参考, 是否保留为标准脚本待人工确认 |
| `fk_h1.py`                                                                         | H1 frame 正运动学        | `02_configuration_site_pose.py`                   | 是         | 是           | 可参考 FK 路径、frame 查询和文本输出         |
| `fk_h1_left_knee_perturb.py`                                                       | 膝关节扰动下 FK 观察     | `02_configuration_site_pose.py`                   | 是         | 是           | 用于后续验证关节扰动对 frame pose 的影响     |
| `jacobian_h1.py`                                                                   | H1 frame Jacobian 计算   | `03_site_jacobian_check.py`               | 是         | 是           | 可参考 Pinocchio Jacobian API                |
| `jacobian_h1_check.py`                                                             | Jacobian 有限差分检查    | `03_site_jacobian_check.py`               | 是         | 是           | 可参考误差验证和图表输出                     |
| `ik_h1_left_foot_step1_target.py`                                                  | IK 目标构造              | `04_dls_differential_ik.py`                     | 是         | 是           | DLS IK 分步学习材料                          |
| `ik_h1_left_foot_step2_jpos.py`                                                    | IK 初始关节配置/目标检查 | `04_dls_differential_ik.py`                     | 是         | 是           | DLS IK 分步学习材料                          |
| `ik_h1_left_foot_step3_one_step.py`                                                | IK 单步更新              | `04_dls_differential_ik.py`                     | 是         | 是           | 可参考单步 dq 计算思路                       |
| `ik_h1_left_foot_step4_apply_update.py`                                            | IK 应用更新              | `04_dls_differential_ik.py`                     | 是         | 是           | 可参考 `pin.integrate` 使用方式            |
| `ik_h1_left_foot_step5_loop.py`                                                    | IK 迭代 loop             | `04_dls_differential_ik.py`                     | 是         | 是           | 可参考误差历史和收敛报告                     |
| `mujoco_h1_left_knee_perturb.py`                                                   | MuJoCo 膝关节扰动仿真    | `06_target_mocap_tracking.py`              | 是         | 是           | 可参考 MuJoCo 加载、步进和日志               |
| `mujoco_h1_sim_learning.py`                                                        | MuJoCo 基础仿真学习      | `06_mujoco_pd_track                       |            |              |                                              |
| 本步骤只做审计、复制和文档更新。未实现 FK、Jacobian、IK、QP、WBC、MuJoCo 控制或 RL。 |                          |                                           |            |              |                                              |

## 10. 下一步

下一步进入 A01 model inspect：读取 `scene.xml`，检查 `nq`、`nv`、`nu`、joint、body、site、actuator 和 keyframe。
ing.py `| 是         | 是           | 用途细节待人工确认                           | |`mujoco_playback_ik_traj.py `           | IK 轨迹 MuJoCo 回放      |`06_target_mocap_tracking.py `                                                                                                                                                                                                                         | 是         | 是           | 后续可作为轨迹回放参考, 不等同 PD 控制       | |`task1_inspect_humanoid_model.py `      | 模型关节/frame 检查      |`01_model_inspect.py`                                                                                                                                                                                                                                | 是         | 是           | 可参考模型摘要输出                           |
