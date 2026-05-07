# A 自研机器人运动控制基础系统

## 当前状态

本目录是 A 项目的文档占位。准备阶段不实现算法，只记录后续文档入口。

## 后续内容

| 文档 | 目标 | 验收标准 |
| --- | --- | --- |
| `01_model_loading.md` | Pinocchio 加载 URDF | 能说明 nq、nv、joint、frame |
| `02_fk_jacobian_validation.md` | FK、Jacobian、有限差分验证 | 能输出误差 norm 和图表 |
| `03_dls_ik.md` | DLS-IK | 能说明阻尼最小二乘公式和收敛曲线 |
| `04_qp_ik.md` | QP-IK | 能说明变量、目标函数、关节约束 |
| `05_mujoco_pd_tracking.md` | MuJoCo PD 轨迹跟踪 | 能输出跟踪误差和视频 |
| `06_mini_wbc_qp.md` | 教学版 Mini-WBC QP | 能说明任务、约束和输出力矩 |

