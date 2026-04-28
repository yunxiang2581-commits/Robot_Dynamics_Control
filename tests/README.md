# tests

当前目录是后续单元测试的占位目录。

准备阶段不写算法测试。后续进入 A 项实现阶段后，每个核心模块至少补一个基础测试，重点验证接口、维度、误差下降和约束是否生效。

| 后续测试 | 目标 | 当前阶段状态 |
| --- | --- | --- |
| `test_model_loader.py` | 验证模型加载接口 | 未创建 |
| `test_kinematics.py` | 验证 FK 输出维度 | 未创建 |
| `test_jacobian_check.py` | 验证 Jacobian 误差计算 | 未创建 |
| `test_ik.py` | 验证 IK 误差下降 | 未创建 |
| `test_qp_ik.py` | 验证关节限位约束 | 未创建 |

