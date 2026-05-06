# Project C 依赖与风险

## 已确认依赖信息

README 提到建议环境包括：

- Ubuntu 22.04.4 LTS
- g++ 11.4.0
- MuJoCo
- Pinocchio
- Eigen
- Quill
- GLFW
- JsonCpp
- OpenGL 系统支持

## 当前不执行的原因

- 本次任务只做资料搜集，不编译、不运行仿真。
- 完整人形 MPC + WBC 工程依赖和运行链路较长。
- 直接运行 demo 前，需要先确认模型、参数、MuJoCo 版本和控制频率。

## 技术风险

- WBC_QP 与 MPC 的状态维度不一致会导致调试困难。
- 浮动基索引和 Pinocchio `nq/nv` 差异容易出错。
- MuJoCo sensor ID、joint ID、body ID 映射需要源码确认。
- 真实机器人相关参数不能直接用于教学仿真。

## 当前安全建议

- 先读 DataBus 和 walking demo。
- 先画数据流。
- 先复现简化 WBC-QP。
- 不在 A 项目内直接套 C 的完整架构。
