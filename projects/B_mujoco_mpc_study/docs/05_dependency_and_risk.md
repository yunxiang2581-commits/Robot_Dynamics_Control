# Project B 依赖与风险

## 已确认依赖信息

公开 README 提到完整构建 MJPC GUI 需要：

- CMake
- Ninja
- clang-12
- MuJoCo
- OpenGL 相关库
- 可选 gRPC service

README 还提示 gRPC 是较大依赖，初次下载可能耗时较长。

## 当前不执行的原因

- 本次任务明确不 clone、不下载大文件、不编译、不运行仿真。
- MJPC 完整构建超出当前资料整理范围。
- Python API 仍偏实验性，README 提醒兼容性错误可能较难调试。

## 技术风险

- Windows 未被 README 列为测试平台。
- task residual / transition 与模型不匹配时，错误可能难定位。
- Gradient Descent planner 对 cost scale 敏感。
- 直接迁移到 A 项目会引入过多工程复杂度。

## 当前安全建议

- 先读文档和源码结构。
- 先自己写极简 MPC demo。
- 等 A 的 MuJoCo tracking 基线稳定后，再考虑把 horizon cost 引入 A 的新脚本。
