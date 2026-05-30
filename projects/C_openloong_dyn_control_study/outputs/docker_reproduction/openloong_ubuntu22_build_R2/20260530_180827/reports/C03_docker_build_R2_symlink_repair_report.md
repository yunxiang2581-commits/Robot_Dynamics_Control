# C03 Docker Build R2 Symlink Repair Report

## 1. 本轮目标

R1 已经验证 Docker Ubuntu 22.04 + `gcc/g++ 11.4.0` + CMake configure 可以成功，但 build 在链接 MuJoCo 时失败。

R2 的目标是：

- 不重新设计 Docker。
- 不修改 `external/open_source_repos/OpenLoong-Dyn-Control/` 官方源码目录。
- 只在 R2 输出目录的隔离 worktree 中恢复第三方库符号链接。
- 复用 `openloong-ubuntu22-build:local` 镜像重新执行 CMake configure + build。
- 不运行生成的 demo，不打开 MuJoCo viewer，不生成 MP4。

本轮输入：

```text
external/open_source_repos/OpenLoong-Dyn-Control/
```

本轮输出：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/
```

数学逻辑没有变化。本轮只处理工程构建层面的动态库 symlink 问题，没有修改控制器、IK、Jacobian 或 MuJoCo 仿真逻辑。

## 2. R1 失败摘要

R1 输出目录：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R1/20260530_000000/
```

R1 状态：

- CMake configure：PASS。
- Build：FAIL。
- 第一条真实错误：`third_party/mujoco/lin_x64/libmujoco.so` 被链接器识别为普通文本，而不是 ELF 动态库。

R1 关键证据：

```text
libmujoco.so:       ASCII text, with no line terminators
libmujoco.so.3.1.1: ELF 64-bit LSB shared object, x86-64
```

R1 目标可执行文件均未生成：

```text
walk_wbc NOT FOUND
walk_mpc_wbc NOT FOUND
wbc_speed_test NOT FOUND
jump_mpc NOT FOUND
float_control NOT FOUND
```

## 3. R2 源码复制策略

R2 没有在官方源码目录中构建，而是先复制到 R2 worktree：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/worktree/OpenLoong-Dyn-Control/
```

复制命令：

```text
rsync -a --delete \
  --exclude build \
  --exclude .git/index.lock \
  external/open_source_repos/OpenLoong-Dyn-Control/ \
  outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/worktree/OpenLoong-Dyn-Control/
```

复制日志：

```text
logs/00_source_copy.log
```

## 4. MuJoCo symlink 修复记录

修复前：

```text
libmujoco.so:       ASCII text, with no line terminators
libmujoco.so.3.1.1: ELF 64-bit LSB shared object, x86-64
cat libmujoco.so => libmujoco.so.3.1.1
```

修复命令只在 R2 worktree 中执行：

```text
rm -f libmujoco.so
ln -s libmujoco.so.3.1.1 libmujoco.so
```

修复后：

```text
libmujoco.so -> libmujoco.so.3.1.1
libmujoco.so: symbolic link to libmujoco.so.3.1.1
```

日志：

```text
logs/01_mujoco_lib_before_fix.log
logs/02_mujoco_lib_after_fix.log
```

## 5. 继续修复的 symlink 问题

第一次 R2 build 在 MuJoCo symlink 修复后继续前进，但随后失败在 `qpOASES`：

```text
/usr/bin/ld:/run_root/worktree/OpenLoong-Dyn-Control/third_party/qpOASES/lin_x64/libqpOASES.so: file format not recognized; treating as linker script
/usr/bin/ld:/run_root/worktree/OpenLoong-Dyn-Control/third_party/qpOASES/lin_x64/libqpOASES.so:0: syntax error
```

诊断结果：

```text
libqpOASES.so:     ASCII text, with no line terminators
libqpOASES.so.3.2: ELF 64-bit LSB shared object, x86-64
cat libqpOASES.so => libqpOASES.so.3.2
```

Git 索引中还有多个 `120000` symlink 条目，在当前 `core.symlinks=false` 工作树中被落成普通文本。继续修复时只在 R2 worktree 中恢复这些 symlink：

```text
third_party/glfw/lin_arm64/libglfw.so -> libglfw.so.3
third_party/glfw/lin_arm64/libglfw.so.3 -> libglfw.so.3.3
third_party/glfw/lin_x64/libglfw.so.3 -> libglfw.so.3.2
third_party/mujoco/lin_arm64/libmujoco.so -> libmujoco.so.3.1.1
third_party/mujoco/lin_x64/libmujoco.so -> libmujoco.so.3.1.1
third_party/qpOASES/lin_arm64/libqpOASES.so -> libqpOASES.so.3.2
third_party/qpOASES/lin_x64/libqpOASES.so -> libqpOASES.so.3.2
```

继续修复日志：

```text
logs/07_symlink_index_audit_before_continue_fix.log
logs/08_symlink_continue_fix.log
```

最终 x64 关键库状态：

```text
third_party/mujoco/lin_x64/libmujoco.so -> libmujoco.so.3.1.1
third_party/qpOASES/lin_x64/libqpOASES.so -> libqpOASES.so.3.2
third_party/glfw/lin_x64/libglfw.so.3 -> libglfw.so.3.2
```

## 6. Docker / 容器环境

复用镜像：

```text
openloong-ubuntu22-build:local
sha256:50f0f1ac188510539635f6b0bbc91c8998b84483256acab82eba93544f9ee305
```

容器内环境：

```text
gcc-11: 11.4.0
g++-11: 11.4.0
cmake: 3.22.1
make: GNU Make 4.3
```

注意：R1 镜像中没有安装 `file` 命令。R2 容器脚本已调整为：容器内没有 `file` 时跳过容器内文件类型检查；宿主机侧仍保存了 `file` 诊断证据。

日志：

```text
logs/03_container_env_and_mujoco.log
logs/03_container_env_and_mujoco_attempt1_file_missing.log
```

## 7. CMake configure 结果

命令：

```text
cmake -S . -B build \
  -DCMAKE_C_COMPILER=gcc-11 \
  -DCMAKE_CXX_COMPILER=g++-11
```

结果：PASS。

关键日志：

```text
-- The C compiler identification is GNU 11.4.0
-- The CXX compiler identification is GNU 11.4.0
x86_64
linux x64架构
-- Configuring done
-- Generating done
-- Build files have been written to: /run_root/worktree/OpenLoong-Dyn-Control/build
```

日志：

```text
logs/04_cmake_configure.log
logs/04_cmake_configure_attempt2_qpoases_failure.log
```

## 8. Build 结果

命令：

```text
cmake --build build -j"$(nproc)"
```

结果：PASS。

最终 build 日志显示：

```text
[ 79%] Built target wbc_speed_test
[ 82%] Built target walk_mpc_wbc_joystick
[ 85%] Built target float_control
[ 88%] Built target jump_mpc
[ 91%] Built target walk_wbc
[ 94%] Built target walk_wbc_staircase
[ 97%] Built target walk_wbc_joystick
[100%] Built target walk_mpc_wbc
```

过程备注：

- 第一次 R2 build 未进入 CMake，因为容器内缺少 `file` 命令；已记录为 `attempt1_file_missing`。
- 第二次 R2 build 在 `qpOASES/libqpOASES.so` 链接阶段失败；已记录为 `attempt2_qpoases_failure`。
- 继续恢复 Git symlink 后，第三次 build 成功。
- 宿主机尝试删除旧 `build/` 时遇到 root-owned 文件 `Permission denied`，但容器内增量 configure/build 成功完成。该权限问题来自前面容器以 root 用户写入挂载目录，不影响本轮最终 build 结果。

日志：

```text
logs/05_cmake_build.log
logs/05_cmake_build_attempt2_qpoases_failure.log
```

## 9. 可执行文件检查

| target | expected_path | status |
|---|---|---|
| walk_wbc | build/walk_wbc | FOUND |
| walk_mpc_wbc | build/walk_mpc_wbc | FOUND |
| wbc_speed_test | build/wbc_speed_test | FOUND |
| jump_mpc | build/jump_mpc | FOUND |
| float_control | build/float_control | FOUND |

完整生成文件：

```text
build/float_control
build/jump_mpc
build/walk_mpc_wbc
build/walk_mpc_wbc_joystick
build/walk_wbc
build/walk_wbc_joystick
build/walk_wbc_staircase
build/wbc_speed_test
```

检查文件：

```text
build_artifacts/executable_files.txt
build_artifacts/build_targets_check.txt
```

## 10. 当前结论

R2 build-only 复现已经通过。

关键结论：

- R1 的 MuJoCo 链接失败确实来自 symlink 被落成普通文本。
- 修复 MuJoCo 后，build 继续暴露出同类的 `qpOASES` symlink 问题。
- 根因不是 g++11、CMake 或 Ubuntu 22.04 不兼容，而是当前官方源码工作树 `core.symlinks=false` 导致第三方库 symlink 未正确落盘。
- 在 R2 隔离 worktree 中恢复 Git 记录的第三方库 symlink 后，CMake configure + build 成功。

## 11. 下一步建议

建议进入 `C04-DOCKER-SMOKE-R1`，但仍保持保守顺序：

1. 不直接跑 `walk_mpc_wbc`。
2. 先检查 `wbc_speed_test` 源码是否确实不打开 MuJoCo viewer。
3. 如果确认安全，再短时运行 `wbc_speed_test`，保存 stdout/stderr 和退出码。
4. GUI demo，例如 `walk_wbc`，单独规划 X11 / OpenGL / GPU 转发，不和 build-only 复现混在一起。

## 12. 边界确认

本轮确认：

- 未修改 `external/open_source_repos/OpenLoong-Dyn-Control/` 官方源码目录。
- 未在官方源码目录生成 `build/`。
- 未修改 Project A / B / D。
- 未修改 `shared/robot_assets/vendor`。
- 未运行 MuJoCo GUI demo。
- 未运行长时间 MuJoCo。
- 未生成 MP4。
- 未执行 `git add`。
- 未执行 `git commit`。
- 未执行 `git push`。

补充说明：官方源码目录中的 `libmujoco.so` 和 `libqpOASES.so` 仍是普通文本文件，这正好说明本轮没有修官方源码；修复只发生在 R2 输出 worktree 中。
