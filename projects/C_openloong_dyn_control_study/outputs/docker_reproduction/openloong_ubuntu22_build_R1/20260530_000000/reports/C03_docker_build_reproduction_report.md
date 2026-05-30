# C03 Docker build-only reproduction report

## 1. 本轮目标

本轮任务是复现 `OpenLoong-Dyn-Control` 的官方构建环境，而不是运行仿真 demo：

- 使用 Ubuntu 22.04 Docker 环境。
- 使用 `gcc-11` / `g++-11`，对齐官方推荐的 11.4.0 编译器版本。
- 在容器内执行 CMake configure 和 build。
- 保存命令、日志、构建产物检查结果和本报告。

本轮输入是官方源码目录的隔离副本：

```text
external/open_source_repos/OpenLoong-Dyn-Control/
```

本轮输出是复现日志目录：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R1/20260530_000000/
```

数学逻辑没有变化。本轮没有修改 IK、Jacobian、控制器或 MuJoCo 仿真逻辑，只验证工程构建链路。

## 2. 宿主机状态

宿主机检查结果：

```text
Ubuntu: Ubuntu 25.10
g++: g++ (Ubuntu 15.2.0-4ubuntu4) 15.2.0
cmake: command not found
Docker: Docker version 29.4.1, build 055a478
Docker Compose: Docker Compose version v5.1.3
```

结论：宿主机本身不是官方推荐的 Ubuntu 22.04 + g++ 11 环境，因此本轮选择 Docker build-only 复现路线是合适的。

## 3. Docker 镜像

本轮新增 Docker 构建辅助文件：

```text
projects/C_openloong_dyn_control_study/docker/openloong_ubuntu22_build/Dockerfile
projects/C_openloong_dyn_control_study/docker/openloong_ubuntu22_build/build_image.sh
projects/C_openloong_dyn_control_study/docker/openloong_ubuntu22_build/run_build_in_container.sh
```

镜像构建结果：成功。

```text
image tag: openloong-ubuntu22-build:local
image id: sha256:50f0f1ac188510539635f6b0bbc91c8998b84483256acab82eba93544f9ee305
created: 2026-05-30T17:44:24.298619085+08:00
size: 229387499 bytes
```

镜像安装的主要依赖包括：

```text
build-essential cmake gcc-11 g++-11 git make pkg-config
libglu1-mesa-dev freeglut3-dev libgl1-mesa-dev
libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
python3 python3-pip rsync
```

完整镜像构建日志：

```text
logs/05_docker_build_image.log
```

## 4. 源码复制策略

本轮没有在官方源码目录中直接运行 CMake/build，而是先复制到本轮输出目录：

```text
worktree/OpenLoong-Dyn-Control/
```

复制命令由 `run_build_in_container.sh` 执行，核心策略是：

```text
rsync -a --delete --exclude build --exclude .git/index.lock \
  external/open_source_repos/OpenLoong-Dyn-Control/ \
  outputs/.../worktree/OpenLoong-Dyn-Control/
```

这样做的目的：

- 避免在官方源码目录下生成 `build/`。
- 避免本轮构建过程直接污染官方源码。
- 把本轮失败现场保存在 `outputs/docker_reproduction/.../worktree/`，便于复盘。

源码复制日志：

```text
logs/04_source_copy.log
```

## 5. 容器内环境

容器内版本检查结果：

```text
gcc-11: gcc-11 (Ubuntu 11.4.0-1ubuntu1~22.04.3) 11.4.0
g++-11: g++-11 (Ubuntu 11.4.0-1ubuntu1~22.04.3) 11.4.0
cmake: cmake version 3.22.1
make: GNU Make 4.3
```

这说明 Docker 环境已经对齐到本轮需要验证的 Ubuntu 22.04 + g++11 构建基线。

完整环境日志：

```text
logs/01_container_env.log
```

## 6. CMake configure 结果

CMake configure 结果：成功。

关键日志：

```text
-- The C compiler identification is GNU 11.4.0
-- The CXX compiler identification is GNU 11.4.0
x86_64
linux x64架构
pinocchio_lin_x64urdfdom_model_lin_x64tinyxml_lin_x64console_bridge_lin_x64jsoncpp_lin_x64quill_lin_x64
-- Configuring done
-- Generating done
-- Build files have been written to: /run_root/worktree/OpenLoong-Dyn-Control/build
```

完整 configure 日志：

```text
logs/02_cmake_configure.log
```

## 7. Build 结果

Build 结果：失败。

构建过程已经成功编译并链接了静态库：

```text
build/libcore.a 3343602 bytes
```

随后在链接 demo 可执行文件时失败。关键错误如下：

```text
/usr/bin/ld:/run_root/worktree/OpenLoong-Dyn-Control/third_party/mujoco/lin_x64/libmujoco.so: file format not recognized; treating as linker script
/usr/bin/ld:/run_root/worktree/OpenLoong-Dyn-Control/third_party/mujoco/lin_x64/libmujoco.so:0: syntax error
collect2: error: ld returned 1 exit status
```

受影响目标包括：

```text
float_control
wbc_speed_test
walk_wbc
walk_mpc_wbc
walk_wbc_joystick
walk_wbc_staircase
walk_mpc_wbc_joystick
jump_mpc
```

完整 build 日志：

```text
logs/03_cmake_build.log
```

## 8. 可执行文件检查

因为 build 在链接阶段失败，容器脚本中后续自动可执行文件检查没有进入执行阶段。

本轮人工记录的目标检查结果：

```text
walk_wbc build/walk_wbc NOT FOUND
walk_mpc_wbc build/walk_mpc_wbc NOT FOUND
wbc_speed_test build/wbc_speed_test NOT FOUND
jump_mpc build/jump_mpc NOT FOUND
float_control build/float_control NOT FOUND
```

检查记录文件：

```text
build_artifacts/build_targets_check.txt
```

## 9. 当前结论

本轮结论分三层：

1. Docker 环境可用，镜像构建成功。
2. Ubuntu 22.04 + `gcc-11/g++-11` + CMake configure 成功。
3. `cmake --build` 失败，失败点不是编译器版本，而是 MuJoCo 动态库链接文件形态异常。

失败根因证据：

```text
file external/open_source_repos/OpenLoong-Dyn-Control/third_party/mujoco/lin_x64/libmujoco.so
=> ASCII text, with no line terminators

sed -n '1,20p' external/open_source_repos/OpenLoong-Dyn-Control/third_party/mujoco/lin_x64/libmujoco.so
=> libmujoco.so.3.1.1

file external/open_source_repos/OpenLoong-Dyn-Control/third_party/mujoco/lin_x64/libmujoco.so.3.1.1
=> ELF 64-bit LSB shared object, x86-64, version 1 (SYSV), dynamically linked, not stripped

git -C external/open_source_repos/OpenLoong-Dyn-Control ls-files -s third_party/mujoco/lin_x64/libmujoco.so third_party/mujoco/lin_x64/libmujoco.so.3.1.1
=> 120000 ... third_party/mujoco/lin_x64/libmujoco.so
=> 100755 ... third_party/mujoco/lin_x64/libmujoco.so.3.1.1

git -C external/open_source_repos/OpenLoong-Dyn-Control config --get core.symlinks
=> false
```

解释：

- Git 索引中的 `120000` 表示 `libmujoco.so` 原本应是符号链接。
- 当前工作树中的 `libmujoco.so` 是 18 字节普通文本文件，内容是 `libmujoco.so.3.1.1`。
- 链接器尝试把这个文本文件当作动态库读取，因此报 `file format not recognized`。
- 真实目标文件 `libmujoco.so.3.1.1` 存在，并且是 x86-64 ELF 动态库。

因此，本轮 build 失败的直接原因是符号链接没有以 symlink 形态落盘，而不是 Ubuntu 22.04 容器无法构建，也不是 g++11 不兼容。

## 10. 下一步建议

建议下一轮 R2 做一个很小的、可审计的验证：

1. 不修改官方源码内容，只在隔离 worktree 中把 `third_party/mujoco/lin_x64/libmujoco.so` 恢复为指向 `libmujoco.so.3.1.1` 的符号链接。
2. 在同一个 Docker 镜像中重新运行 CMake build。
3. 若 build 通过，再只运行非 viewer 的 smoke candidate，例如 `wbc_speed_test`，并明确限制运行时间。
4. GUI viewer / MuJoCo 可视化需要单独设计 X11 或 GPU 转发方案，不应和 build-only 复现混在同一轮。

R2 的风险点：

- 如果官方源码目录本身由 `core.symlinks=false` checkout 得到，重新复制时仍会把 symlink 变成普通文本文件。
- 若直接在官方源码目录修 symlink，会改变官方工作树状态；更合适的做法是在输出 worktree 或新隔离副本中验证。
- GUI viewer 还涉及 DISPLAY、X11 socket、OpenGL/GPU 权限，和本轮链接问题不是同一个层面的故障。

## 11. 边界确认

本轮确认没有执行以下操作：

- 没有运行 `walk_wbc`、`walk_mpc_wbc` 或其他 MuJoCo GUI demo。
- 没有运行长时间 MuJoCo 仿真。
- 没有生成 MP4。
- 没有执行 `git add`、`git commit`、`git push`。
- 没有修改 `/var/run/docker.sock`。
- 没有使用 `chmod 666 /var/run/docker.sock`。
- 没有对官方源码做补丁修复。

需要注意：官方源码目录在检查时不是 clean 工作树，并且 `core.symlinks=false`。本轮没有把构建输出写入官方源码目录；实际 CMake/build 发生在 `outputs/docker_reproduction/.../worktree/OpenLoong-Dyn-Control/`。
