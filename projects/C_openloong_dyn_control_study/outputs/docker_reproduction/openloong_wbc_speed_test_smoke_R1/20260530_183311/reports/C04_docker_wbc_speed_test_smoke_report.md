# C04 Docker wbc_speed_test Smoke Report

## 1. 本轮目标

本轮基于 C03 R2 build-only 成功产物，先审查 `wbc_speed_test` 是否属于 non-viewer 程序，再在 Docker 中短时运行并保存日志。

本轮明确不做：

- 不运行 `walk_wbc`。
- 不运行 `walk_mpc_wbc`。
- 不打开 MuJoCo GUI viewer。
- 不生成 MP4。
- 不修改官方源码和 R2 worktree 源码。

## 2. 输入

R2 输出路径：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/
```

目标可执行文件：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_ubuntu22_build_R2/20260530_180827/worktree/OpenLoong-Dyn-Control/build/wbc_speed_test
```

Docker image：

```text
openloong-ubuntu22-build:local
```

运行时挂载策略：

```text
R2_ROOT:/r2_root:ro
C04_RUN_ROOT:/c04_root
```

其中 R2 worktree 使用只读挂载，C04 输出目录用于保存日志。

## 3. 源码审查结果

源码位置：

```text
demo/walk_wbc_speed_test.cpp
```

只读审查结论：

- 未发现 `GLFW` / `glfwCreateWindow` / `viewer` / `window` 创建逻辑。
- 未发现 `mj_loadXML` 或 MuJoCo XML 加载逻辑。
- 未发现无限 `while` 主循环。
- 主循环是有限循环：`const int LoopNum=10000`。
- 主要逻辑是构造 `Pin_KinDyn`、`WBC_priority`、`GaitScheduler`、`FootPlacement`、`PVT_Ctr`，然后进行 WBC 数值计算并记录循环耗时。
- 程序会读取模型与配置：`../models/AzureLoong.urdf`、`../common/joint_ctrl_config.json`。
- 程序会尝试写日志：`../record/datalog.log`。

因此，`wbc_speed_test` 适合用 `timeout 20s` 做短时 smoke run。主要风险不是 viewer，而是运行时路径、动态库搜索路径、模型/配置路径或只读挂载下的日志写入问题。

源码审查日志：

```text
logs/00_wbc_speed_test_source_candidates.log
logs/01_wbc_speed_test_source_audit.log
```

## 4. 运行命令

宿主机 Docker 命令：

```bash
docker run --rm \
  -v "$R2_ROOT:/r2_root:ro" \
  -v "$C04_RUN_ROOT:/c04_root" \
  openloong-ubuntu22-build:local \
  bash /c04_root/commands/container_run_wbc_speed_test.sh
```

容器内核心命令：

```bash
cd /r2_root/worktree/OpenLoong-Dyn-Control/build
ldd ./wbc_speed_test || true
timeout 20s ./wbc_speed_test \
  > /c04_root/logs/03_wbc_speed_test_stdout.log \
  2> /c04_root/logs/04_wbc_speed_test_stderr.log
```

命令记录：

```text
commands/C04_wbc_speed_test_smoke_commands.sh
commands/container_run_wbc_speed_test.sh
```

## 5. 运行结果

结果：FAIL。

退出码：

```text
127
```

第一条真实错误：

```text
./wbc_speed_test: error while loading shared libraries: libqpOASES.so.3.2: cannot open shared object file: No such file or directory
```

`ldd` 预检查也显示：

```text
libqpOASES.so.3.2 => not found
```

stdout：空。

stderr：包含上述动态库加载错误。

日志路径：

```text
logs/02_wbc_speed_test_precheck.log
logs/03_wbc_speed_test_stdout.log
logs/04_wbc_speed_test_stderr.log
logs/05_wbc_speed_test_summary.log
logs/06_runtime_dynamic_library_diagnostic.log
artifacts/wbc_speed_test_exit_code.txt
```

## 6. 失败原因判断

本轮失败不是 viewer、GPU、OpenGL 或程序长时间运行问题。程序没有进入 WBC 主循环，而是在动态库加载阶段退出。

诊断证据：

```text
R2 worktree 中存在库文件：
third_party/qpOASES/lin_x64/libqpOASES.so -> libqpOASES.so.3.2
third_party/qpOASES/lin_x64/libqpOASES.so.3.2

运行时 ldd：
libqpOASES.so.3.2 => not found
```

关键原因：

```text
wbc_speed_test 的 rpath 是构建时路径：
/run_root/worktree/OpenLoong-Dyn-Control/third_party/qpOASES/lin_x64

本轮运行时 R2 只读挂载路径是：
/r2_root/worktree/OpenLoong-Dyn-Control/third_party/qpOASES/lin_x64
```

因此动态加载器按旧 rpath 查找 `/run_root/...`，但容器运行时实际挂载在 `/r2_root/...`，导致 `libqpOASES.so.3.2` 找不到。

## 7. 当前结论

`wbc_speed_test` 源码层面确认是 non-viewer：没有打开窗口，没有启动 MuJoCo GUI，没有无限 viewer loop。

本轮 smoke run 已执行，但结果为 FAIL，失败点是运行时动态库搜索路径不匹配。

本轮没有得到 WBC benchmark 的主循环输出，因为程序在进入主逻辑前已由动态加载器退出。

## 8. 下一步建议

推荐 `C04-FIX-RUNTIME-R1`，只围绕第一条真实 runtime error 修复运行方式，不改官方源码：

1. 保持 R2 worktree 只读。
2. Docker 运行时把 R2_ROOT 挂载到 `/run_root`，匹配 build-time rpath。
3. 或者在容器命令中设置 `LD_LIBRARY_PATH` 指向 `/r2_root/worktree/OpenLoong-Dyn-Control/third_party/qpOASES/lin_x64` 等第三方库路径。
4. 仍然只运行 `wbc_speed_test`，继续使用 `timeout 20s`。
5. 不要直接进入 `walk_wbc` / `walk_mpc_wbc`。

当前更推荐第 2 种路径匹配法，因为它不改变可执行文件、不改源码，也更贴合 R2 build 时生成的 rpath。

## 9. 边界确认

本轮确认：

- 未修改 `external/open_source_repos/OpenLoong-Dyn-Control/` 官方源码。
- 未修改 R2 worktree 源码。
- R2 worktree 使用只读挂载。
- 未运行 `walk_wbc`。
- 未运行 `walk_mpc_wbc`。
- 未运行长时间 MuJoCo。
- 未打开 MuJoCo GUI viewer。
- 未生成 MP4。
- 未执行 `git add`。
- 未执行 `git commit`。
- 未执行 `git push`。
