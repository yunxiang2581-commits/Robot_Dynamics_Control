# C04 FIX Runtime wbc_speed_test Report

## 1. 本轮目标

C04 首次运行 `wbc_speed_test` 失败于动态库加载阶段：

```text
libqpOASES.so.3.2: cannot open shared object file: No such file or directory
```

本轮目标是：

- 不改源码。
- 不改 rpath。
- 不改 R2 worktree。
- 将 R2 worktree 以只读方式挂载到 `/run_root`，匹配 build-time RUNPATH。
- 只重跑 `wbc_speed_test`。
- 使用 `timeout 20s` 防止阻塞。

## 2. 上一轮失败摘要

C04 输出目录：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_smoke_R1/20260530_183311/
```

上一轮状态：

```text
exit_code=127
```

第一条真实错误：

```text
./wbc_speed_test: error while loading shared libraries: libqpOASES.so.3.2: cannot open shared object file: No such file or directory
```

原因判断：

- `wbc_speed_test` 的 RUNPATH 指向 `/run_root/worktree/OpenLoong-Dyn-Control/...`。
- C04 首次运行时把 R2 挂载到 `/r2_root`。
- 动态加载器按 RUNPATH 查找 `/run_root/...`，因此找不到 `libqpOASES.so.3.2`。

## 3. 输入

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

本轮挂载方式：

```text
R2_ROOT:/run_root:ro
RUN_ROOT:/c04_fix_root
```

其中 R2 worktree 只读挂载，C04-FIX 输出目录可写，用于保存日志。

## 4. 动态库预检查

`ldd` 结果显示动态库路径问题已经修复：

```text
libqpOASES.so.3.2 => /run_root/worktree/OpenLoong-Dyn-Control/third_party/qpOASES/lin_x64/libqpOASES.so.3.2
```

`readelf` 显示 `wbc_speed_test` 的 RUNPATH：

```text
/run_root/worktree/OpenLoong-Dyn-Control/third_party/eigen3
/run_root/worktree/OpenLoong-Dyn-Control/third_party/glfw
/run_root/worktree/OpenLoong-Dyn-Control/third_party/pinocchio
/run_root/worktree/OpenLoong-Dyn-Control/third_party/jsoncpp
/run_root/worktree/OpenLoong-Dyn-Control/third_party/quill
/run_root/worktree/OpenLoong-Dyn-Control/third_party/urdfdom
/run_root/worktree/OpenLoong-Dyn-Control/third_party/mujoco/lin_x64
/run_root/worktree/OpenLoong-Dyn-Control/third_party/qpOASES/lin_x64
```

预检查日志：

```text
logs/01_wbc_speed_test_precheck.log
```

## 5. 运行命令

宿主机 Docker 命令：

```bash
docker run --rm \
  -v "$R2_ROOT:/run_root:ro" \
  -v "$RUN_ROOT:/c04_fix_root" \
  openloong-ubuntu22-build:local \
  bash /c04_fix_root/commands/container_run_wbc_speed_test_rpath_fix.sh
```

容器内核心命令：

```bash
cd /run_root/worktree/OpenLoong-Dyn-Control/build
ldd ./wbc_speed_test || true
readelf -d ./wbc_speed_test | grep -E "RPATH|RUNPATH" || true
timeout 20s ./wbc_speed_test \
  > /c04_fix_root/logs/02_wbc_speed_test_stdout.log \
  2> /c04_fix_root/logs/03_wbc_speed_test_stderr.log
```

命令记录：

```text
commands/C04_fix_runtime_commands.sh
commands/container_run_wbc_speed_test_rpath_fix.sh
commands/container_run_wbc_speed_test_ld_library_path.sh
```

## 6. 运行结果

结果：FAIL。

退出码：

```text
134
```

第一条真实错误：

```text
terminate called after throwing an instance of 'quill::QuillError'
  what():  fopen failed with error message errno: "30"
```

`timeout` 还记录到程序 core dump：

```text
timeout: the monitored command dumped core
```

stdout：空。

stderr：包含上述 `quill::QuillError`。

是否进入主逻辑：没有看到 benchmark loop 输出。程序在初始化 `DataLogger` 相关写文件阶段退出。

运行日志：

```text
logs/02_wbc_speed_test_stdout.log
logs/03_wbc_speed_test_stderr.log
logs/04_wbc_speed_test_summary.log
logs/09_runtime_fopen_readonly_diagnostic.log
artifacts/wbc_speed_test_exit_code.txt
```

## 7. 新失败原因判断

本轮已经修复上一轮的动态库搜索路径问题。新的失败不是 `libqpOASES.so.3.2 not found`。

新的失败来自只读挂载与程序日志写入冲突：

```cpp
DataLogger logger("../record/datalog.log");
```

运行目录是：

```text
/run_root/worktree/OpenLoong-Dyn-Control/build
```

因此程序会写：

```text
/run_root/worktree/OpenLoong-Dyn-Control/record/datalog.log
```

但 `/run_root` 是按本轮边界要求只读挂载的：

```text
R2_ROOT:/run_root:ro
```

`errno: 30` 对应只读文件系统写入失败。因此程序在打开 datalog 文件时抛出 `quill::QuillError` 并 abort。

## 8. LD_LIBRARY_PATH 对照实验

未执行。

原因：本轮 `/run_root` 路径匹配已经让 `ldd` 找到 `libqpOASES.so.3.2`。当前失败不再是 shared library not found，而是只读挂载导致的日志写入失败。因此按任务要求，不执行 `LD_LIBRARY_PATH` 对照实验。

## 9. 当前结论

`/run_root` 只读挂载方案成功解决了 C04 首次运行的 rpath mismatch / shared library not found 问题。

`wbc_speed_test` runtime smoke 仍未通过。新的阻塞点是 `DataLogger("../record/datalog.log")` 需要写入 R2 worktree 的 `record/` 目录，但本轮必须只读挂载 R2 worktree。

因此当前状态是：

```text
dynamic library lookup: PASS
wbc_speed_test execution: FAIL
failure class: readonly record/datalog.log write
exit code: 134
```

## 10. 下一步建议

推荐 `C04-FIX-RUNTIME-R2`，只围绕新的第一条真实错误处理，不改官方源码：

1. 继续保持 R2 worktree 只读挂载到 `/run_root`。
2. 在 C04-FIX 输出目录创建一个可写 `record/` 目录。
3. 用额外 bind mount 覆盖容器内路径：

```text
RUN_ROOT/runtime_record:/run_root/worktree/OpenLoong-Dyn-Control/record
```

4. 再次只运行 `wbc_speed_test`，继续使用 `timeout 20s`。
5. 仍然不运行 `walk_wbc` / `walk_mpc_wbc`。

这个方案不需要改源码，也不需要改 R2 worktree，只是给程序期望的 `record/` 写入位置提供一个可写 overlay。

## 11. 边界确认

本轮确认：

- 未修改 `external/open_source_repos/OpenLoong-Dyn-Control/` 官方源码。
- 未修改 R2 worktree 源码。
- R2 worktree 使用只读挂载。
- 未运行 `walk_wbc`。
- 未运行 `walk_mpc_wbc`。
- 未运行长时间 MuJoCo。
- 未打开 GUI viewer。
- 未生成 MP4。
- 未执行 `git add`。
- 未执行 `git commit`。
- 未执行 `git push`。
