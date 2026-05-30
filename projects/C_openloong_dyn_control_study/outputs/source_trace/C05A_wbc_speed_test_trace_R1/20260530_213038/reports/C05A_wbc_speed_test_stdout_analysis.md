# C05A wbc_speed_test stdout Analysis

## 1. 输入

stdout 输入文件：

```text
projects/C_openloong_dyn_control_study/outputs/docker_reproduction/openloong_wbc_speed_test_runtime_fix_R2/20260530_184509/logs/02_wbc_speed_test_stdout.log
```

对应源码：

```text
demo/walk_wbc_speed_test.cpp
```

本轮只读取 C04 已生成日志，没有重新运行 `wbc_speed_test`。

## 2. 行数与 LoopNum 对齐关系

运行证据：

```text
stdout lines: 10001
```

源码中主循环为：

```cpp
const int LoopNum=10000;
for (int LoopCount = 0; LoopCount < LoopNum; LoopCount++) {
    ...
    printf("Execution time: %.6f sec. \n", duration.count() );
}
std::cout<<"loop time recorded to the last column of record/datalog.log"<<std::endl;
```

因此 stdout 的结构是：

```text
10000 行 Execution time: <秒> sec.
1 行 final hint: loop time recorded to the last column of record/datalog.log
```

这和 `LoopNum=10000` 精确对齐。

## 3. 每行含义

每个循环中的 stdout 行来自：

```text
demo/walk_wbc_speed_test.cpp:225
```

其值来自：

```text
demo/walk_wbc_speed_test.cpp:203-205
```

即：

```cpp
end = std::chrono::high_resolution_clock::now();
std::chrono::duration<double> duration = end - start;
start = end;
```

含义：当前循环计时窗口的 wall-clock duration，单位为秒。

注意一个重要细节：`start = end` 发生在 DataLogger 写入和 `printf` 之前。因此第 2 轮及后续循环的 duration 会包含上一轮的日志写入与 stdout 打印开销。它不是纯 WBC/QP 求解耗时。

## 4. 是否包含 cost、QP status 或 WBC 结果

stdout 不包含：

- QP cost。
- `qpStatus`。
- `nWSR`。
- `qp_cpuTime`。
- WBC 输出 `wbc_tauJointRes`。
- WBC 输出 `wbc_FrRes`。
- PVT 输出力矩。

源码中 `WBC_priority::computeTau()` 会写入：

```text
qpStatus
nWSR
cpu_time
```

并通过 `WBC_priority::dataBusWrite()` 写到 `RobotState`，但 `wbc_speed_test` 没有把这些字段打印到 stdout，也没有写入 datalog。

## 5. 能否作为性能 benchmark 数据

可以，但要限定解释范围。

适合用来表示：

- 这个程序整体循环的 wall-clock speed。
- 包含 Pinocchio kinematics/dynamics、gait/foot placement、WBC、PVT、DataLogger/printf 影响的运行耗时。
- `wbc_speed_test` 在当前 Docker/R2 运行链路下是否稳定完成 10000 次循环。

不适合直接表示：

- 纯 `WBC_priority::computeTau()` QP 求解耗时。
- 纯 `PriorityTasks::computeAll()` 耗时。
- 纯 Pinocchio CRBA/Jacobian 耗时。
- 实际 MuJoCo 闭环仿真帧耗时。

原因是源码只在循环较后位置做一次整体 duration，且计时窗口跨过上一轮日志/打印开销。

## 6. datalog 对齐

`datalog.log` 最后一列 `runTime` 来自同一个 `duration.count()`：

```text
demo/walk_wbc_speed_test.cpp:218
logger.recItermData("runTime", duration.count());
```

因此 stdout 的 10000 个 `Execution time` 数值应和 datalog 第 84 列逐行对应。stdout 额外多出的第 10001 行是循环后的说明文字，不对应 datalog 行。

## 7. 已计算统计

从 datalog 第 84 列读取的统计值：

```text
count      10000
min_sec    0.000480524
mean_sec   0.0005172386373
median_sec 0.0005050585
p95_sec    0.000560175
max_sec    0.003082548
first_sec  0.003082548
last_sec   0.000495746
```

前几轮明显更慢，后续稳定在约 0.5 ms 量级。这个数值可以作为 non-viewer speed smoke 的粗略性能记录。

## 8. 不确定点

- `duration` 统计是否覆盖 quill 异步写入的真实落盘成本：`NEEDS_RUNTIME_CONFIRMATION`。
- stdout 的 wall-clock 耗时是否能跨平台比较：`LOW`，因为它受容器、宿主机负载、stdout 重定向和文件系统影响。
- 若要得到纯 WBC/QP 时间，需要在源码中对 `computeDdq()`、`computeTau()`、`calMotorsPVT()` 分段计时，当前 C05A 不修改源码。
