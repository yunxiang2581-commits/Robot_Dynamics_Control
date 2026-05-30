"""B02-to-B03 adapter.

本模块负责把 B02 的二连杆 tracking 任务包装成 B03 solver 可消费的接口：
- B02 提供任务和仿真事实；
- B03 提供 solver 家族；
- adapter 负责把当前状态、目标 horizon、rollout cost 和 baseline controller 接起来。

真实调用顺序可以理解为：
1. 从 B02 MuJoCo model/data 读取当前状态，形成 `B02TrackingSnapshot`；
2. 把 snapshot、未来目标轨迹和 cost/solver 配置组装成 B03 的 `MPCProblem`；
3. B03 sampling solver 采样一批候选控制序列 `candidate_controls`；
4. solver 调用 `problem.rollout_cost_fn(candidate_controls, problem)`；
5. adapter 在内部复用 B02 `env.rollout()` 做预测，并按 B02 tracking cost 打分；
6. B03 solver 只根据 cost 排序，返回统一的 `MPCSolution`。

本文件只做接口适配，不改变 B02 的动力学模型，也不改变 B03 solver 的统一输入输出格式。
"""

# 语法说明：`from __future__ import annotations` 让类型标注延迟求值。
# 好处是可以在类型标注里写还没完全加载的类名，并减少运行时类型解析负担。
from __future__ import annotations

# 语法说明：`from ... import ...` 从标准库模块中导入指定名字。
from dataclasses import dataclass  # `dataclass` 用来自动生成数据容器类的初始化函数等方法。
import time  # `import 模块名` 导入整个模块；这里用 `time.perf_counter()` 统计 solver 耗时。
from typing import Any, Callable  # `Any` 表示任意类型；`Callable` 表示“可调用对象/函数”的类型。

import numpy as np  # `as np` 是别名，后面用 `np.asarray/np.zeros/np.linalg.norm` 等 NumPy API。

# 从 B03 统一 solver 接口模块导入基础类和数据结构，adapter 的输出必须符合这些接口。
from planners.mpc_solver_interface import BaseMPCSolver, MPCProblem, MPCSolution, SolverStats


# 语法说明：`@dataclass(...)` 是“装饰器”，写在 class 上一行。
# 它会让 Python 自动生成 `__init__`、`__repr__` 等常用方法。
# `frozen=True` 表示对象创建后字段不能再重新赋值，适合保存一帧不可变的快照。
@dataclass(frozen=True)
# 语法说明：`class 类名:` 用来定义一个新类型。
# 这里的类名是 `B02TrackingSnapshot`，表示“B02 tracking 快照”这种数据结构。
class B02TrackingSnapshot:
    """B02 当前 tracking 任务的一张状态快照。

    学习重点：
    - `q/qvel` 是 MuJoCo 关节空间状态；
    - `state=[q1,q2,dq1,dq2]` 是 B03 solver 接收的当前状态向量；
    - `target_xy/actual_xy/error_norm` 记录当前末端 tracking 误差，方便日志和复盘。
    """

    # 语法说明：`字段名: 类型` 是类型标注。
    # dataclass 会把这些标注过的名字当成构造函数参数。
    time: float  # `float` 表示浮点数，例如 0.01。
    q: np.ndarray  # `np.ndarray` 表示 NumPy 数组；这里保存关节角 [q1, q2]。
    qvel: np.ndarray  # 保存关节速度 [dq1, dq2]。
    state: np.ndarray  # 保存拼接后的状态 [q1, q2, dq1, dq2]。
    target_xy: np.ndarray  # 保存当前末端目标位置 [x_target, y_target]。
    actual_xy: np.ndarray  # 保存当前末端实际位置 [x_ee, y_ee]。
    error_norm: float  # 保存一个标量误差 ||actual_xy - target_xy||。


# 第二个 dataclass：语法结构和上面一样，但表达的是 rollout cost 的结果。
@dataclass(frozen=True)
class B02RolloutCostResult:
    """B02 rollout cost 的批量评估结果。

    `costs[i]` 对应第 i 条候选控制序列的 horizon cost。
    如果配置要求记录轨迹，则同时保存每条候选序列的状态预测和末端位置预测，
    方便 B03 logger / visualizer 后续统一消费。
    """

    costs: np.ndarray  # shape 通常是 (num_candidates,)，每个元素是一条候选序列的 cost。
    predicted_states: np.ndarray | None  # `A | None` 表示这个字段可以是 A，也可以是 None。
    predicted_ee_positions: np.ndarray | None  # 可选保存末端预测轨迹；不记录时为 None。
    candidate_controls: np.ndarray  # 保存被实际评估的候选控制序列。


# 第三个 dataclass：保存 adapter 配置。
# 配置对象也用 frozen=True，避免规划过程中参数被意外改掉。
@dataclass(frozen=True)
class B02AdapterConfig:
    """B02-to-B03 adapter 的最小配置集合。

    这里把两类参数放在一起：
    - rollout 结构参数：`horizon`、`dt`、`control_dim`、`torque_limit`；
    - cost 权重参数：末端误差、速度正则、力矩正则和 terminal error。
    """

    horizon: int  # `int` 表示整数；这里是 MPC 向未来看的步数 H。
    dt: float  # 仿真/控制步长。
    control_dim: int  # 控制向量维度；B02 二连杆通常是 2，对应 [tau1, tau2]。
    torque_limit: float  # 力矩限幅，用于裁剪候选 torque。
    ee_weight: float  # 末端位置误差权重。
    dq_weight: float  # 关节速度正则权重。
    torque_weight: float  # 控制力矩正则权重。
    terminal_weight: float  # horizon 最后一步 terminal error 权重。
    record_predicted_states: bool  # `bool` 表示 True/False；是否记录预测状态轨迹。
    record_predicted_ee_positions: bool  # 是否记录预测末端位置轨迹。


# 语法说明：`def 函数名(参数: 类型) -> 返回类型:` 定义一个函数。
# `_capture_env_state` 前面的下划线表示“模块内部辅助函数”，外部通常不直接调用。
# `env: Any` 表示 env 可以是任意对象；`dict[str, Any] | None` 表示返回字典或 None。
def _capture_env_state(env: Any) -> dict[str, Any] | None:
    """保存真实 MuJoCo 状态，保证 rollout 不污染闭环环境。

    为什么需要：
    MPC 的 rollout 是“从当前状态出发假想未来”，不是实际推进真实闭环环境。
    所以在批量评估候选控制序列之前，必须先保存真实 `qpos/qvel/ctrl/time`。
    """
    data = getattr(env, "data", None)  # `getattr(obj, name, default)` 安全读取属性；没有 `data` 时返回 None。
    if data is None:  # `if` 条件判断；没有 MuJoCo data 就无法保存状态。
        return None  # `return` 立刻结束函数，并把 None 交给调用方。

    return {  # 返回一个字典；key 是字符串，value 是后续恢复状态需要的数据。
        "qpos": np.array(data.qpos, copy=True),  # `copy=True` 强制复制，避免后续 rollout 改到同一块内存。
        "qvel": np.array(data.qvel, copy=True),  # 保存关节速度数组。
        "ctrl": np.array(data.ctrl, copy=True),  # 保存当前 actuator 控制输入。
        "time": float(getattr(data, "time", 0.0)),  # 读取仿真时间；`float(...)` 保证保存成普通浮点数。
        "last_applied_torque": getattr(env, "last_applied_torque", None),  # 保存 B02 env 自己记录的上一条力矩。
    }  # 字典字面量结束。


# 语法说明：返回类型 `-> None` 表示这个函数只执行副作用，不返回业务结果。
def _restore_env_state(env: Any, saved_state: dict[str, Any] | None) -> None:
    """恢复真实 MuJoCo 状态。

    输入是 `_capture_env_state()` 返回的字典。
    恢复后调用 `mj_forward`，让 site 位置、body 位姿等 MuJoCo 派生量重新对齐。
    """
    if saved_state is None:  # 如果保存阶段没有拿到状态，这里也没有东西可恢复。
        return  # 空返回；调用方只需要它安全结束。

    data = getattr(env, "data", None)  # 再次安全读取 env.data，避免 env 不是完整 MuJoCo 环境。
    model = getattr(env, "model", None)  # 安全读取 env.model，后面有 model 才能调用 mj_forward。
    if data is None:  # 没有 data 就无法写回 qpos/qvel/ctrl。
        return  # 直接结束，避免 AttributeError。

    data.qpos[:] = saved_state["qpos"]  # `[:]` 表示原地写入数组内容，不替换 MuJoCo 持有的数组对象。
    data.qvel[:] = saved_state["qvel"]  # 原地恢复速度。
    data.ctrl[:] = saved_state["ctrl"]  # 原地恢复控制输入。
    if hasattr(data, "time"):  # `hasattr` 检查对象是否有某个属性。
        data.time = saved_state["time"]  # MuJoCo data 有 time 字段时恢复仿真时间。
    if saved_state["last_applied_torque"] is not None:  # 只有保存过 B02 自定义字段时才恢复。
        env.last_applied_torque = saved_state["last_applied_torque"]  # 恢复 env 自己记录的上一条真实力矩。

    if model is not None:  # 有 model 才能调用 MuJoCo forward 更新派生量。
        try:  # `try/except` 用来保护可选依赖或底层调用失败，不让恢复逻辑中断主流程。
            import mujoco  # 在函数内部导入，避免模块加载时强依赖 mujoco。

            mujoco.mj_forward(model, data)  # 根据恢复后的 qpos/qvel 重新计算 site/body 等派生状态。
        except Exception:  # 捕获所有异常；这里是清理恢复阶段，失败也不继续抛出。
            pass  # `pass` 是空语句，表示这里故意什么都不做。


# 这个函数返回 `int`，也就是 MuJoCo site 的整数编号。
def _resolve_site_id(model: Any, site_name: str | None, ee_site_id: int | None) -> int:
    """解析末端 site id。

    B02 的 tracking cost 是 task-space cost，需要读取末端点 `p_ee=(x,y)`。
    如果外部已经传入 `ee_site_id`，直接使用；否则根据 `site_name` 查询 MuJoCo 名称表。
    """

    if ee_site_id is not None:  # 如果调用方已经给了 site id，就不需要再按名字查表。
        return int(ee_site_id)  # `int(...)` 保证返回 Python 整数。
    if site_name is None:  # 如果既没有 id 也没有名字，就无法确定末端 site。
        raise ValueError("Either site_name or ee_site_id must be provided.")  # `raise` 主动抛出清晰错误。

    try:  # 下面需要 MuJoCo 的名称查询 API，所以这里尝试导入。
        import mujoco  # 导入 MuJoCo Python 包。
    except ImportError as exc:  # `as exc` 把原始导入错误保存到变量里。
        raise RuntimeError("extract_b02_tracking_snapshot needs mujoco to resolve site_name.") from exc  # `from exc` 保留异常链。

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)  # 按 site 名字查询 MuJoCo 内部 id。
    if site_id < 0:  # MuJoCo 查不到名字时通常返回负数。
        available_names = [  # 列表推导式：遍历所有 site id，收集可用名字，方便报错定位。
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)  # 把第 i 个 site id 转回名字。
            for i in range(getattr(model, "nsite", 0))  # `range(nsite)` 遍历 site 编号；没有 nsite 时默认 0。
        ]  # 列表推导结束。
        raise ValueError(f"Cannot find site {site_name!r}. Available sites: {available_names}")  # f-string 插入变量值。
    return int(site_id)  # 查询成功，把 MuJoCo 返回值转成普通 int。


# 输入 env 和状态数组，输出末端位置数组。
def _compute_ee_positions_for_states(env: Any, states: np.ndarray) -> np.ndarray:
    """从状态序列恢复每个时刻的末端位置。

    输入：
    - `states.shape == (T, state_dim)`，通常 state_dim=4。

    输出：
    - `ee_positions.shape == (T, 2)`，每一行是对应状态下的末端 xy。

    这个函数主要给 baseline wrapper 使用：当旧 B02 controller 只记录状态轨迹时，
    adapter 可以临时 set_state，再用 B02 env 查询末端位置轨迹。
    """
    saved_state = _capture_env_state(env)  # 保存真实环境，防止下面 set_state 改坏闭环状态。
    ee_positions = np.zeros((states.shape[0], 2), dtype=float)  # 预分配输出数组；行数等于状态数量，列是 x/y。
    try:  # 用 try/finally 保证即使中途出错，也会恢复环境。
        for index, state in enumerate(states):  # `enumerate` 同时给出序号 index 和当前 state。
            env.set_state(tuple(float(value) for value in state.tolist()))  # 生成器表达式逐个转 float，再打包成 tuple。
            ee_positions[index] = np.asarray(env.get_end_effector_position(), dtype=float)  # 查询末端 xy 并写到对应行。
    finally:  # finally 块无论 try 成功还是异常都会执行。
        _restore_env_state(env, saved_state)  # 恢复进入函数前的真实 env 状态。
    return ee_positions  # 返回所有状态对应的末端位置轨迹。


# 多行函数签名语法：参数太多时可以分行写，最后用 `) -> 返回类型:` 收尾。
def extract_b02_tracking_snapshot(
    model: Any,  # `model` 是 MuJoCo MjModel；这里用 Any 是为了让测试 fake object 也能传入。
    data: Any,  # `data` 是 MuJoCo MjData；函数只读取其中的 qpos/qvel/site_xpos。
    target_xy: np.ndarray | list[float] | tuple[float, float],  # `|` 是联合类型：允许数组、列表或二元 tuple。
    site_name: str | None = None,  # `= None` 是默认值；调用方可以不传 site_name。
    ee_site_id: int | None = None,  # 也可以直接传整数 site id，避免按名字查表。
    time_value: float | None = None,  # 可选覆盖时间；不传则读取 data.time。
) -> B02TrackingSnapshot:
    """从 B02 当前 model/data 提取 tracking snapshot。

    这个函数只读取当前事实，不修改 MuJoCo 状态。

    输入：
    - `model/data`：B02 当前 MuJoCo 模型和数据；
    - `target_xy`：当前时刻的末端目标；
    - `site_name` 或 `ee_site_id`：告诉函数应该读取哪个末端 site。

    输出：
    - `B02TrackingSnapshot`，其中 `state=[q1,q2,dq1,dq2]` 会成为 B03 的 `current_state`。
    """
    target_xy_array = np.asarray(target_xy, dtype=float)  # 把 list/tuple/array 统一转成 float NumPy 数组。
    if target_xy_array.shape != (2,):  # B02 末端目标必须是二维平面点 [x, y]。
        raise ValueError(f"target_xy must have shape (2,), got {target_xy_array.shape}")  # shape 不对就立即报错。

    site_id = _resolve_site_id(model=model, site_name=site_name, ee_site_id=ee_site_id)  # 用关键字参数调用，增强可读性。
    if site_id >= getattr(model, "nsite", 0):  # 检查 site id 没有超过 MuJoCo site 数量。
        raise ValueError(f"site id out of range: {site_id}")  # id 越界说明调用方传错了 ee_site_id。

    q = np.asarray(data.qpos[:2], dtype=float).copy()  # 切片 `[:2]` 取前两个关节角；copy 避免引用 MuJoCo 内部数组。
    qvel = np.asarray(data.qvel[:2], dtype=float).copy()  # 取前两个关节速度。
    if q.shape != (2,) or qvel.shape != (2,):  # `or` 表示任一条件成立就报错。
        raise ValueError("B02 snapshot expects two-link q/qvel with shape (2,).")  # B02 当前假设是二连杆。

    actual_xy = np.asarray(data.site_xpos[site_id][:2], dtype=float).copy()  # 读取末端 site 世界坐标的 x/y。
    if actual_xy.shape != (2,):  # 正常情况下 site_xpos[site_id][:2] 应该一定是二维。
        raise RuntimeError(f"Failed to read site position for site_id={site_id}.")  # 如果失败，说明 MuJoCo 数据异常。

    state = np.concatenate([q, qvel]).astype(float, copy=False)  # `concatenate` 拼成 [q1,q2,dq1,dq2]。
    snapshot_time = float(time_value if time_value is not None else getattr(data, "time", 0.0))  # 条件表达式：优先用显式时间。
    error_norm = float(np.linalg.norm(actual_xy - target_xy_array))  # `np.linalg.norm` 计算欧氏距离误差。

    return B02TrackingSnapshot(  # 调用 dataclass 自动生成的构造函数，返回一张不可变快照。
        time=snapshot_time,  # 关键字参数：字段名和值一一对应。
        q=q,  # 当前关节角。
        qvel=qvel,  # 当前关节速度。
        state=state,  # B03 solver 使用的当前状态向量。
        target_xy=target_xy_array,  # 当前目标。
        actual_xy=actual_xy,  # 当前实际末端位置。
        error_norm=error_norm,  # 当前误差范数。
    )  # 构造函数调用结束。


# 这个函数把 B02 数据结构翻译成 B03 的统一问题对象。
def build_b03_problem_from_b02_snapshot(
    snapshot: B02TrackingSnapshot,  # 当前时刻 B02 快照。
    target_horizon: np.ndarray,  # 未来 H 步目标，期望 shape=(H,2)。
    adapter_config: B02AdapterConfig,  # adapter 配置，提供 horizon/dt/cost 权重等。
    solver_config: dict[str, Any] | None = None,  # 可选 solver 配置；不传则用默认空字典。
    rollout_cost_fn: Callable[[np.ndarray, MPCProblem], Any] | None = None,  # 可选 cost 回调，B03 sampling solver 会调用它。
) -> MPCProblem:
    """把 B02 snapshot 转成 B03 可消费的 `MPCProblem`。

    这是当前状态和目标 horizon 进入 B03 solver 的关键入口。

    映射关系：
    - `snapshot.state` -> `problem.current_state`
    - `target_horizon` -> `problem.target_horizon`
    - `adapter_config` -> horizon、dt、control_dim、cost 权重
    - `rollout_cost_fn` -> B03 solver 后续评估候选控制序列时调用的函数
    """
    target_horizon_array = np.asarray(target_horizon, dtype=float)  # 统一转成 float NumPy 数组，便于后续 shape 检查和 cost 计算。
    expected_shape = (adapter_config.horizon, 2)  # Python tuple，表示期望二维形状：(H, xy)。
    if target_horizon_array.shape != expected_shape:  # 目标 horizon 必须和配置中的 H 一致。
        raise ValueError(f"target_horizon must have shape {expected_shape}, got {target_horizon_array.shape}")  # 提前报错避免 solver 内部难查。

    solver_config_dict = dict(solver_config or {})  # `or {}` 处理 None；`dict(...)` 复制一份，避免修改外部原字典。
    # B03 solver 统一从 `problem.solver_config` 读取采样/约束相关参数。
    # 外部没有显式给出时，这里用 adapter_config 补齐 B02 二连杆的默认约定。
    solver_config_dict.setdefault("horizon", adapter_config.horizon)  # `setdefault` 只在 key 不存在时写入默认值。
    solver_config_dict.setdefault("control_dim", adapter_config.control_dim)  # 确保 B03 solver 能读到控制维度。
    solver_config_dict.setdefault("torque_limit", adapter_config.torque_limit)  # 确保 B03 solver 能读到力矩限制。

    return MPCProblem(  # 构造 B03 统一问题对象；后续 solver.solve(problem) 只认这个格式。
        current_state=np.asarray(snapshot.state, dtype=float),  # 当前状态 x_k。
        target_horizon=target_horizon_array,  # 未来 H 步末端目标。
        horizon=adapter_config.horizon,  # 预测步数。
        control_dim=adapter_config.control_dim,  # 控制维度。
        dt=float(adapter_config.dt),  # 控制/仿真步长，转成普通 float。
        cost_config={  # cost_config 是字典，集中保存代价函数权重。
            "ee_weight": float(adapter_config.ee_weight),  # 末端误差权重。
            "dq_weight": float(adapter_config.dq_weight),  # 速度正则权重。
            "torque_weight": float(adapter_config.torque_weight),  # 力矩正则权重。
            "terminal_weight": float(adapter_config.terminal_weight),  # 终端误差权重。
        },
        solver_config=solver_config_dict,  # solver 自己需要的参数，例如 num_candidates/sampling_std 等也会放这里。
        rollout_cost_fn=rollout_cost_fn,  # B03 sampling solver 通过这个回调评估候选控制序列。
        metadata={  # metadata 不参与控制计算，主要给日志、调试、可视化使用。
            "snapshot_time": float(snapshot.time),  # 快照时间。
            "snapshot_error_norm": float(snapshot.error_norm),  # 当前 tracking 误差。
            "snapshot_target_xy": np.asarray(snapshot.target_xy, dtype=float),  # 当前目标位置。
            "snapshot_actual_xy": np.asarray(snapshot.actual_xy, dtype=float),  # 当前实际末端位置。
        },
    )  # MPCProblem 构造结束。


# 这个函数是 B02 动力学 rollout 和 B02 cost 真正接入 B03 solver 的地方。
def rollout_cost_candidates(
    candidate_controls: np.ndarray,  # B03 solver 采样出的候选控制序列，shape=(N,H,control_dim)。
    problem: MPCProblem,  # 当前 B03 问题，里面有 current_state 和 target_horizon。
    env: Any,  # B02 风格环境，需要提供 rollout/set_state/get_end_effector_position。
    adapter_config: B02AdapterConfig,  # adapter 配置，提供 H、控制维度、权重和记录开关。
) -> B02RolloutCostResult:
    """对 candidate control sequences 做真实 B02 two-link rollout 并计算 cost。

    当前实现直接复用 B02 `TwoLinkEnv.rollout()`、`set_state()` 与
    `get_end_effector_position()`，因此 solver 不需要知道 MuJoCo 细节。

    输入：
    - `candidate_controls.shape == (num_candidates, horizon, control_dim)`；
    - `problem.current_state` 是 rollout 起点；
    - `problem.target_horizon` 是未来 H 步末端目标；
    - `env` 必须提供 B02 风格的 `rollout/set_state/get_end_effector_position`。

    输出：
    - 每条候选序列的 cost；
    - 可选的预测状态轨迹和末端位置轨迹；
    - 被 torque limit 裁剪后的候选控制序列。
    """
    controls = np.asarray(candidate_controls, dtype=float)  # 统一转成 float 数组，避免 list 或整数数组影响计算。
    if controls.ndim != 3:  # `.ndim` 是数组维数；候选控制必须是三维批量数据。
        raise ValueError(f"candidate_controls must be 3D, got shape={controls.shape}")  # shape 错误直接拒绝。
    num_candidates, horizon, control_dim = controls.shape  # 元组拆包：把 shape 三个维度分别命名。
    if horizon != adapter_config.horizon or horizon != problem.horizon:  # horizon 必须在 controls/config/problem 三处一致。
        raise ValueError(  # 多行字符串拼接在括号内自动成立。
            "candidate_controls horizon does not match adapter/problem horizon: "
            f"{horizon}, {adapter_config.horizon}, {problem.horizon}"
        )
    if control_dim != adapter_config.control_dim or control_dim != problem.control_dim:  # 控制维度也必须一致。
        raise ValueError(
            "candidate_controls control_dim does not match adapter/problem control_dim: "
            f"{control_dim}, {adapter_config.control_dim}, {problem.control_dim}"
        )

    target_horizon = np.asarray(problem.target_horizon, dtype=float)  # 从问题对象中取出目标 horizon。
    if target_horizon.shape != (horizon, 2):  # B02 当前 task-space 目标固定是二维 xy。
        raise ValueError(f"problem.target_horizon must have shape {(horizon, 2)}, got {target_horizon.shape}")  # 目标长度不匹配就报错。

    if not hasattr(env, "rollout") or not hasattr(env, "set_state") or not hasattr(env, "get_end_effector_position"):  # 检查 env 接口是否够用。
        raise NotImplementedError(  # env 不满足 B02 风格接口时，当前 adapter 没法评估 rollout。
            "rollout_cost_candidates currently expects a B02-style env with rollout/set_state/get_end_effector_position."
        )

    # B03 solver 可以采样任意实数 torque；真正交给 B02 rollout 前要先按 actuator 安全范围裁剪。
    clipped_controls = np.clip(controls, -adapter_config.torque_limit, adapter_config.torque_limit)
    state_dim = int(np.asarray(problem.current_state, dtype=float).size)  # `.size` 是状态向量元素总数。
    if state_dim < adapter_config.control_dim * 2:  # 二连杆 cost 需要至少 q 和 dq 两段。
        raise ValueError(
            "current_state is too short to contain q and dq for two-link cost evaluation: "
            f"state_dim={state_dim}, control_dim={adapter_config.control_dim}"
        )

    predicted_states = (  # 条件表达式：根据配置决定是否预分配状态轨迹数组。
        np.zeros((num_candidates, horizon + 1, state_dim), dtype=float)  # 每条 rollout 有 H+1 个状态。
        if adapter_config.record_predicted_states  # True 时记录完整预测状态。
        else None  # False 时不记录，节省内存。
    )
    predicted_ee_positions = (  # 同样根据配置决定是否记录末端轨迹。
        np.zeros((num_candidates, horizon + 1, 2), dtype=float)  # 每个末端位置是 xy 两维。
        if adapter_config.record_predicted_ee_positions
        else None
    )
    costs = np.zeros((num_candidates,), dtype=float)  # 初始化每条候选序列的 cost，shape=(N,)。

    # 下面会反复调用 env.rollout / env.set_state。
    # 这些操作都是“假想未来”，所以必须在 finally 中恢复真实闭环状态。
    saved_env_state = _capture_env_state(env)
    initial_state = tuple(float(value) for value in np.asarray(problem.current_state, dtype=float).tolist())  # 转成 B02 env.rollout 需要的 tuple。

    try:  # 保护整个批量 rollout，确保 finally 恢复 env。
        for candidate_index in range(num_candidates):  # 遍历第 0 到 N-1 条候选控制序列。
            # 取出第 candidate_index 条未来控制序列：
            # [(tau1_0,tau2_0), ..., (tau1_H-1,tau2_H-1)]。
            torque_sequence = [
                tuple(float(value) for value in control)
                for control in clipped_controls[candidate_index]
            ]

            # B02 env.rollout 从同一个当前状态出发，预测执行这条 torque 序列后的未来状态。
            # 期望长度是 H+1：第 0 个状态是当前状态，后 H 个状态是每步控制后的状态。
            states_list = env.rollout(initial_state, torque_sequence)
            states = np.asarray(states_list, dtype=float)
            if states.shape != (horizon + 1, state_dim):  # rollout 必须返回当前状态 + H 个未来状态。
                raise ValueError(
                    "rollout returned unexpected state shape: "
                    f"{states.shape}, expected {(horizon + 1, state_dim)}"
                )

            # B02 的 cost 在末端 task-space 上定义，因此需要把每个预测状态转成末端 xy。
            ee_positions = np.zeros((horizon + 1, 2), dtype=float)
            for step_index, state in enumerate(states):  # 遍历这条 rollout 中的每个预测状态。
                env.set_state(tuple(float(value) for value in state.tolist()))  # 临时把 env 放到该预测状态。
                ee_positions[step_index] = np.asarray(env.get_end_effector_position(), dtype=float)  # 查询并记录末端 xy。

            if predicted_states is not None:  # 只有配置要求记录时才写入。
                predicted_states[candidate_index] = states  # 保存第 candidate_index 条状态轨迹。
            if predicted_ee_positions is not None:  # 同理，按需保存末端轨迹。
                predicted_ee_positions[candidate_index] = ee_positions  # 保存第 candidate_index 条末端轨迹。

            # `states[0]` / `ee_positions[0]` 是 rollout 起点，不对应任何已执行控制。
            # 因此 cost 从第 1 个预测点开始，与 `target_horizon[0:H]` 对齐。
            dq = states[1:, adapter_config.control_dim : adapter_config.control_dim * 2]
            ee_error = ee_positions[1:] - target_horizon
            torque_penalty = clipped_controls[candidate_index]  # 当前候选序列的全部 torque，用于控制正则。
            terminal_error = ee_positions[-1] - target_horizon[-1]  # `[-1]` 取最后一步，计算 terminal error。

            # B02 tracking cost:
            # 1. 末端位置误差让机械臂追目标；
            # 2. 速度正则抑制过快运动；
            # 3. 力矩正则抑制过大控制输入；
            # 4. terminal error 额外强调 horizon 末端要靠近目标。
            costs[candidate_index] = (
                float(adapter_config.ee_weight) * float(np.sum(ee_error**2))
                + float(adapter_config.dq_weight) * float(np.sum(dq**2))
                + float(adapter_config.torque_weight) * float(np.sum(torque_penalty**2))
                + float(adapter_config.terminal_weight) * float(np.sum(terminal_error**2))
            )
    finally:  # 无论 rollout/cost 是否报错，都恢复真实环境。
        _restore_env_state(env, saved_env_state)  # 恢复调用函数前的 MuJoCo 状态。

    return B02RolloutCostResult(  # 把批量评估结果打包成 dataclass 返回给 B03 solver。
        costs=costs,  # 每条候选序列的 cost。
        predicted_states=predicted_states,  # 可选预测状态轨迹。
        predicted_ee_positions=predicted_ee_positions,  # 可选末端轨迹。
        candidate_controls=clipped_controls,  # 实际评估过的、已经限幅的控制序列。
    )


# 这个函数返回另一个函数，所以返回类型是 `Callable[...]`。
def make_b02_rollout_cost_fn(
    env: Any,  # B02 环境对象，会被内部闭包记住。
    adapter_config: B02AdapterConfig,  # adapter 配置，也会被内部闭包记住。
    evaluator: Callable[[np.ndarray, MPCProblem, Any, B02AdapterConfig], Any] | None = None,  # 可选自定义评估函数。
    return_costs_only: bool = False,  # 默认返回完整 result；设为 True 时只返回 costs。
) -> Callable[[np.ndarray, MPCProblem], Any]:
    """生成供 B03 solver 调用的 rollout cost callable。

    兼容两种模式：
    - 返回 `B02RolloutCostResult`，供 solver 读取 cost 和预测轨迹；
    - 返回纯 `costs` 数组，兼容只关心排序的旧 solver。

    这层闭包的作用是把 B02 的 `env` 和 `adapter_config` 藏起来。
    B03 solver 只需要调用统一签名：
    `rollout_cost_fn(candidate_controls, problem)`。
    """

    # 嵌套函数语法：在函数内部再定义一个函数。
    # 这个内部函数会“闭包捕获”外层的 env、adapter_config、evaluator、return_costs_only。
    def rollout_cost_fn(candidate_controls: np.ndarray, problem: MPCProblem) -> Any:
        """B03 solver 实际调用的 cost 函数。

        solver 不知道内部是否用了 MuJoCo，也不知道 B02 env 的存在；
        它只提交候选控制序列，拿回 cost 或带轨迹的 cost result。
        """

        result = (  # 条件表达式：如果传了 evaluator，就优先用 evaluator；否则用默认 B02 rollout cost。
            evaluator(candidate_controls, problem, env, adapter_config)  # 自定义 evaluator 的调用签名。
            if evaluator is not None  # 判断 evaluator 是否存在。
            else rollout_cost_candidates(  # 默认路径：调用本文件中的真实 B02 rollout cost 评估函数。
                candidate_controls=candidate_controls,  # 候选控制序列。
                problem=problem,  # 当前 B03 问题。
                env=env,  # 外层闭包捕获的 B02 env。
                adapter_config=adapter_config,  # 外层闭包捕获的 adapter 配置。
            )
        )
        if return_costs_only and hasattr(result, "costs"):  # 同时满足“只要 costs”和“result 有 costs 属性”才进入。
            return np.asarray(result.costs, dtype=float)  # 把 dataclass result 中的 costs 拿出来，转成 float 数组。
        return result  # 默认返回完整 result，里面可能包含 cost 和预测轨迹。

    return rollout_cost_fn  # 返回内部函数对象；B03 problem 会保存并在 solver 内调用它。


# 类继承语法：`class 子类(父类):` 表示 B02BaselineSolver 继承 BaseMPCSolver。
class B02BaselineSolver(BaseMPCSolver):
    """把 B02 baseline controller 包装成 B03 solver 比较对象。

    注意：这个类不是新的优化算法。
    它只是让旧 B02 controller 也满足 B03 的 `BaseMPCSolver.solve(problem)` 接口，
    方便后续 benchmark / logger / visualizer 用同一种格式比较不同 solver。
    """

    solver_name = "b02_baseline"  # 类属性；所有实例默认共享这个 solver 名称。

    # `__init__` 是构造函数；创建 B02BaselineSolver(...) 时自动调用。
    # `self` 表示当前对象实例，方法里通过 self 保存和读取对象状态。
    def __init__(self, controller: Any, env: Any) -> None:
        """保存旧 B02 controller 和 env。

        后续 `solve()` 会把 B03 `MPCProblem` 转回 B02 controller 熟悉的
        `current_state + target_sequence` 形式。
        """

        self.controller = controller  # 给当前对象添加 controller 属性。
        self.env = env  # 给当前对象添加 env 属性。

    # 这是对 BaseMPCSolver.solve 的具体实现。
    def solve(
        self,  # 实例方法第一个参数必须是 self。
        problem: MPCProblem,  # B03 标准问题输入。
        previous_solution: MPCSolution | None = None,  # 可选上一轮解；这个 baseline 暂时不用。
    ) -> MPCSolution:
        """调用 B02 controller，但不修改它的原始接口。

        输入：
        - `problem.current_state`：当前二连杆状态；
        - `problem.target_horizon`：未来 H 步末端目标。

        输出：
        - B03 标准 `MPCSolution`，其中 `first_control` 是当前闭环真正要执行的 torque。
        """
        _ = previous_solution  # 用 `_` 接住未使用参数，表示“有意忽略”，避免读者误会漏写逻辑。
        start_time = time.perf_counter()  # 记录开始时间，用于计算 runtime_ms。

        # B02 controller 的旧接口使用 list[tuple[x,y]] 表示未来目标序列。
        target_sequence = [  # 列表推导式，把 NumPy 目标轨迹转成 B02 controller 旧接口需要的 list[tuple]。
            (float(target[0]), float(target[1]))  # 每个 target 是二维点，转成普通 Python float tuple。
            for target in np.asarray(problem.target_horizon, dtype=float)  # 遍历 horizon 内每个目标点。
        ]

        # B02 controller 的旧接口使用 tuple 表示当前状态。
        current_state = tuple(float(value) for value in np.asarray(problem.current_state, dtype=float).tolist())  # 转成 tuple[float,...]。
        first_control = self.controller.compute_control(  # 调用旧 B02 controller，得到当前要执行的第一步 torque。
            self.env,  # 传入 B02 env。
            current_state=current_state,  # 用关键字传入当前状态。
            target_sequence=target_sequence,  # 用关键字传入未来目标序列。
        )

        # B02 controller 通常会把最近一次规划详情放在 `last_plan`。
        # adapter 尽量读取这些信息，转换成 B03 的 predicted_* 和 metadata。
        plan = getattr(self.controller, "last_plan", None) or {}  # 安全读取 controller.last_plan；没有时用空字典。
        best_sequence = np.asarray(plan.get("best_sequence", [first_control]), dtype=float)  # 字典 `.get` 提供默认控制序列。
        if best_sequence.ndim == 1:  # 如果只有单步控制，数组 shape 可能是 (control_dim,)。
            best_sequence = best_sequence[None, :]  # `None` 新增一个 batch/time 维，变成 (1, control_dim)。

        predicted_states = None  # 默认没有预测状态。
        predicted_ee_positions = None  # 默认没有预测末端轨迹。
        best_index = plan.get("best_index", 0)  # 从 last_plan 读取最佳候选序号；没有就默认 0。
        candidate_rollouts = plan.get("candidate_rollouts")  # 尝试读取旧 controller 记录的候选状态轨迹。
        if isinstance(candidate_rollouts, list) and candidate_rollouts:  # 确认它是非空 list。
            predicted_states = np.asarray(candidate_rollouts[int(best_index)], dtype=float)  # 取出最佳 rollout 状态轨迹。
            try:  # 补算末端轨迹可能失败，例如 env 不支持 set_state；失败时降级为 None。
                # 如果旧 plan 只有状态轨迹，这里临时用 B02 env 补算末端轨迹。
                predicted_ee_positions = _compute_ee_positions_for_states(self.env, predicted_states)
            except Exception:  # 捕获补算末端轨迹的异常，不影响 baseline solver 输出控制。
                predicted_ee_positions = None  # 降级：没有末端预测轨迹。

        best_cost = float(plan.get("best_cost", np.nan))  # 读取最佳 cost；没有就用 NaN 表示未知。
        num_rollouts = len(plan.get("candidate_sequences", [])) if isinstance(plan.get("candidate_sequences"), list) else 0  # 条件表达式统计候选数量。

        return MPCSolution(  # 把旧 B02 controller 的输出包装成 B03 标准解对象。
            first_control=np.asarray(first_control, dtype=float),  # 当前闭环真正执行的第一步控制。
            predicted_states=predicted_states,  # 最佳预测状态轨迹，可能为 None。
            predicted_controls=best_sequence,  # 最佳控制序列。
            best_cost=best_cost,  # 最佳 cost。
            solver_name=self.solver_name,  # solver 名称来自类属性。
            solver_stats=SolverStats(  # 嵌套构造统计信息 dataclass。
                runtime_ms=(time.perf_counter() - start_time) * 1000.0,  # 当前时间减开始时间，秒转毫秒。
                num_rollouts=num_rollouts,  # 旧 controller 记录的候选数量。
                num_iterations=1,  # baseline wrapper 视为一次规划调用。
                success=True,  # 能返回到这里就认为包装成功。
                message="Wrapped B02 controller output.",  # 给日志看的说明。
            ),
            predicted_ee_positions=predicted_ee_positions,  # 最佳末端预测轨迹，可能为 None。
            selected_index=int(best_index) if best_index is not None else None,  # 条件表达式处理 None。
            metadata={"wrapped_last_plan": plan},  # 把原始 B02 last_plan 放进 metadata，方便复盘。
        )  # MPCSolution 构造结束。


# 工厂函数：接收旧 controller/env，返回一个符合 B03 BaseMPCSolver 接口的对象。
def wrap_b02_controller_as_solver(controller: Any, env: Any) -> BaseMPCSolver:
    """把 B02 baseline controller 包装成 B03 `BaseMPCSolver`。

    这是一个语义化的小工厂函数，让外部代码可以写：
    `solver = wrap_b02_controller_as_solver(controller, env)`。
    """
    return B02BaselineSolver(controller=controller, env=env)  # 实际创建并返回 B02BaselineSolver 实例。
