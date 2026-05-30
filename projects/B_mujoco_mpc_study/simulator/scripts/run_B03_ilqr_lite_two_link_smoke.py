"""B03-R4C mini iLQR two-link smoke TODO skeleton.

本脚本是学习型骨架，不实现真实 two-link iLQR rollout。

当前目标：
- 把 B03-R4B 的 ILQGLiteSolver 调用位置预留出来；
- 把 B02 two-link MuJoCo 环境实例化位置预留出来；
- 明确后续需要补齐的 dynamics adapter、state tracking、task-space tracking、
  CEM/MPPI warm-start 和绘图输出；
- 保证 `python -m py_compile` 可以通过。

本脚本不做：
- 不运行长时间 MuJoCo 闭环仿真；
- 不生成正式 MP4；
- 不修改 B02 controller；
- 不修改 B02 benchmark / regression 配置。
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import logging
from pathlib import Path
import sys
from typing import Any, Callable

import numpy as np
import yaml

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import seaborn as sns


SCRIPT_PATH = Path(__file__).resolve()
SIMULATOR_ROOT = SCRIPT_PATH.parents[1]
PROJECT_ROOT = SIMULATOR_ROOT.parent
REPO_ROOT = PROJECT_ROOT.parent.parent

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(SIMULATOR_ROOT) not in sys.path:
    sys.path.insert(0, str(SIMULATOR_ROOT))

try:
    from projects.B_mujoco_mpc_study.simulator.envs.two_link_env import TwoLinkEnv
except ModuleNotFoundError as exc:
    if exc.name != "mujoco":
        raise
    TwoLinkEnv = Any  # type: ignore[misc, assignment]
    TWO_LINK_ENV_IMPORT_ERROR: ModuleNotFoundError | None = exc
else:
    TWO_LINK_ENV_IMPORT_ERROR = None

from projects.B_mujoco_mpc_study.simulator.planners.ilqg_solver import ILQGLiteSolver
from projects.B_mujoco_mpc_study.simulator.planners.mpc_solver_interface import (
    MPCProblem,
    MPCSolution,
)
from projects.B_mujoco_mpc_study.simulator.planners.sampling_mpc_solvers import shift_control_sequence


LOGGER = logging.getLogger(__name__)

TASK_NAME = "B03_ilqr_lite_two_link_smoke"
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "B03_mpc_solver_ladder.yaml"
DEFAULT_TWO_LINK_MODEL_PATH = SIMULATOR_ROOT / "models" / "B02_two_link.xml"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "outputs" / "runs" / TASK_NAME

# B03-R4C 后续会把这些接口接到更完整的任务系统中。
# 当前只保留名字和位置，避免在 TODO skeleton 阶段提前实现真实逻辑。
m_target_interface: Any | None = None
ik_interface: Any | None = None
viewer_interface: Any | None = None
actuator_interface: Any | None = None


def validate_state_shape(x: np.ndarray) -> None:
    """检查 two-link state 是否满足 x=[q1,q2,dq1,dq2]。

    这是非核心控制算法，允许在 skeleton 阶段真实实现。
    """
    x_array = np.asarray(x, dtype=float)
    if x_array.shape != (4,):
        raise ValueError(f"two-link state x must have shape (4,), got {x_array.shape}")
    if not np.isfinite(x_array).all():
        raise ValueError("two-link state x must contain only finite values.")


def validate_control_shape(u: np.ndarray) -> None:
    """检查 two-link control 是否满足 u=[tau1,tau2]。"""
    u_array = np.asarray(u, dtype=float)
    if u_array.shape != (2,):
        raise ValueError(f"two-link control u must have shape (2,), got {u_array.shape}")
    if not np.isfinite(u_array).all():
        raise ValueError("two-link control u must contain only finite values.")


def validate_nominal_trajectory_shapes(x_refs: np.ndarray, u_nominal: np.ndarray) -> None:
    """检查 state reference 和 nominal control 的 horizon 关系。

    iLQR-lite 约定：
    - x_refs.shape == (H+1, 4)
    - u_nominal.shape == (H, 2)
    """
    x_refs_array = np.asarray(x_refs, dtype=float)
    u_nominal_array = np.asarray(u_nominal, dtype=float)

    if x_refs_array.ndim != 2 or x_refs_array.shape[1] != 4:
        raise ValueError(f"x_refs must have shape (H+1, 4), got {x_refs_array.shape}")
    if u_nominal_array.ndim != 2 or u_nominal_array.shape[1] != 2:
        raise ValueError(f"u_nominal must have shape (H, 2), got {u_nominal_array.shape}")
    if x_refs_array.shape[0] != u_nominal_array.shape[0] + 1:
        raise ValueError(
            "x_refs and u_nominal horizon mismatch: "
            f"x_refs={x_refs_array.shape}, u_nominal={u_nominal_array.shape}"
        )
    if not np.isfinite(x_refs_array).all() or not np.isfinite(u_nominal_array).all():
        raise ValueError("x_refs and u_nominal must contain only finite values.")


def _forward_two_link_env(env: Any) -> None:
    """修改 qpos/qvel 后，尽量刷新 MuJoCo 派生量。

    TODO 教学说明：
    MuJoCo 中直接修改 qpos/qvel 后，site、body、传感器等派生量不会自动更新。
    因此状态写入和状态恢复后，通常需要 forward 一次。

    兼容顺序：
    1. 如果 env.forward() 存在，优先调用；
    2. 否则如果 env.model/env.data 存在，尝试 mujoco.mj_forward；
    3. fake env 或无 MuJoCo 环境中安全跳过。

    注意：这里绝不调用 env.step()，因为 forward 只刷新当前状态，不推进仿真时间。
    """
    # 先尝试读取 env.forward；fake env 和部分封装环境可能直接提供这个方法。
    forward_fn = getattr(env, "forward", None)
    # callable(...) 用来确认 forward_fn 真的是函数/方法，而不是普通属性。
    if callable(forward_fn):
        # 调用 forward 只刷新当前位置对应的派生量，不推进仿真一步。
        forward_fn()
        # 已经完成刷新，直接返回，避免继续尝试 MuJoCo 底层路径。
        return

    # 如果 env 没有 forward 方法，再尝试读取 MuJoCo 风格的 model。
    model = getattr(env, "model", None)
    # 同时读取 MuJoCo 风格的 data；mj_forward 需要 model 和 data 同时存在。
    data = getattr(env, "data", None)
    # fake env 或轻量测试对象可能没有 model/data，此时安全跳过。
    if model is None or data is None:
        return

    # 只有在确实需要 MuJoCo 底层 forward 时才导入 mujoco。
    try:
        import mujoco
    # 测试环境可能没有安装 mujoco；这里不能让 fake env 测试失败。
    except ModuleNotFoundError:
        return

    # MuJoCo 的 forward 会根据当前 qpos/qvel 重新计算 site/body 等派生量。
    mujoco.mj_forward(model, data)


def _state_from_mapping(raw_state: dict[str, Any]) -> np.ndarray:
    """把 dict 风格状态转换成 x=[q1,q2,dq1,dq2]。"""
    # 最直接的情况：调用方已经把完整 state 放在 "state" 字段里。
    if "state" in raw_state:
        # 转成 float ndarray 并 copy，避免返回外部 dict 中数组的 view。
        return np.asarray(raw_state["state"], dtype=float).copy()

    # 第二种兼容格式：MuJoCo 命名的 qpos/qvel。
    if "qpos" in raw_state and "qvel" in raw_state:
        # 只取前两个 qpos，对应 two-link 的 q1/q2，并立刻 copy 防止共享内存。
        q = np.asarray(raw_state["qpos"], dtype=float)[:2].copy()
        # 只取前两个 qvel，对应 two-link 的 dq1/dq2，并立刻 copy 防止共享内存。
        dq = np.asarray(raw_state["qvel"], dtype=float)[:2].copy()
        # 拼成统一状态 x=[q1,q2,dq1,dq2]，astype(..., copy=True) 保证返回独立数组。
        return np.concatenate([q, dq]).astype(float, copy=True)

    # 第三种兼容格式：更简洁的 q/dq 命名。
    if "q" in raw_state and "dq" in raw_state:
        # 读取两个关节角，转换为 float，并复制出来。
        q = np.asarray(raw_state["q"], dtype=float)[:2].copy()
        # 读取两个关节速度，转换为 float，并复制出来。
        dq = np.asarray(raw_state["dq"], dtype=float)[:2].copy()
        # 统一拼成 iLQR-lite 需要的一维状态向量。
        return np.concatenate([q, dq]).astype(float, copy=True)

    # dict 里没有任何已知格式时，不猜测字段含义，给出清晰 TODO。
    raise NotImplementedError(
        "B03-R4C-1B TODO: get_state() returned a dict, but it does not contain "
        "'state', 'qpos'+'qvel', or 'q'+'dq'. 后续应在 TwoLinkEnv 中补统一 state API。"
    )


def get_two_link_state(env: Any) -> np.ndarray:
    """读取当前 two-link 状态 x=[q1,q2,dq1,dq2]。

    读取优先级：
    1. env.get_state()；
    2. get_state() 返回 dict 时兼容 state / qpos+qvel / q+dq；
    3. fallback 到 env.data.qpos[:2] 和 env.data.qvel[:2]。
    """
    # 优先查找 env.get_state；真实 TwoLinkEnv 已经提供这个统一接口。
    get_state_fn = getattr(env, "get_state", None)
    # 只有 get_state 真的是可调用方法时才使用。
    if callable(get_state_fn):
        # 调用环境自己的状态读取函数，避免脚本猜测内部 MuJoCo 索引。
        raw_state = get_state_fn()
        # 如果返回 dict，说明状态可能是结构化字段，需要专门解析。
        if isinstance(raw_state, dict):
            # 交给兼容函数处理 "state" / "qpos"+"qvel" / "q"+"dq"。
            state = _state_from_mapping(raw_state)
        # 如果不是 dict，则按 tuple/list/ndarray 风格状态处理。
        else:
            # 统一转成 float ndarray，并 copy，避免共享 env 内部数组。
            state = np.asarray(raw_state, dtype=float).copy()
    # 如果没有 get_state，则进入 MuJoCo data fallback。
    else:
        # 尝试读取 env.data；MuJoCo 环境通常把 qpos/qvel 放在 data 里。
        data = getattr(env, "data", None)
        # 没有 data/qpos/qvel 就无法可靠读取 two-link 状态。
        if data is None or not hasattr(data, "qpos") or not hasattr(data, "qvel"):
            raise NotImplementedError(
                "B03-R4C-1B TODO: current two-link adapter needs env.get_state() "
                "or env.data.qpos/env.data.qvel. 后续应在 TwoLinkEnv 中补统一 state API。"
            )
        # 读取前两个 qpos，约定为 q1/q2；copy 防止返回 MuJoCo 内部 view。
        q = np.asarray(data.qpos[:2], dtype=float).copy()
        # 读取前两个 qvel，约定为 dq1/dq2；copy 防止返回 MuJoCo 内部 view。
        dq = np.asarray(data.qvel[:2], dtype=float).copy()
        # 拼接成统一状态 x=[q1,q2,dq1,dq2]。
        state = np.concatenate([q, dq]).astype(float, copy=True)

    # 所有路径最终都必须满足 two-link 状态 shape=(4,)。
    validate_state_shape(state)
    # iLQR 的 finite-difference 不能接受 NaN/inf，否则线性化会被污染。
    if not np.all(np.isfinite(state)):
        raise ValueError("two-link state contains non-finite values.")
    # 最终再次返回 float copy，保证调用者拿到的是独立状态快照。
    return np.asarray(state, dtype=float).copy()


def set_two_link_state(env: Any, x: np.ndarray) -> None:
    """把 env 设置到指定状态 x=[q1,q2,dq1,dq2]。

    这是状态写入 helper，不是 dynamics_fn：
    - 不 reset simulation；
    - 不 step 仿真；
    - 只设置当前位置/速度并 forward 刷新派生量。
    """
    # 先把输入状态统一成 float ndarray，并 copy，避免修改调用者传入的数组。
    state = np.asarray(x, dtype=float).copy()
    # 写入环境之前先检查 shape，防止把错误维度写进 qpos/qvel。
    validate_state_shape(state)
    # NaN/inf 写进仿真环境会让后续 forward/rollout 结果不可用。
    if not np.all(np.isfinite(state)):
        raise ValueError("two-link state x must contain only finite values.")

    # 优先使用 env.set_state；真实 TwoLinkEnv 知道自己的 joint index。
    set_state_fn = getattr(env, "set_state", None)
    # 只有 set_state 是可调用方法时才使用。
    if callable(set_state_fn):
        # 转成 tuple[float,...]，兼容 TwoLinkEnv.set_state 的教学接口。
        set_state_fn(tuple(float(value) for value in state.tolist()))
        # set_state 后再 forward 一次，保证派生量刷新；fake env 会记录调用次数。
        _forward_two_link_env(env)
        # 已经通过 env API 完成写入，直接返回。
        return

    # 没有 set_state 时，尝试 MuJoCo data fallback。
    data = getattr(env, "data", None)
    # 如果没有 data/qpos/qvel，就没有可靠方式写入 two-link 状态。
    if data is None or not hasattr(data, "qpos") or not hasattr(data, "qvel"):
        raise NotImplementedError(
            "B03-R4C-1B TODO: set_two_link_state needs env.set_state() "
            "or env.data.qpos/env.data.qvel. 后续应在 TwoLinkEnv 中补统一 state API。"
        )

    # 写入两个关节角 q1/q2；这里只设置状态，不 reset。
    data.qpos[:2] = state[:2]
    # 写入两个关节速度 dq1/dq2；这里只设置状态，不 step。
    data.qvel[:2] = state[2:]
    # 修改 qpos/qvel 后刷新派生量，但不推进仿真时间。
    _forward_two_link_env(env)


def save_two_link_env_state(env: Any) -> dict[str, Any]:
    """保存 two-link env 当前状态快照。

    snapshot 用 dict，是因为 fake env 和 MuJoCo env 可用字段不同。
    至少保存 "state"，如果存在 MuJoCo data，则额外保存 qpos/qvel/time/ctrl。
    """
    # 先通过统一 helper 读取标准状态 x=[q1,q2,dq1,dq2]。
    state = get_two_link_state(env)
    # snapshot 至少保存标准 state；copy 防止后续 env 变化污染快照。
    snapshot: dict[str, Any] = {"state": state.copy()}

    # 如果存在 MuJoCo 风格 data，就额外保存更完整的底层状态。
    data = getattr(env, "data", None)
    # fake env 也可能有 data；这里逐字段检查，保持兼容。
    if data is not None:
        # qpos 可能不止两个元素，完整保存便于后续原样恢复。
        if hasattr(data, "qpos"):
            snapshot["qpos"] = np.asarray(data.qpos, dtype=float).copy()
        # qvel 也完整保存，避免只恢复前两个速度导致底层状态不一致。
        if hasattr(data, "qvel"):
            snapshot["qvel"] = np.asarray(data.qvel, dtype=float).copy()
        # time 是仿真时间；rollout 临时 step 后需要恢复它。
        if hasattr(data, "time"):
            snapshot["time"] = float(data.time)
        # ctrl 是 actuator 当前控制缓存；也要 copy，避免 rollout 后污染真实控制。
        if hasattr(data, "ctrl"):
            snapshot["ctrl"] = np.asarray(data.ctrl, dtype=float).copy()

    # TwoLinkEnv.step() 会记录上一拍实际执行力矩。
    # dynamics_fn 里的临时 step 不应污染真实闭环的这份记录。
    if hasattr(env, "last_applied_torque"):
        last_torque = tuple(float(value) for value in getattr(env, "last_applied_torque"))
        if len(last_torque) != 2:
            raise ValueError("env.last_applied_torque must contain exactly 2 values.")
        snapshot["last_applied_torque"] = last_torque

    # 返回普通 dict，供 dynamics_fn 用 try/finally 恢复。
    return snapshot


def restore_two_link_env_state(env: Any, snapshot: dict[str, Any]) -> None:
    """恢复 save_two_link_env_state(env) 保存的快照。"""
    # snapshot 必须是 dict，否则无法按字段恢复。
    if not isinstance(snapshot, dict):
        raise ValueError(f"snapshot must be a dict, got {type(snapshot)!r}")
    # 至少要有标准 state；没有它说明快照不是由本 helper 生成的。
    if "state" not in snapshot:
        raise ValueError("snapshot must contain at least the 'state' field.")

    # 尝试读取 env.data；如果有完整 qpos/qvel，优先恢复底层数组。
    data = getattr(env, "data", None)
    # 记录是否走了 qpos/qvel 恢复路径；该路径后面需要显式 forward。
    restored_qpos_qvel = False
    # 只有 snapshot 和 env 同时具备 qpos/qvel 时，才走完整底层恢复。
    if (
        data is not None
        and "qpos" in snapshot
        and "qvel" in snapshot
        and hasattr(data, "qpos")
        and hasattr(data, "qvel")
    ):
        # 读取 snapshot 里的完整 qpos，转成 float 数组。
        qpos = np.asarray(snapshot["qpos"], dtype=float)
        # 读取 snapshot 里的完整 qvel，转成 float 数组。
        qvel = np.asarray(snapshot["qvel"], dtype=float)
        # 恢复前先检查 shape，避免把不匹配数组写进 MuJoCo data。
        if qpos.shape != np.asarray(data.qpos).shape or qvel.shape != np.asarray(data.qvel).shape:
            raise ValueError(
                "snapshot qpos/qvel shapes do not match env data: "
                f"snapshot qpos={qpos.shape}, env qpos={np.asarray(data.qpos).shape}; "
                f"snapshot qvel={qvel.shape}, env qvel={np.asarray(data.qvel).shape}"
            )
        # 原地恢复 qpos，保持 MuJoCo data 内部数组对象不变。
        data.qpos[:] = qpos
        # 原地恢复 qvel，保持 MuJoCo data 内部数组对象不变。
        data.qvel[:] = qvel
        # 标记已经恢复 qpos/qvel，后续需要 forward 刷新派生量。
        restored_qpos_qvel = True
    # 如果没有完整 qpos/qvel，就退回到标准 state 恢复。
    else:
        # 使用 set_two_link_state 统一处理 shape、finite 和 forward。
        set_two_link_state(env, np.asarray(snapshot["state"], dtype=float))

    # 如果有 data，再恢复 time/ctrl 这类非 state 但会被 rollout 污染的字段。
    if data is not None:
        # 恢复仿真时间，避免临时 rollout 改变真实闭环时间。
        if "time" in snapshot and hasattr(data, "time"):
            data.time = float(snapshot["time"])
        # 恢复 actuator 控制缓存。
        if "ctrl" in snapshot and hasattr(data, "ctrl"):
            # 转成 float 数组，便于 shape 检查和原地写入。
            ctrl = np.asarray(snapshot["ctrl"], dtype=float)
            # ctrl shape 必须和当前 env.data.ctrl 一致。
            if ctrl.shape != np.asarray(data.ctrl).shape:
                raise ValueError(
                    "snapshot ctrl shape does not match env data: "
                    f"snapshot ctrl={ctrl.shape}, env ctrl={np.asarray(data.ctrl).shape}"
                )
            # 原地恢复 ctrl，避免替换 MuJoCo 内部数组对象。
            data.ctrl[:] = ctrl

    # 恢复 TwoLinkEnv 记录的上一拍实际执行力矩。
    if "last_applied_torque" in snapshot and hasattr(env, "last_applied_torque"):
        last_torque = tuple(float(value) for value in snapshot["last_applied_torque"])
        if len(last_torque) != 2:
            raise ValueError("snapshot last_applied_torque must contain exactly 2 values.")
        env.last_applied_torque = last_torque

    # 只有直接写 qpos/qvel 的路径需要在这里 forward；set_two_link_state 内部已 forward。
    if restored_qpos_qvel:
        _forward_two_link_env(env)

    # 恢复后重新读取标准 state，确认 env 处在可读状态。
    restored_state = get_two_link_state(env)
    # 再次检查 shape，防止恢复逻辑把状态弄成错误维度。
    validate_state_shape(restored_state)
    # 再次检查 finite，防止 snapshot 或恢复过程引入 NaN/inf。
    if not np.all(np.isfinite(restored_state)):
        raise ValueError("restored two-link state contains non-finite values.")


def build_arg_parser() -> argparse.ArgumentParser:
    """创建命令行参数解析器。

    输入：
    - 无显式输入，读取命令行参数。

    输出：
    - argparse.ArgumentParser，供 main() 使用。
    """
    parser = argparse.ArgumentParser(
        description="B03-R4C TODO skeleton: connect ILQGLiteSolver to B02 two-link dynamics."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help="YAML 配置路径，默认使用 B03_mpc_solver_ladder.yaml。",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="输出根目录。若不提供，则写入 projects/B_mujoco_mpc_study/outputs/runs/B03_ilqr_lite_two_link_smoke/<timestamp>。",
    )
    parser.add_argument(
        "--log-level",
        choices=("INFO", "DEBUG"),
        default="INFO",
        help="日志等级。INFO 用于普通 smoke，DEBUG 用于后续排查 adapter 细节。",
    )
    return parser


def configure_logging(log_level: str) -> None:
    """配置脚本日志。

    这一步只影响本脚本的控制台输出，不改变 solver 或 controller 逻辑。
    """
    logging.basicConfig(
        level=getattr(logging, log_level),
        format="%(levelname)s %(name)s: %(message)s",
    )


def load_yaml_config(config_path: Path) -> dict[str, Any]:
    """读取 YAML 配置文件。

    TODO:
    - 后续 R4C 可以在这里增加 schema 检查，例如确认 model_family=two_link、
      ilqg_lite.horizon 存在、control_dim=2 等。
    - 当前只做最小读取，避免把配置解析写得过重。
    """
    if not config_path.exists():
        raise FileNotFoundError(f"找不到配置文件: {config_path}")

    with config_path.open("r", encoding="utf-8") as config_file:
        loaded = yaml.safe_load(config_file) or {}

    if not isinstance(loaded, dict):
        raise ValueError(f"配置文件必须是 YAML mapping，当前类型为: {type(loaded)!r}")

    return loaded


def ensure_output_dirs(output_dir: Path) -> dict[str, Path]:
    """创建 R4C skeleton 约定的输出目录。

    输入：
    - output_dir: 本次 smoke run 的根目录。

    输出：
    - 包含 cache / figures / reports / metrics 的路径字典。

    目录结构固定为：
    - outputs/cache/
    - outputs/figures/
    - outputs/reports/
    - outputs/metrics/
    """
    root = Path(output_dir)
    output_paths = {
        "cache": root / "outputs" / "cache",
        "figures": root / "outputs" / "figures",
        "reports": root / "outputs" / "reports",
        "metrics": root / "outputs" / "metrics",
    }
    for path in output_paths.values():
        path.mkdir(parents=True, exist_ok=True)
    return output_paths


def build_output_paths(output_dir: Path | None) -> dict[str, Path]:
    """创建 B03-R4C smoke 输出路径规划。

    输出目录固定分成四类：
    - cache: 保存后续可能需要复用的 numpy/cache 数据；
    - figures: 保存轨迹图和 cost history 图；
    - reports: 保存 smoke run 文本或 Markdown 报告；
    - metrics: 保存 cost、误差、runtime 等 CSV 指标。
    """
    run_dir = output_dir or DEFAULT_OUTPUT_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
    base_output_paths = ensure_output_dirs(run_dir)
    output_paths = {
        "run_dir": run_dir,
        **base_output_paths,
        "state_cache": run_dir / "outputs" / "cache" / "B03_R4C_two_link_state_tracking_placeholder.npz",
        "trajectory_figure": run_dir / "outputs" / "figures" / "B03_R4C_two_link_state_trajectory_todo.png",
        "cost_figure": run_dir / "outputs" / "figures" / "B03_R4C_ilqr_cost_history_todo.png",
        "todo_report": run_dir / "outputs" / "reports" / "B03_R4C_ilqr_two_link_todo_report.md",
        "smoke_report": run_dir / "outputs" / "reports" / "B03_R4C_ilqr_two_link_smoke_report.md",
        "metrics_csv": run_dir / "outputs" / "metrics" / "B03_R4C_ilqr_two_link_smoke_metrics.csv",
        "cost_history_csv": run_dir / "outputs" / "metrics" / "B03_R4C_ilqr_cost_history.csv",
    }

    return output_paths


def setup_environment(config_path: Path) -> tuple[TwoLinkEnv, np.ndarray]:
    """创建 B02 two-link 环境，并准备 state tracking 参考轨迹骨架。

    输入：
    - config_path: B03 YAML 配置路径。

    输出：
    - env: TwoLinkEnv 实例，后续用它构造 `x_{k+1}=f(x_k,u_k)` adapter。
    - x_ref: shape=(H+1, 4) 的状态参考轨迹占位，状态约定为
      x=[q1, q2, dq1, dq2]。

    TODO:
    - state tracking smoke run:
      后续应把 x_ref 设计成一条简单、稳定、短 horizon 的关节空间参考轨迹，
      例如固定目标角度或缓慢线性插值目标。
    - task-space tracking TODO:
      B02 原任务更关注末端位置误差，后续要从 env.get_end_effector_position()
      或 MuJoCo site 信息构造 task-space residual。
    """
    config = load_yaml_config(config_path)
    simulation_config = config.get("simulation", {})
    ilqg_config = config.get("ilqg_lite", {})

    if TWO_LINK_ENV_IMPORT_ERROR is not None:
        raise RuntimeError(
            "B03-R4C TODO skeleton can be imported without MuJoCo, "
            "but setup_environment() requires the mujoco package to instantiate TwoLinkEnv."
        ) from TWO_LINK_ENV_IMPORT_ERROR

    dt = float(simulation_config.get("dt", 0.01))
    model_path = Path(simulation_config.get("model_path", DEFAULT_TWO_LINK_MODEL_PATH))
    if not model_path.is_absolute():
        model_path = (PROJECT_ROOT / model_path).resolve()
    if not model_path.exists():
        model_path = DEFAULT_TWO_LINK_MODEL_PATH

    end_effector_site = str(simulation_config.get("end_effector_site", "ee_site"))

    # 这里保留 B02 two-link dynamics 环境实例化。
    # 后续 iLQR 的 dynamics_fn 会围绕这个 env 做 one-step adapter。
    env = TwoLinkEnv(
        model_path=model_path,
        dt=dt,
        end_effector_site=end_effector_site,
    )

    initial_q = simulation_config.get("initial_q", [0.3, 0.4])
    initial_dq = simulation_config.get("initial_dq", [0.0, 0.0])
    env.reset(q=(float(initial_q[0]), float(initial_q[1])), dq=(float(initial_dq[0]), float(initial_dq[1])))

    horizon = int(ilqg_config.get("horizon", 32))
    initial_state = get_two_link_state(env)

    # 当前只生成 state reference 占位。
    # TODO: 在 R4C 后续实现中，把这里替换为明确的 two-link state tracking 目标。
    x_ref = np.repeat(initial_state[None, :], horizon + 1, axis=0)

    LOGGER.info("Two-link env 已实例化: model=%s, dt=%s, horizon=%s", model_path, dt, horizon)
    LOGGER.debug("state reference placeholder shape=%s", x_ref.shape)

    return env, x_ref


def build_two_link_dynamics_fn(env: TwoLinkEnv) -> Callable[[np.ndarray, np.ndarray], np.ndarray]:
    """返回 iLQR-lite 可调用的一步动力学函数 dynamics_fn(x, u) -> x_next。

    本步实现的是 B03-R4C-1C 的最小 adapter：
    - 保存 env 当前状态；
    - 设置 env 到输入 x；
    - 执行一步控制 u；
    - 读取 x_next；
    - 恢复 env 原始状态；
    - 返回 x_next。

    注意：
    - 这里不构造完整 MPCProblem，也不调用 ILQGLiteSolver.solve()。
    - dynamics_fn 会被 iLQR 的 rollout 和 finite-difference 多次调用，
      所以每次调用都必须恢复 env，不能污染真实闭环状态。
    """
    step_fn = getattr(env, "step", None)
    if not callable(step_fn):
        raise NotImplementedError(
            "B03-R4C-1C TODO: build_two_link_dynamics_fn needs env.step(torque) "
            "to compute x_next = f(x, u)."
        )

    def dynamics_fn(x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """执行一次无副作用的 two-link 离散动力学查询。"""
        state = np.asarray(x, dtype=float).copy()
        control = np.asarray(u, dtype=float).copy()
        validate_state_shape(state)
        validate_control_shape(control)

        snapshot = save_two_link_env_state(env)
        try:
            # 1. 把环境临时放到 iLQR 想象中的状态 x。
            set_two_link_state(env, state)
            # 2. 执行一步控制 u，得到假想未来状态。
            step_fn(tuple(float(value) for value in control.tolist()))
            # 3. 从 env 读取标准状态，避免依赖 step() 的具体返回类型。
            next_state = get_two_link_state(env)
            validate_state_shape(next_state)
            return np.asarray(next_state, dtype=float).copy()
        finally:
            # 无论 step 是否成功，都尽量恢复真实闭环环境。
            restore_two_link_env_state(env, snapshot)

    return dynamics_fn


def _diagonal_weight_matrix(raw_value: Any, dim: int, default_diagonal: list[float], name: str) -> np.ndarray:
    """把教学配置中的权重转换成 iLQR 使用的二维方阵。"""
    if raw_value is None:
        matrix = np.diag(np.asarray(default_diagonal, dtype=float))
    else:
        value = np.asarray(raw_value, dtype=float)
        if value.ndim == 0:
            matrix = np.eye(dim, dtype=float) * float(value)
        elif value.ndim == 1:
            if value.shape != (dim,):
                raise ValueError(f"{name} weight vector must have shape ({dim},), got {value.shape}")
            matrix = np.diag(value)
        elif value.ndim == 2:
            matrix = value
        else:
            raise ValueError(f"{name} must be a scalar, vector, or matrix, got shape {value.shape}")

    if matrix.shape != (dim, dim):
        raise ValueError(f"{name} matrix must have shape ({dim}, {dim}), got {matrix.shape}")
    if not np.isfinite(matrix).all():
        raise ValueError(f"{name} matrix must contain only finite values.")
    return np.asarray(matrix, dtype=float).copy()


def _build_state_tracking_cost_matrices(config: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """从 YAML config 构造 state tracking 的 Q/R/Q_terminal。"""
    cost_config = dict(config.get("cost", {}))

    q_default = [10.0, 10.0, 1.0, 1.0]
    r_default = [0.1, 0.1]
    q_terminal_default = [20.0, 20.0, 2.0, 2.0]

    Q = _diagonal_weight_matrix(cost_config.get("Q"), dim=4, default_diagonal=q_default, name="Q")
    R = _diagonal_weight_matrix(cost_config.get("R"), dim=2, default_diagonal=r_default, name="R")
    Q_terminal = _diagonal_weight_matrix(
        cost_config.get("Q_terminal"),
        dim=4,
        default_diagonal=q_terminal_default,
        name="Q_terminal",
    )

    return Q, R, Q_terminal


def build_warm_start_controls_from_previous_solution(
    previous_solution: MPCSolution | None,
    horizon: int,
    control_dim: int,
) -> np.ndarray | None:
    """从 sampling-family 的上一条预测控制序列构造 iLQR 初始控制。

    输入：
    - previous_solution.predicted_controls: shape=(H, control_dim)。

    输出：
    - shifted controls，满足 `u_init[:-1] = previous_controls[1:]`。
    - 如果 previous_solution 缺失则返回 None。
    """
    if previous_solution is None or previous_solution.predicted_controls is None:
        return None

    expected_shape = (int(horizon), int(control_dim))
    previous_controls = np.asarray(previous_solution.predicted_controls, dtype=float)
    if previous_controls.shape != expected_shape:
        raise ValueError(
            "previous_solution.predicted_controls shape must match "
            f"{expected_shape}, got {previous_controls.shape}"
        )
    if not np.isfinite(previous_controls).all():
        raise ValueError("previous_solution.predicted_controls must contain only finite values.")

    return shift_control_sequence(previous_controls)


def build_state_tracking_problem(
    env: TwoLinkEnv,
    dynamics_fn: Callable[[np.ndarray, np.ndarray], np.ndarray],
    x0: np.ndarray,
    x_refs: np.ndarray,
    u_nominal: np.ndarray,
    config: dict[str, Any],
) -> MPCProblem:
    """构造 state tracking MPCProblem。

    B03-R4C-1D 的最小目标：
    - 检查 x0 shape == (4,)；
    - 检查 x_refs shape == (H+1, 4)；
    - 检查 u_nominal shape == (H, 2)；
    - 构造 Q, R, Q_terminal；
    - 构造 MPCProblem；
    - 后续传给 ILQGLiteSolver.solve(problem)。

    本函数只定义关节空间 state tracking，不实现 task-space 末端跟踪 cost。
    """
    current_state = np.asarray(x0, dtype=float).copy()
    reference_states = np.asarray(x_refs, dtype=float).copy()
    initial_controls = np.asarray(u_nominal, dtype=float).copy()

    validate_state_shape(current_state)
    validate_nominal_trajectory_shapes(x_refs=reference_states, u_nominal=initial_controls)
    if not callable(dynamics_fn):
        raise ValueError("dynamics_fn must be callable as dynamics_fn(x, u) -> x_next.")

    horizon = int(initial_controls.shape[0])
    control_dim = int(initial_controls.shape[1])
    simulation_config = dict(config.get("simulation", {}))
    ilqg_config = dict(config.get("ilqg_lite", {}))
    dt = float(simulation_config.get("dt", getattr(env, "dt", 0.01)))
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")

    Q, R, Q_terminal = _build_state_tracking_cost_matrices(config)
    solver_config = dict(ilqg_config)
    solver_config.setdefault("horizon", horizon)

    metadata = {
        "task_name": "B03-R4C-1D_state_tracking",
        "state_convention": "x=[q1,q2,dq1,dq2]",
        "control_convention": "u=[tau1,tau2]",
        "x_refs": reference_states,
        "initial_controls": initial_controls,
        "Q": Q,
        "R": R,
        "Q_terminal": Q_terminal,
        "dynamics_fn": dynamics_fn,
    }

    return MPCProblem(
        current_state=current_state,
        target_horizon=reference_states,
        horizon=horizon,
        control_dim=control_dim,
        dt=dt,
        cost_config={
            "Q": Q,
            "R": R,
            "Q_terminal": Q_terminal,
        },
        solver_config=solver_config,
        dynamics_fn=dynamics_fn,
        metadata=metadata,
    )


def setup_solver(config_path: Path) -> ILQGLiteSolver:
    """创建 ILQGLiteSolver 实例。

    TODO:
    - 后续可以从 config_path 读取 regularization、line_search_alphas、
      tolerance、control_limit 等参数，并显式传入 ILQGConfig。
    - 当前骨架只保留 solver 对象创建，避免在 R4C TODO 阶段改 solver 内部逻辑。
    """
    _ = load_yaml_config(config_path)
    solver = ILQGLiteSolver()
    LOGGER.info("ILQGLiteSolver 已创建，等待 two-link MPCProblem adapter 补全。")
    return solver


def build_two_link_ilqr_problem_todo(env: TwoLinkEnv, x_ref: np.ndarray) -> MPCProblem:
    """构造 two-link iLQR-lite MPCProblem 的 TODO 入口。

    这里故意不实现真实物理 adapter。

    后续需要补齐：
    - dynamics_fn(x, u) -> x_next:
      需要临时保存 env 当前真实状态，把 env 设置到 x，执行一次 torque u，
      读取下一状态，然后恢复真实状态，避免 rollout 污染闭环环境。
    - quadratic tracking cost:
      需要在 metadata 中放入 x_refs、Q、R、Q_terminal。
    - CEM/MPPI warm-start TODO:
      可以把 sampling-family 的 predicted_controls 作为 previous_solution，
      交给 ILQGLiteSolver.solve(problem, previous_solution=...) 做 shifted warm start。
    """
    current_state = get_two_link_state(env)
    horizon = int(x_ref.shape[0] - 1)

    # TODO: R4C 后续在这里定义真实 two-link dynamics adapter。
    # def dynamics_fn(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    #     ...

    # TODO: R4C 后续在这里定义 tracking 权重。
    # Q = np.diag([...])
    # R = np.diag([...])
    # Q_terminal = np.diag([...])

    # 这行展示后续应构造的标准 MPCProblem 字段，但当前不返回半成品 problem。
    _problem_preview = MPCProblem(
        current_state=current_state,
        target_horizon=x_ref,
        horizon=horizon,
        control_dim=2,
        dt=float(env.dt),
        cost_config={},
        solver_config={},
        dynamics_fn=None,
        metadata={
            "x_refs": x_ref,
            "TODO": "补齐 dynamics_fn、Q、R、Q_terminal 后再交给 ILQGLiteSolver.solve。",
        },
    )
    _ = _problem_preview

    raise NotImplementedError(
        "B03-R4C TODO: two-link dynamics adapter 尚未实现；"
        "请先补齐 dynamics_fn(x,u)、x_refs、Q/R/Q_terminal，再调用 ILQGLiteSolver.solve(problem)。"
    )


def run_smoke_simulation(
    env: TwoLinkEnv,
    solver: ILQGLiteSolver,
    x_ref: np.ndarray,
    config: dict[str, Any] | None = None,
    output_paths: dict[str, Path] | None = None,
    previous_solution: MPCSolution | None = None,
) -> MPCSolution:
    """运行 B03-R4C two-link smoke 仿真骨架。

    输入：
    - env: B02 two-link 环境。
    - solver: B03-R4B 已实现的 ILQGLiteSolver。
    - x_ref: state tracking 参考轨迹。

    输出：
    - MPCSolution。当前 TODO 阶段不会真实返回，后续补齐 problem 后返回 solver 结果。

    TODO:
    - 状态跟踪 smoke run:
      先只验证关节空间状态 x=[q1,q2,dq1,dq2] 能被 mini iLQR-lite 处理。
    - task-space tracking TODO:
      后续再把末端位置 p_ee(q) 的误差接入 cost 或外层 adapter。
    - CEM/MPPI warm-start TODO:
      后续可先运行 sampling solver，再把它的 predicted_controls 作为 iLQR 初值。
    """
    x0 = get_two_link_state(env)
    validate_state_shape(x0)

    horizon = int(np.asarray(x_ref).shape[0] - 1)
    warm_start_controls = build_warm_start_controls_from_previous_solution(
        previous_solution=previous_solution,
        horizon=horizon,
        control_dim=2,
    )
    warm_start_used = warm_start_controls is not None
    u_nominal = warm_start_controls if warm_start_controls is not None else np.zeros((horizon, 2), dtype=float)
    validate_nominal_trajectory_shapes(x_refs=x_ref, u_nominal=u_nominal)

    # TODO: two-link dynamics adapter 的契约入口。
    # 当前 build_two_link_dynamics_fn 仍抛 NotImplementedError；
    # 下一步会在这里补齐 x_next = f(x, u) 的局部实现。
    dynamics_fn = build_two_link_dynamics_fn(env=env)

    # TODO: state tracking MPCProblem 的契约入口。
    # 后续需要把 x_refs、Q/R/Q_terminal、initial_controls 和 dynamics_fn 放进 problem。
    problem = build_state_tracking_problem(
        env=env,
        dynamics_fn=dynamics_fn,
        x0=x0,
        x_refs=x_ref,
        u_nominal=u_nominal,
        config=config or {},
    )
    problem.metadata["warm_start_source"] = (
        getattr(previous_solution, "solver_name", "previous_solution") if warm_start_used else "zeros"
    )
    problem.metadata["warm_start_used"] = bool(warm_start_used)

    # solver 调用位置保留在这里。
    # 后续 build_state_tracking_problem 补齐后，本行应成为 two-link smoke 的核心入口。
    solution = solver.solve(problem, previous_solution=previous_solution)
    if output_paths is not None:
        write_smoke_outputs(solution=solution, problem=problem, output_paths=output_paths)
        write_smoke_report(solution=solution, problem=problem, output_paths=output_paths)
    return solution


def write_smoke_outputs(
    solution: MPCSolution,
    problem: MPCProblem,
    output_paths: dict[str, Path],
) -> None:
    """写入 B03-R4C-1E 的最小 smoke 可复盘输出。

    当前只保存：
    - cost history CSV；
    - 单行 metrics CSV；
    - state/control/reference cache npz。
    """
    predicted_states = solution.predicted_states
    if predicted_states is None:
        raise ValueError("solution.predicted_states is required for B03-R4C smoke outputs.")

    cost_history = [float(value) for value in solution.metadata.get("cost_history", [])]
    if not cost_history:
        cost_history = [float(solution.best_cost)]

    cost_history_path = output_paths["cost_history_csv"]
    with cost_history_path.open("w", encoding="utf-8", newline="") as cost_file:
        writer = csv.DictWriter(cost_file, fieldnames=["iteration", "cost"])
        writer.writeheader()
        for iteration, cost in enumerate(cost_history):
            writer.writerow({"iteration": iteration, "cost": cost})

    predicted_controls = np.asarray(solution.predicted_controls, dtype=float)
    predicted_states_array = np.asarray(predicted_states, dtype=float)
    x_refs = np.asarray(problem.metadata.get("x_refs", problem.target_horizon), dtype=float)

    metrics = {
        "solver_name": solution.solver_name,
        "success": bool(solution.solver_stats.success),
        "message": solution.solver_stats.message,
        "termination_reason": solution.metadata.get("termination_reason", ""),
        "warm_start_used": bool(problem.metadata.get("warm_start_used", False)),
        "warm_start_source": str(problem.metadata.get("warm_start_source", "zeros")),
        "horizon": int(problem.horizon),
        "state_dim": int(predicted_states_array.shape[1]),
        "control_dim": int(problem.control_dim),
        "best_cost": float(solution.best_cost),
        "initial_cost": float(cost_history[0]),
        "final_cost": float(cost_history[-1]),
        "num_cost_entries": int(len(cost_history)),
        "runtime_ms": float(solution.solver_stats.runtime_ms),
        "num_rollouts": int(solution.solver_stats.num_rollouts),
        "num_iterations": int(solution.solver_stats.num_iterations),
        "max_abs_control": float(np.max(np.abs(predicted_controls))) if predicted_controls.size else 0.0,
        "final_state_error_norm": float(np.linalg.norm(predicted_states_array[-1] - x_refs[-1])),
    }

    metrics_path = output_paths["metrics_csv"]
    with metrics_path.open("w", encoding="utf-8", newline="") as metrics_file:
        writer = csv.DictWriter(metrics_file, fieldnames=list(metrics.keys()))
        writer.writeheader()
        writer.writerow(metrics)

    np.savez(
        output_paths["state_cache"],
        predicted_states=predicted_states_array,
        predicted_controls=predicted_controls,
        x_refs=x_refs,
        initial_controls=np.asarray(problem.metadata.get("initial_controls"), dtype=float),
        first_control=np.asarray(solution.first_control, dtype=float),
        cost_history=np.asarray(cost_history, dtype=float),
    )

    LOGGER.info("B03-R4C smoke cost history 已写入: %s", cost_history_path)
    LOGGER.info("B03-R4C smoke metrics 已写入: %s", metrics_path)
    LOGGER.info("B03-R4C smoke cache 已写入: %s", output_paths["state_cache"])


def plot_trajectory(solution: MPCSolution, x_ref: np.ndarray, output_dir: Path) -> None:
    """绘制 two-link state tracking 的关节角轨迹图。

    当前只画 q1/q2 的 predicted vs reference，后续 task-space tracking 再画末端 xy。
    """
    predicted_states = solution.predicted_states
    if predicted_states is None:
        raise ValueError("solution.predicted_states is required for trajectory plotting.")

    states = np.asarray(predicted_states, dtype=float)
    refs = np.asarray(x_ref, dtype=float)
    if states.ndim != 2 or states.shape[1] < 2:
        raise ValueError(f"predicted_states must have shape (H+1, >=2), got {states.shape}")
    if refs.shape[0] != states.shape[0] or refs.shape[1] < 2:
        raise ValueError(f"x_ref must match predicted_states rows and have >=2 columns, got {refs.shape}")

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "B03_R4C_two_link_state_trajectory_todo.png"

    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    pal = sns.cubehelix_palette(6, rot=-0.25, light=0.7)
    time_index = np.arange(states.shape[0])

    fig, axes = plt.subplots(1, 2, figsize=(11, 4), dpi=150, sharex=True)
    joint_specs = [(0, "q1"), (1, "q2")]
    for ax, (joint_index, joint_name) in zip(axes, joint_specs):
        ax.plot(time_index, refs[:, joint_index], color=pal[5], linewidth=2.2, label="reference")
        ax.plot(time_index, states[:, joint_index], color="#4575b4", linewidth=1.8, linestyle="--", label="predicted")
        final_error = float(states[-1, joint_index] - refs[-1, joint_index])
        ax.text(
            0.98,
            0.06,
            f"final error: {final_error:+.2e} rad",
            transform=ax.transAxes,
            ha="right",
            va="bottom",
            fontsize=8,
            color="dimgrey",
            style="italic",
        )
        ax.set_title(f"{joint_name} state tracking", fontsize=11, loc="left", color="dimgrey")
        ax.set_xlabel("step", fontsize=10, labelpad=6, color="dimgrey")
        ax.set_ylabel("angle (rad)", fontsize=10, labelpad=6, color="dimgrey")
        ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
        ax.grid(False)
        ax.patch.set_edgecolor("lightgrey")
        ax.patch.set_linewidth(0.8)

    axes[0].legend(
        frameon=True,
        facecolor="white",
        framealpha=0.8,
        edgecolor="lightgrey",
        labelcolor="dimgrey",
        fontsize=9,
        loc="upper left",
    )
    fig.suptitle("B03-R4C iLQR-lite State Tracking Smoke", fontsize=13, color="dimgrey")
    sns.despine(left=True, bottom=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    LOGGER.info("B03-R4C state tracking figure 已写入: %s", output_path)


def plot_cost_history(solution: MPCSolution, output_dir: Path) -> None:
    """绘制 iLQR-lite cost history 曲线。

    输入来自 `solution.metadata['cost_history']`。
    """
    cost_history = np.asarray(solution.metadata.get("cost_history", [solution.best_cost]), dtype=float)
    if cost_history.ndim != 1 or cost_history.size == 0 or not np.isfinite(cost_history).all():
        raise ValueError("cost_history must be a finite 1D array with at least one value.")

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "B03_R4C_ilqr_cost_history_todo.png"

    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    pal = sns.cubehelix_palette(6, rot=-0.25, light=0.7)
    iterations = np.arange(cost_history.size)

    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=150)
    ax.plot(iterations, cost_history, color=pal[5], linewidth=2.2, marker="o", markersize=5)
    ax.scatter(iterations, cost_history, color=pal[5], edgecolors="white", linewidths=0.8, zorder=4)
    ax.set_title("B03-R4C iLQR-lite Cost History", fontsize=13, loc="left", color="dimgrey")
    ax.set_xlabel("iteration", fontsize=10, labelpad=6, color="dimgrey")
    ax.set_ylabel("total cost", fontsize=10, labelpad=6, color="dimgrey")
    ax.text(
        0.98,
        0.94,
        f"final cost: {float(cost_history[-1]):.2e}",
        transform=ax.transAxes,
        ha="right",
        va="top",
        fontsize=9,
        color="dimgrey",
        style="italic",
    )
    ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
    ax.grid(False)
    sns.despine(left=True, bottom=True)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    LOGGER.info("B03-R4C cost history figure 已写入: %s", output_path)


def _relative_output_path(path: Path, run_dir: Path) -> str:
    """把输出路径尽量写成相对 run_dir 的形式，方便报告阅读。"""
    try:
        return str(path.relative_to(run_dir)).replace("\\", "/")
    except ValueError:
        return str(path).replace("\\", "/")


def write_smoke_report(
    solution: MPCSolution,
    problem: MPCProblem,
    output_paths: dict[str, Path],
) -> None:
    """写入 B03-R4C-1G smoke Markdown 报告。"""
    predicted_states = solution.predicted_states
    if predicted_states is None:
        raise ValueError("solution.predicted_states is required for smoke report.")

    states = np.asarray(predicted_states, dtype=float)
    controls = np.asarray(solution.predicted_controls, dtype=float)
    x_refs = np.asarray(problem.metadata.get("x_refs", problem.target_horizon), dtype=float)
    cost_history = np.asarray(solution.metadata.get("cost_history", [solution.best_cost]), dtype=float)
    final_state_error_norm = float(np.linalg.norm(states[-1] - x_refs[-1]))
    max_abs_control = float(np.max(np.abs(controls))) if controls.size else 0.0
    run_dir = output_paths["run_dir"]
    report_path = output_paths["smoke_report"]

    report_lines = [
        "# B03-R4C iLQR Two-Link Smoke Report",
        "",
        "## Summary",
        "",
        "This smoke run connects the B03 mini iLQR-lite solver to the B02 two-link MuJoCo environment for joint-space state tracking.",
        "It is still a short learning smoke, not a task-space benchmark or video demo.",
        "",
        "## Key Metrics",
        "",
        f"- solver_name: `{solution.solver_name}`",
        f"- success: `{solution.solver_stats.success}`",
        f"- message: `{solution.solver_stats.message}`",
        f"- termination_reason: `{solution.metadata.get('termination_reason', '')}`",
        f"- warm_start_used: `{bool(problem.metadata.get('warm_start_used', False))}`",
        f"- warm_start_source: `{problem.metadata.get('warm_start_source', 'zeros')}`",
        f"- horizon: `{problem.horizon}`",
        f"- state_dim: `{states.shape[1]}`",
        f"- control_dim: `{problem.control_dim}`",
        f"- best_cost: `{float(solution.best_cost):.12g}`",
        f"- initial_cost: `{float(cost_history[0]):.12g}`",
        f"- final_cost: `{float(cost_history[-1]):.12g}`",
        f"- cost_entries: `{cost_history.size}`",
        f"- runtime_ms: `{float(solution.solver_stats.runtime_ms):.6g}`",
        f"- num_rollouts: `{solution.solver_stats.num_rollouts}`",
        f"- num_iterations: `{solution.solver_stats.num_iterations}`",
        f"- max_abs_control: `{max_abs_control:.12g}`",
        f"- final_state_error_norm: `{final_state_error_norm:.12g}`",
        "",
        "## Output Files",
        "",
        f"- metrics_csv: `{_relative_output_path(output_paths['metrics_csv'], run_dir)}`",
        f"- cost_history_csv: `{_relative_output_path(output_paths['cost_history_csv'], run_dir)}`",
        f"- state_cache: `{_relative_output_path(output_paths['state_cache'], run_dir)}`",
        f"- trajectory_figure: `{_relative_output_path(output_paths['trajectory_figure'], run_dir)}`",
        f"- cost_figure: `{_relative_output_path(output_paths['cost_figure'], run_dir)}`",
        "",
        "## Current Scope",
        "",
        "- Uses joint-space state tracking: `x=[q1,q2,dq1,dq2]`.",
        "- Uses torque input: `u=[tau1,tau2]`.",
        "- Does not implement task-space end-effector tracking yet.",
        "- Does not generate a formal MP4 or long benchmark yet.",
        "- Does not modify the B02 controller core logic.",
        "",
    ]

    report_path.write_text("\n".join(report_lines), encoding="utf-8")
    LOGGER.info("B03-R4C smoke report 已写入: %s", report_path)


def write_todo_report(output_paths: dict[str, Path], message: str) -> None:
    """写入一个最小 TODO 报告，方便确认输出路径规划已经生效。"""
    report_path = output_paths["todo_report"]
    report_path.write_text(
        "\n".join(
            [
                "# B03-R4C iLQR Two-Link TODO Smoke Report",
                "",
                "当前脚本是 TODO skeleton，尚未实现真实 two-link iLQR rollout。",
                "",
                f"- cache: `{output_paths['cache']}`",
                f"- figures: `{output_paths['figures']}`",
                f"- reports: `{output_paths['reports']}`",
                f"- metrics: `{output_paths['metrics']}`",
                "",
                f"运行状态: {message}",
                "",
            ]
        ),
        encoding="utf-8",
    )
    LOGGER.info("TODO report 已写入: %s", report_path)


def main(argv: list[str] | None = None) -> None:
    """脚本入口。

    当前 main() 只串起路径、日志、环境、solver 和 TODO smoke 调用框架。
    未实现位置会给出清晰 NotImplementedError 信息，不做长时间 MuJoCo 仿真。
    """
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    configure_logging(args.log_level)

    output_paths = build_output_paths(args.output_dir)
    LOGGER.info("B03-R4C 输出根目录: %s", output_paths["run_dir"])

    try:
        config = load_yaml_config(args.config)
        env, x_ref = setup_environment(config_path=args.config)
        solver = setup_solver(config_path=args.config)
        solution = run_smoke_simulation(
            env=env,
            solver=solver,
            x_ref=x_ref,
            config=config,
            output_paths=output_paths,
        )

        # 当前绘图函数也是 TODO。后续补齐 solve 后，再打开这两行。
        plot_trajectory(solution=solution, x_ref=x_ref, output_dir=output_paths["figures"])
        plot_cost_history(solution=solution, output_dir=output_paths["figures"])
    except NotImplementedError as exc:
        LOGGER.warning("B03-R4C skeleton 停在预期 TODO: %s", exc)
        write_todo_report(output_paths=output_paths, message=str(exc))


if __name__ == "__main__":
    main()
