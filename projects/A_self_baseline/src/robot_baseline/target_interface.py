"""Target interface TODO skeleton.

A06 后续通过本模块生成 ``A06_target_definition.json``，A05 后续读取该
target definition 作为 IK 输入。本模块不启动 viewer，不调用 mink，不写
``data.ctrl``。
"""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
from typing import Any

import numpy as np

from robot_baseline.motion_types import TargetDefinition


def _array_to_list(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: _array_to_list(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_array_to_list(item) for item in value]
    return value


def parse_vector_optional(value: Any, length: int, name: str) -> np.ndarray | None:
    """解析可选向量，并做 shape 检查。

    输入：list/tuple/np.ndarray、逗号分隔字符串或 ``None``。
    输出：``np.ndarray``，shape=(length,)，或 ``None``。
    验证：长度不对时抛出带字段名的 ValueError。
    """
    if value is None:
        return None
    if isinstance(value, str):
        value = [float(item.strip()) for item in value.split(",") if item.strip()]
    vector = np.asarray(value, dtype=float)
    if vector.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},), got {vector.shape}")
    return vector


def quat_wxyz_to_rotation_matrix(quat_wxyz: Any) -> np.ndarray:
    """把 wxyz 四元数转换成旋转矩阵。

    为什么需要：配置/JSON 使用 wxyz，但姿态误差推荐用 rotation matrix。
    对标 mink：FrameTask 需要明确目标姿态，而不是混用 RPY。
    """
    q = parse_vector_optional(quat_wxyz, 4, "quat_wxyz")
    if q is None:
        raise ValueError("quat_wxyz cannot be None")
    norm = float(np.linalg.norm(q))
    if norm <= 0.0:
        raise ValueError("quat_wxyz norm must be positive")
    w, x, y, z = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotation_matrix_to_quat_wxyz(rotation_matrix: Any) -> np.ndarray:
    """把旋转矩阵转换成 wxyz 四元数。

    当前保留一个最小实现，后续若接入 SciPy，需要注意 SciPy 使用 xyzw。
    """
    matrix = np.asarray(rotation_matrix, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError(f"rotation_matrix must have shape (3, 3), got {matrix.shape}")

    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        raise NotImplementedError("TODO: 补全 trace<=0 的稳定分支，验证四元数 wxyz 顺序。")
    return np.asarray([w, x, y, z], dtype=float)


def load_target_definition(path: str | Path) -> TargetDefinition:
    """读取 TargetDefinition JSON。

    TODO:
    - 要做什么：读取 A06 生成的 target definition。
    - 为什么：A05 不应该私有生成 target，而应消费统一 target schema。
    - 对标 mink：viewer / mocap target 最终都要变成 task target。
    - 输入：JSON path。
    - 输出：TargetDefinition。
    - 验证：position shape=(3,)，quat_wxyz shape=(4,)。
    """
    target_path = Path(path).expanduser().resolve()
    data = json.loads(target_path.read_text(encoding="utf-8"))
    return TargetDefinition(
        target_id=data["target_id"],
        source=data["source"],
        target_site=data["target_site"],
        target_body=data.get("target_body"),
        coordinate_frame=data.get("coordinate_frame", "world"),
        position=parse_vector_optional(data.get("position"), 3, "position"),
        rotation_matrix=None if data.get("rotation_matrix") is None else np.asarray(data["rotation_matrix"], dtype=float),
        quat_wxyz=parse_vector_optional(data.get("quat_wxyz"), 4, "quat_wxyz"),
        position_offset=parse_vector_optional(data.get("position_offset"), 3, "position_offset"),
        orientation_mode=data.get("orientation_mode", "keep_current"),
        metadata=data.get("metadata", {}),
    )


def save_target_definition(target: TargetDefinition, path: str | Path) -> None:
    """保存 TargetDefinition JSON。

    输出：后续 A06 的 ``outputs/cache/A06_target_definition.json``。
    当前只负责 schema 序列化，不求 IK。
    """
    target_path = Path(path)
    target_path.parent.mkdir(parents=True, exist_ok=True)
    payload = _array_to_list(asdict(target))
    target_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def target_from_offset(current_position: Any, current_rotation: Any, offset: Any) -> TargetDefinition:
    """从当前位姿和 position offset 构造 fixed target。

    输入：current_position shape=(3,)，current_rotation shape=(3,3)，offset shape=(3,)。
    输出：TargetDefinition，position = current_position + offset。
    验证：target-current 的 position error 应等于 offset。
    """
    current_position_vec = parse_vector_optional(current_position, 3, "current_position")
    offset_vec = parse_vector_optional(offset, 3, "offset")
    rotation = np.asarray(current_rotation, dtype=float)
    if current_position_vec is None or offset_vec is None:
        raise ValueError("current_position and offset cannot be None")
    if rotation.shape != (3, 3):
        raise ValueError(f"current_rotation must have shape (3, 3), got {rotation.shape}")
    return TargetDefinition(
        target_id="offset_from_current",
        source="offset_from_current",
        target_site="attachment_site",
        coordinate_frame="world",
        position=current_position_vec + offset_vec,
        rotation_matrix=rotation,
        quat_wxyz=rotation_matrix_to_quat_wxyz(rotation),
        position_offset=offset_vec,
        orientation_mode="keep_current",
    )


def target_from_fixed_pose(position: Any, orientation_mode: str, rpy: Any = None, quat_wxyz: Any = None) -> TargetDefinition:
    """构造 fixed pose target 的骨架。

    当前只支持 ``orientation_mode="fixed_quat"`` 的 shape check。RPY 转换留给 R1/R2。
    """
    position_vec = parse_vector_optional(position, 3, "position")
    if position_vec is None:
        raise ValueError("position cannot be None")
    if orientation_mode == "fixed_quat":
        quat = parse_vector_optional(quat_wxyz, 4, "quat_wxyz")
        return TargetDefinition(
            target_id="fixed_pose",
            source="fixed_pose",
            target_site="attachment_site",
            coordinate_frame="world",
            position=position_vec,
            rotation_matrix=quat_wxyz_to_rotation_matrix(quat),
            quat_wxyz=quat,
            orientation_mode="fixed_quat",
        )
    raise NotImplementedError("TODO: 补 fixed_rpy / keep_current 的 fixed pose 解析和验证。")


def target_from_pose_sequence(start_pose: TargetDefinition, end_pose: TargetDefinition, num_waypoints: int) -> list[TargetDefinition]:
    """规划 pose sequence target。

    TODO:
    - 要做什么：在 start/end pose 之间生成多个 target。
    - 为什么：A06 后续需要从 fixed target 扩展到轨迹目标。
    - 对标 mink：viewer target 可以随时间变化，IK 每帧消费新 target。
    - 推荐 API：numpy.linspace，后续姿态用 slerp 或 keep_current。
    - 输入：start_pose、end_pose、num_waypoints。
    - 输出：TargetDefinition list。
    - 验证：len(result) == num_waypoints。
    """
    raise NotImplementedError("TODO: 实现 pose sequence 插值；当前 Step R-C 只定义接口。")


def target_from_mocap_placeholder(model_n_mocap: int, mocap_body_name: str) -> TargetDefinition:
    """构造 mocap target placeholder。

    验证：``model_n_mocap > 0`` 才能读取 ``data.mocap_pos[mocap_id]``。
    当前不启动 viewer，只记录未来数据流。
    """
    if model_n_mocap <= 0:
        raise ValueError("model.nmocap == 0; 当前模型没有 mocap body，不能访问 data.mocap_pos[0]。")
    return TargetDefinition(
        target_id=mocap_body_name,
        source="mocap_placeholder",
        target_site="attachment_site",
        coordinate_frame="world",
        orientation_mode="mocap_quat_wxyz",
        metadata={"model_n_mocap": model_n_mocap, "mocap_body_name": mocap_body_name},
    )


def resolve_target_definition(*args: Any, **kwargs: Any) -> TargetDefinition:
    """统一解析 target 来源。

    TODO:
    - 要做什么：根据 target source 分发到 offset/fixed/sequence/mocap。
    - 为什么：A04/A05/A06 不再各自私有定义 target。
    - 对标 mink：把 viewer/mocap target 统一成 FrameTask 可消费的目标 pose。
    - 输入：配置、CLI 参数、A02 current pose 或 mocap metadata。
    - 输出：TargetDefinition。
    - 验证：target schema 字段完整，position/quat shape 正确。
    """
    raise NotImplementedError("TODO: R1 完成 TargetDefinition load/save/validate 后补 resolve_target_definition。")
