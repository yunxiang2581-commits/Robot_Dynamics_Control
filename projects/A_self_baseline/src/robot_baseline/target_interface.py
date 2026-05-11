"""Target interface for A project.

本模块负责 TargetDefinition 的读写、基础验证和最小目标构造。

边界：
- 不启动 MuJoCo viewer；
- 不调用 mink 替代实现；
- 不写 data.ctrl；
- 不求解 IK。

R1 当前只补：
- parse_vector_optional；
- load_target_definition；
- save_target_definition；
- validate_target_definition；
- target_from_offset 的基础可用路径。
"""

from __future__ import annotations

from dataclasses import asdict
import json
from pathlib import Path
from typing import Any

import numpy as np

from robot_baseline.motion_types import TargetDefinition


def _array_to_list(value: Any) -> Any:
    """把 dataclass 中的 numpy array 递归转换成 JSON 可写的 list。"""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: _array_to_list(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_array_to_list(item) for item in value]
    return value


def parse_vector_optional(value: Any, length: int, name: str) -> np.ndarray | None:
    """解析可选向量，并做 shape / finite 检查。

    要做什么：
    - 支持 None、list、tuple、np.ndarray、逗号分隔字符串；
    - 输出 shape=(length,) 的 float ndarray；
    - 空字符串按 None 处理。

    为什么需要：
    target JSON / YAML / CLI 的输入形式可能不同。进入 IK 前必须统一成数组，
    否则 FrameTask 或 QP 里会出现更难定位的 shape 错误。

    输入：value、期望长度 length、字段名 name。
    输出：np.ndarray 或 None。
    验证：长度不对、NaN、Inf 都抛出 ValueError。
    """
    if value is None:
        return None
    if isinstance(value, str):
        if not value.strip():
            return None
        value = [float(item.strip()) for item in value.split(",") if item.strip()]

    vector = np.asarray(value, dtype=float)
    if vector.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},), got {vector.shape}")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} contains NaN or Inf")
    return vector


def parse_rotation_matrix_optional(value: Any, name: str = "rotation_matrix") -> np.ndarray | None:
    """解析可选旋转矩阵，并做基础 shape / finite 检查。

    输入：None、list、tuple 或 np.ndarray。
    输出：shape=(3, 3) 的 float ndarray，或 None。
    """
    if value is None:
        return None
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError(f"{name} must have shape (3, 3), got {matrix.shape}")
    if not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name} contains NaN or Inf")
    return matrix


def validate_target_definition(target: TargetDefinition) -> None:
    """验证 TargetDefinition 的基础 schema。

    要做什么：
    - 检查必填字符串字段；
    - 检查 position / quat_wxyz / rotation_matrix / position_offset 的 shape；
    - 检查数值是否有限；
    - 检查 quat_wxyz 范数是否有效；
    - 检查 rotation_matrix 是否近似正交且 det 接近 1。

    为什么需要：
    A06 生成 target，A04/A05 消费 target。target schema 如果不稳定，后续 IK
    会在 Jacobian、误差计算或 QP 求解时失败，定位成本更高。

    对标 mink：
    mink 中 viewer/mocap target 最终会变成 FrameTask 的目标 pose。本项目不调用
    mink，但 target pose 也必须可安全进入 IK task。

    输入：TargetDefinition。
    输出：无返回；不合法时抛出异常。
    验证：position shape=(3,)，quat_wxyz shape=(4,)，rotation_matrix shape=(3,3)。
    """
    if not isinstance(target, TargetDefinition):
        raise TypeError(f"target must be TargetDefinition, got {type(target)!r}")

    for field_name in ("target_id", "source", "target_site", "coordinate_frame", "orientation_mode"):
        value = getattr(target, field_name)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{field_name} must be a non-empty string")

    if target.target_body is not None and not isinstance(target.target_body, str):
        raise ValueError("target_body must be a string or None")
    if not isinstance(target.metadata, dict):
        raise ValueError("metadata must be a dict")

    for field_name, length in (("position", 3), ("quat_wxyz", 4), ("position_offset", 3)):
        vector = parse_vector_optional(getattr(target, field_name), length, field_name)
        if field_name == "quat_wxyz" and vector is not None:
            if float(np.linalg.norm(vector)) <= 0.0:
                raise ValueError("quat_wxyz norm must be positive")

    matrix = parse_rotation_matrix_optional(target.rotation_matrix)
    if matrix is not None:
        if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1.0e-6):
            raise ValueError("rotation_matrix must be orthonormal within atol=1e-6")
        if not np.isclose(float(np.linalg.det(matrix)), 1.0, atol=1.0e-6):
            raise ValueError("rotation_matrix determinant must be close to 1")


def quat_wxyz_to_rotation_matrix(quat_wxyz: Any) -> np.ndarray:
    """把 wxyz 四元数转换成旋转矩阵。

    配置和 JSON 中统一使用 wxyz。若后续使用 scipy Rotation，需要注意 scipy
    ``Rotation.from_quat`` 默认使用 xyzw。
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

    这是最小稳定实现，覆盖 trace 正负分支。输出会归一化。
    """
    matrix = parse_rotation_matrix_optional(rotation_matrix)
    if matrix is None:
        raise ValueError("rotation_matrix cannot be None")

    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale

    quat = np.asarray([w, x, y, z], dtype=float)
    return quat / np.linalg.norm(quat)


def load_target_definition(path: str | Path) -> TargetDefinition:
    """读取 TargetDefinition JSON。

    输入：JSON path。
    输出：TargetDefinition。
    验证：读取后立即执行 validate_target_definition。
    """
    target_path = Path(path).expanduser()
    if not target_path.exists():
        raise FileNotFoundError(f"TargetDefinition JSON not found: {target_path}")

    data = json.loads(target_path.read_text(encoding="utf-8"))
    target = TargetDefinition(
        target_id=data["target_id"],
        source=data["source"],
        target_site=data["target_site"],
        target_body=data.get("target_body"),
        coordinate_frame=data.get("coordinate_frame", "world"),
        position=parse_vector_optional(data.get("position"), 3, "position"),
        rotation_matrix=parse_rotation_matrix_optional(data.get("rotation_matrix")),
        quat_wxyz=parse_vector_optional(data.get("quat_wxyz"), 4, "quat_wxyz"),
        position_offset=parse_vector_optional(data.get("position_offset"), 3, "position_offset"),
        orientation_mode=data.get("orientation_mode", "keep_current"),
        metadata=data.get("metadata", {}),
    )
    validate_target_definition(target)
    return target


def save_target_definition(target: TargetDefinition, path: str | Path) -> None:
    """保存 TargetDefinition JSON。

    输入：TargetDefinition 和输出路径。
    输出：JSON 文件。
    验证：保存前执行 validate_target_definition，避免写出坏 schema。
    """
    validate_target_definition(target)
    target_path = Path(path)
    target_path.parent.mkdir(parents=True, exist_ok=True)
    payload = _array_to_list(asdict(target))
    target_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def target_from_offset(current_position: Any, current_rotation: Any, offset: Any) -> TargetDefinition:
    """从当前位姿和 position offset 构造 fixed target。

    输入：
    - current_position: shape=(3,)，单位 m；
    - current_rotation: shape=(3,3)；
    - offset: shape=(3,)，单位 m。

    输出：
    - TargetDefinition；
    - position = current_position + offset；
    - orientation_mode = keep_current。

    验证：
    target.position - current_position 应等于 offset。
    """
    current_position_vec = parse_vector_optional(current_position, 3, "current_position")
    offset_vec = parse_vector_optional(offset, 3, "offset")
    rotation = parse_rotation_matrix_optional(current_rotation, "current_rotation")
    if current_position_vec is None or offset_vec is None or rotation is None:
        raise ValueError("current_position, current_rotation and offset cannot be None")

    target = TargetDefinition(
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
    validate_target_definition(target)
    return target


def target_from_fixed_pose(position: Any, orientation_mode: str, rpy: Any = None, quat_wxyz: Any = None) -> TargetDefinition:
    """构造 fixed pose target 的最小实现。

    当前只实现 fixed_quat。keep_current 需要 current rotation，fixed_rpy 需要 RPY
    转 rotation matrix，留到后续步骤。
    """
    position_vec = parse_vector_optional(position, 3, "position")
    if position_vec is None:
        raise ValueError("position cannot be None")

    if orientation_mode == "fixed_quat":
        quat = parse_vector_optional(quat_wxyz, 4, "quat_wxyz")
        target = TargetDefinition(
            target_id="fixed_pose",
            source="fixed_pose",
            target_site="attachment_site",
            coordinate_frame="world",
            position=position_vec,
            rotation_matrix=quat_wxyz_to_rotation_matrix(quat),
            quat_wxyz=quat,
            orientation_mode="fixed_quat",
        )
        validate_target_definition(target)
        return target

    raise NotImplementedError("TODO R2: 补 fixed_rpy / keep_current 的 fixed pose 解析。")


def target_from_pose_sequence(start_pose: TargetDefinition, end_pose: TargetDefinition, num_waypoints: int) -> list[TargetDefinition]:
    """规划 pose sequence target。

    TODO:
    - 输入：start_pose、end_pose、num_waypoints；
    - 输出：TargetDefinition list；
    - 推荐 API：numpy.linspace；
    - 验证：len(result) == num_waypoints。
    """
    raise NotImplementedError("TODO R6/R7: 实现 pose sequence 插值。")


def target_from_mocap_placeholder(model_n_mocap: int, mocap_body_name: str) -> TargetDefinition:
    """构造 mocap target placeholder。

    当前不启动 viewer，只记录未来数据流。model_n_mocap <= 0 时不能访问
    data.mocap_pos[0]。
    """
    if model_n_mocap <= 0:
        raise ValueError("model.nmocap == 0; cannot access data.mocap_pos[0].")

    target = TargetDefinition(
        target_id=mocap_body_name,
        source="mocap_placeholder",
        target_site="attachment_site",
        coordinate_frame="world",
        orientation_mode="mocap_quat_wxyz",
        metadata={"model_n_mocap": model_n_mocap, "mocap_body_name": mocap_body_name},
    )
    validate_target_definition(target)
    return target


def resolve_target_definition(*args: Any, **kwargs: Any) -> TargetDefinition:
    """统一解析 target 来源。

    TODO R2/R3:
    - 根据 source 分发到 offset/fixed/sequence/mocap；
    - 输入配置、CLI 参数、A02 current pose 或 mocap metadata；
    - 输出 TargetDefinition；
    - 验证 target schema 完整。
    """
    raise NotImplementedError("TODO R2/R3: 补 resolve_target_definition 分发逻辑。")
