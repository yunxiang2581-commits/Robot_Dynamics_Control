# 学习副本说明:
# - 原始文件来自 external/mink_upstream/src/mink/lie/utils.py。
# - 本文件保存 Lie group 计算用到的小工具函数。
# - 阅读重点: 第一轮不用深挖，只在读 SO3/SE3 时按需回来看。

import numpy as np


def get_epsilon(dtype: np.dtype) -> float:
    return {
        np.dtype("float32"): 1e-5,
        np.dtype("float64"): 1e-10,
    }[dtype]


def skew(x: np.ndarray) -> np.ndarray:
    assert x.shape == (3,)
    wx, wy, wz = x
    return np.array(
        [
            [0.0, -wz, wy],
            [wz, 0.0, -wx],
            [-wy, wx, 0.0],
        ],
        dtype=x.dtype,
    )
