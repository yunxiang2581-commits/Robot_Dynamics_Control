from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from projects.B_mujoco_mpc_study.simulator.scripts import (
    run_B03_ilqr_lite_two_link_smoke as runner,
)


class FakeData:
    def __init__(self) -> None:
        # qpos 模拟 MuJoCo data.qpos，前两个元素就是 two-link 的 q1/q2。
        self.qpos = np.array([0.1, -0.2], dtype=float)
        # qvel 模拟 MuJoCo data.qvel，前两个元素就是 two-link 的 dq1/dq2。
        self.qvel = np.array([0.3, -0.4], dtype=float)
        # time 模拟 MuJoCo 仿真时间，用来测试 snapshot 是否能恢复时间。
        self.time = 1.25
        # ctrl 模拟 actuator 控制缓存，用来测试 rollout 后能否恢复控制输入。
        self.ctrl = np.array([0.0, 0.0], dtype=float)


class FakeTwoLinkEnv:
    def __init__(self) -> None:
        # data 提供 qpos/qvel/time/ctrl，模拟最小 MuJoCo data 接口。
        self.data = FakeData()
        # model 只做占位；fake 测试不需要真实 MuJoCo model。
        self.model = object()
        # last_applied_torque 模拟 TwoLinkEnv.step() 记录的实际执行力矩。
        self.last_applied_torque = (0.0, 0.0)
        # forward_called 用来确认 set/restore 后确实调用过 forward。
        self.forward_called = 0

    def forward(self) -> None:
        # fake forward 不做物理计算，只记录被调用次数。
        self.forward_called += 1


class FakeStepTwoLinkEnv(FakeTwoLinkEnv):
    def __init__(self) -> None:
        super().__init__()
        # step_called 用来确认 dynamics_fn 确实推进了一步临时仿真。
        self.step_called = 0

    def step(self, torque: tuple[float, float]) -> tuple[float, float, float, float]:
        # fake step 使用一个简单的离散系统，便于测试 x_next 是否来自输入 x/u。
        self.step_called += 1
        self.data.ctrl[:] = np.asarray(torque, dtype=float)
        self.last_applied_torque = (float(torque[0]), float(torque[1]))
        self.data.time += 0.01
        self.data.qpos[:] = self.data.qpos + 0.01 * self.data.qvel
        self.data.qvel[:] = self.data.qvel + 0.1 * self.data.ctrl
        return tuple(runner.get_two_link_state(self))


class FakeGetStateDictEnv:
    def get_state(self) -> dict[str, np.ndarray]:
        # 返回 dict 风格状态，用来测试 adapter 对结构化状态 API 的兼容。
        return {
            # q 表示两个关节角。
            "q": np.array([0.5, -0.6], dtype=float),
            # dq 表示两个关节速度。
            "dq": np.array([0.7, -0.8], dtype=float),
        }


def test_get_two_link_state_from_data_qpos_qvel() -> None:
    # Arrange：构造只有 data.qpos/qvel 的 fake two-link 环境。
    env = FakeTwoLinkEnv()

    # Act：调用状态读取 helper。
    state = runner.get_two_link_state(env)

    # Assert：状态必须是一维 4 元向量。
    assert state.shape == (4,)
    # Assert：读取顺序必须是 [q1,q2,dq1,dq2]。
    np.testing.assert_allclose(state, np.array([0.1, -0.2, 0.3, -0.4], dtype=float))
    # Assert：状态必须全是有限值，避免污染后续 finite-difference。
    assert np.isfinite(state).all()


def test_get_two_link_state_from_dict_q_dq() -> None:
    # Arrange：构造 get_state() 返回 dict 的环境。
    env = FakeGetStateDictEnv()

    # Act：读取 dict 风格状态。
    state = runner.get_two_link_state(env)

    # Assert：adapter 应把 q/dq 拼成统一的 [q1,q2,dq1,dq2]。
    np.testing.assert_allclose(state, np.array([0.5, -0.6, 0.7, -0.8], dtype=float))


def test_set_two_link_state_updates_qpos_qvel() -> None:
    # Arrange：构造 fake 环境和目标状态。
    env = FakeTwoLinkEnv()
    # target_state 对应 [q1,q2,dq1,dq2]。
    target_state = np.array([0.9, -1.0, 1.1, -1.2], dtype=float)

    # Act：把 fake env 写入目标状态。
    runner.set_two_link_state(env, target_state)
    # Act：再用统一读取 helper 读回来。
    state = runner.get_two_link_state(env)

    # Assert：读回来的状态应和写入状态完全一致。
    np.testing.assert_allclose(state, target_state)
    # Assert：写 qpos/qvel 后应调用 forward 刷新派生量。
    assert env.forward_called >= 1


def test_save_snapshot_is_copy_not_view() -> None:
    # Arrange：构造 fake 环境。
    env = FakeTwoLinkEnv()

    # Act：保存当前快照。
    snapshot = runner.save_two_link_env_state(env)
    # Act：故意修改 env 内部数组，模拟 rollout 污染环境。
    env.data.qpos[:] = [10.0, 20.0]
    # Act：同时修改速度数组。
    env.data.qvel[:] = [30.0, 40.0]

    # Assert：snapshot["state"] 不能被 env 后续变化污染。
    np.testing.assert_allclose(snapshot["state"], np.array([0.1, -0.2, 0.3, -0.4], dtype=float))
    # Assert：snapshot["qpos"] 必须是 copy，不是 data.qpos 的 view。
    np.testing.assert_allclose(snapshot["qpos"], np.array([0.1, -0.2], dtype=float))
    # Assert：snapshot["qvel"] 也必须是 copy。
    np.testing.assert_allclose(snapshot["qvel"], np.array([0.3, -0.4], dtype=float))


def test_restore_two_link_env_state() -> None:
    # Arrange：构造 fake 环境。
    env = FakeTwoLinkEnv()
    # Arrange：先保存初始状态快照。
    snapshot = runner.save_two_link_env_state(env)

    # Act：故意把 qpos 改成错误值。
    env.data.qpos[:] = [9.0, 8.0]
    # Act：故意把 qvel 改成错误值。
    env.data.qvel[:] = [7.0, 6.0]
    # Act：故意改变仿真时间，模拟临时 rollout 推进了时间。
    env.data.time = 99.0
    # Act：故意改变控制缓存，模拟临时 rollout 写入了 torque。
    env.data.ctrl[:] = [5.0, 4.0]
    # Act：恢复之前保存的 snapshot。
    runner.restore_two_link_env_state(env, snapshot)

    # Assert：恢复后标准 state 应回到初始值。
    state = runner.get_two_link_state(env)
    np.testing.assert_allclose(state, np.array([0.1, -0.2, 0.3, -0.4], dtype=float))
    # Assert：time 也应恢复，防止 rollout 污染真实闭环时间。
    assert env.data.time == pytest.approx(1.25)
    # Assert：ctrl 应恢复，防止 rollout 污染真实 actuator 缓存。
    np.testing.assert_allclose(env.data.ctrl, np.array([0.0, 0.0], dtype=float))
    # Assert：恢复 qpos/qvel 后应调用 forward 刷新派生量。
    assert env.forward_called >= 1


def test_set_state_rejects_wrong_shape() -> None:
    # Arrange：构造 fake 环境。
    env = FakeTwoLinkEnv()

    # Assert：3 维状态缺少一个分量，必须拒绝。
    with pytest.raises(ValueError):
        runner.set_two_link_state(env, np.zeros(3, dtype=float))

    # Assert：5 维状态多出一个分量，也必须拒绝。
    with pytest.raises(ValueError):
        runner.set_two_link_state(env, np.zeros(5, dtype=float))


def test_set_state_rejects_nonfinite() -> None:
    # Arrange：构造 fake 环境。
    env = FakeTwoLinkEnv()

    # Assert：NaN 写入 env 会污染 rollout，必须拒绝。
    with pytest.raises(ValueError):
        runner.set_two_link_state(env, np.array([0.0, np.nan, 0.0, 0.0], dtype=float))

    # Assert：inf 也不能进入状态。
    with pytest.raises(ValueError):
        runner.set_two_link_state(env, np.array([0.0, 0.0, np.inf, 0.0], dtype=float))


def test_restore_rejects_invalid_snapshot() -> None:
    # Arrange：构造 fake 环境。
    env = FakeTwoLinkEnv()

    # Assert：snapshot 必须是 dict，None 应被拒绝。
    with pytest.raises(ValueError):
        runner.restore_two_link_env_state(env, snapshot=None)

    # Assert：空 dict 缺少 "state"，不是合法快照。
    with pytest.raises(ValueError):
        runner.restore_two_link_env_state(env, snapshot={})


def test_build_two_link_dynamics_fn_steps_without_polluting_env_state() -> None:
    # Arrange：构造带 step() 的 fake 环境，并保存真实闭环状态。
    env = FakeStepTwoLinkEnv()
    original_snapshot = runner.save_two_link_env_state(env)
    x = np.array([0.5, -0.6, 0.7, -0.8], dtype=float)
    u = np.array([1.2, -1.5], dtype=float)

    # Act：构造 one-step dynamics_fn，并在临时状态 x 上执行控制 u。
    dynamics_fn = runner.build_two_link_dynamics_fn(env)
    x_next = dynamics_fn(x, u)

    # Assert：x_next 来自 fake dynamics 的一步推进。
    expected_next = np.array(
        [
            0.5 + 0.01 * 0.7,
            -0.6 + 0.01 * -0.8,
            0.7 + 0.1 * 1.2,
            -0.8 + 0.1 * -1.5,
        ],
        dtype=float,
    )
    np.testing.assert_allclose(x_next, expected_next)
    assert env.step_called == 1

    # Assert：临时 rollout 后，真实 env 的 state/time/ctrl 必须恢复。
    restored_snapshot = runner.save_two_link_env_state(env)
    np.testing.assert_allclose(restored_snapshot["state"], original_snapshot["state"])
    np.testing.assert_allclose(restored_snapshot["qpos"], original_snapshot["qpos"])
    np.testing.assert_allclose(restored_snapshot["qvel"], original_snapshot["qvel"])
    np.testing.assert_allclose(restored_snapshot["ctrl"], original_snapshot["ctrl"])
    assert restored_snapshot["time"] == pytest.approx(original_snapshot["time"])
    assert restored_snapshot["last_applied_torque"] == original_snapshot["last_applied_torque"]
