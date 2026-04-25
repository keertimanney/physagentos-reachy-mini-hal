from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import numpy as np
import pytest

from physagentos_reachy_mini_hal import (
    AntennaTargets,
    HeadPose,
    ReachyMiniDriver,
    RobotHAL,
    RobotState,
)


class FakeMedia:
    def __init__(self) -> None:
        self.frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.get_frame_calls = 0

    def get_frame(self) -> np.ndarray:
        self.get_frame_calls += 1
        return self.frame


class FakeReachyMini:
    """Stand-in covering the calls the driver makes against `reachy_mini==1.5.1`."""

    def __init__(self) -> None:
        self.goto_calls: list[dict[str, Any]] = []
        self.head_set_calls: list[np.ndarray] = []
        self.body_yaw_set_calls: list[float] = []
        self.antenna_set_calls: list[list[float]] = []
        self.played_moves: list[Any] = []
        self.wake_up_calls = 0
        self.goto_sleep_calls = 0
        self.entered = False
        self.exited = False
        self._head_pose = np.eye(4)
        self._head_pose[0, 3] = 0.05
        self._antennas = [0.3, -0.2]  # [right, left]
        self.media = FakeMedia()

    def __enter__(self):
        self.entered = True
        return self

    def __exit__(self, *_):
        self.exited = True

    def goto_target(self, head=None, antennas=None, duration=0.5, method=None, body_yaw=None):
        self.goto_calls.append(
            {"head": head, "antennas": antennas, "duration": duration,
             "method": method, "body_yaw": body_yaw}
        )

    def set_target_head_pose(self, pose):
        self.head_set_calls.append(np.asarray(pose))

    def set_target_body_yaw(self, yaw):
        self.body_yaw_set_calls.append(yaw)

    def set_target_antenna_joint_positions(self, antennas):
        self.antenna_set_calls.append(list(antennas))

    def get_current_head_pose(self):
        return self._head_pose.copy()

    def get_present_antenna_joint_positions(self):
        return list(self._antennas)

    def wake_up(self):
        self.wake_up_calls += 1

    def goto_sleep(self):
        self.goto_sleep_calls += 1


@pytest.fixture
def fake_sdk() -> FakeReachyMini:
    return FakeReachyMini()


@pytest.fixture
def driver(fake_sdk: FakeReachyMini):
    d = ReachyMiniDriver(sdk_factory=lambda: fake_sdk)
    d.connect()
    yield d
    d.disconnect()


def test_driver_implements_robothal_protocol(driver: ReachyMiniDriver) -> None:
    assert isinstance(driver, RobotHAL)


def test_lifecycle_transitions(fake_sdk: FakeReachyMini) -> None:
    d = ReachyMiniDriver(sdk_factory=lambda: fake_sdk)
    assert d.state() is RobotState.DISCONNECTED
    d.connect()
    assert d.state() is RobotState.READY
    assert fake_sdk.entered
    d.disconnect()
    assert d.state() is RobotState.DISCONNECTED
    assert fake_sdk.exited


def test_context_manager(fake_sdk: FakeReachyMini) -> None:
    with ReachyMiniDriver(sdk_factory=lambda: fake_sdk) as d:
        assert d.state() is RobotState.READY
    assert fake_sdk.exited


def test_head_goto_sends_4x4_matrix(driver: ReachyMiniDriver, fake_sdk: FakeReachyMini) -> None:
    driver.head_goto(HeadPose(pitch=0.3, yaw=0.5), duration=1.0, interpolation="minjerk")
    call = fake_sdk.goto_calls[-1]
    assert call["duration"] == 1.0
    assert call["method"] is not None
    assert getattr(call["method"], "value", call["method"]) == "minjerk"
    head = call["head"]
    assert head is not None
    assert head.shape == (4, 4)
    assert head[3, 3] == pytest.approx(1.0)


def test_head_set_uses_dedicated_method(driver: ReachyMiniDriver, fake_sdk: FakeReachyMini) -> None:
    driver.head_set(HeadPose(roll=0.1))
    assert len(fake_sdk.head_set_calls) == 1
    assert fake_sdk.head_set_calls[0].shape == (4, 4)


def test_head_read_returns_ndarray(driver: ReachyMiniDriver) -> None:
    pose = driver.head_read()
    assert isinstance(pose, np.ndarray)
    assert pose.shape == (4, 4)
    assert pose[0, 3] == pytest.approx(0.05)


def test_body_motion(driver: ReachyMiniDriver, fake_sdk: FakeReachyMini) -> None:
    driver.body_goto(0.7, duration=0.5)
    assert fake_sdk.goto_calls[-1]["body_yaw"] == pytest.approx(0.7)
    driver.body_set(0.4)
    assert fake_sdk.body_yaw_set_calls == [0.4]


def test_antennas_use_right_left_wire_order(driver: ReachyMiniDriver, fake_sdk: FakeReachyMini) -> None:
    driver.antennas_set(AntennaTargets(left=0.1, right=0.9))
    # Wire format is [right, left]
    assert fake_sdk.antenna_set_calls[-1] == [0.9, 0.1]


def test_antennas_goto(driver: ReachyMiniDriver, fake_sdk: FakeReachyMini) -> None:
    driver.antennas_goto(AntennaTargets(left=0.2, right=-0.2), duration=0.4)
    call = fake_sdk.goto_calls[-1]
    assert call["antennas"] == [-0.2, 0.2]
    assert call["duration"] == 0.4


def test_antennas_read_decodes_wire_order(driver: ReachyMiniDriver) -> None:
    # FakeReachyMini._antennas = [0.3, -0.2] which is [right, left]
    targets = driver.antennas_read()
    assert targets.right == pytest.approx(0.3)
    assert targets.left == pytest.approx(-0.2)


def test_camera_frame_returns_ndarray(driver: ReachyMiniDriver, fake_sdk: FakeReachyMini) -> None:
    frame = driver.read_camera_frame()
    assert isinstance(frame, np.ndarray)
    assert frame.shape == (480, 640, 3)
    assert fake_sdk.media.get_frame_calls == 1


def test_calls_before_connect_raise() -> None:
    d = ReachyMiniDriver(sdk_factory=lambda: FakeReachyMini())
    with pytest.raises(RuntimeError):
        d.head_set(HeadPose())


def test_wake_up_and_sleep(driver: ReachyMiniDriver, fake_sdk: FakeReachyMini) -> None:
    driver.wake_up()
    assert fake_sdk.wake_up_calls == 1
    driver.sleep()
    assert fake_sdk.goto_sleep_calls == 1


def test_head_pose_to_matrix_is_homogeneous() -> None:
    mat = HeadPose(x=0.01, pitch=0.2).to_matrix()
    assert mat.shape == (4, 4)
    assert mat[3, 3] == pytest.approx(1.0)
    assert mat[0, 3] == pytest.approx(0.01)


def test_antenna_targets_round_trip() -> None:
    a = AntennaTargets(left=0.1, right=0.9)
    assert a.to_sdk_list() == [0.9, 0.1]
    b = AntennaTargets.from_sdk_list([0.9, 0.1])
    assert b.left == pytest.approx(0.1)
    assert b.right == pytest.approx(0.9)
