from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from physagentos_reachy_mini_hal import (
    AntennaTargets,
    HeadPose,
    ReachyMiniDriver,
    RobotHAL,
    RobotState,
)


class FakeReachyMini:
    """Minimal stand-in for `reachy_mini.ReachyMini` covering the calls the driver makes."""

    def __init__(self) -> None:
        self.goto_calls: list[tuple[dict[str, float], float, str | None]] = []
        self.set_calls: list[dict[str, float]] = []
        self.played_moves: list[Any] = []
        self.entered = False
        self.exited = False
        self._pose = {"x": 0.1, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.2,
                      "body_rotation": 0.5}
        self._buttons = {"left_antenna": False, "right_antenna": True}

    def __enter__(self):
        self.entered = True
        return self

    def __exit__(self, *_):
        self.exited = True

    def goto_target(self, target, duration, interpolation=None):
        self.goto_calls.append((dict(target), duration, interpolation))

    def set_target(self, target):
        self.set_calls.append(dict(target))

    def get_current_pose(self):
        return dict(self._pose)

    def is_button_pressed(self, name: str) -> bool:
        return self._buttons[name]

    def get_camera_frame(self):
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def play_move(self, move, initial_goto_duration):
        self.played_moves.append((move, initial_goto_duration))


@pytest.fixture
def fake_sdk() -> FakeReachyMini:
    return FakeReachyMini()


@pytest.fixture
def driver(fake_sdk: FakeReachyMini) -> ReachyMiniDriver:
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


def test_head_goto_forwards_pose(driver: ReachyMiniDriver, fake_sdk: FakeReachyMini) -> None:
    driver.head_goto(HeadPose(pitch=0.3, yaw=0.5), duration=1.0, interpolation="minjerk")
    target, duration, interp = fake_sdk.goto_calls[-1]
    assert duration == 1.0
    assert interp == "minjerk"
    assert target["pitch"] == pytest.approx(0.3)
    assert target["yaw"] == pytest.approx(0.5)


def test_head_set_realtime(driver: ReachyMiniDriver, fake_sdk: FakeReachyMini) -> None:
    driver.head_set(HeadPose(roll=0.1))
    assert fake_sdk.set_calls[-1]["roll"] == pytest.approx(0.1)


def test_head_read_round_trip(driver: ReachyMiniDriver) -> None:
    pose = driver.head_read()
    assert pose.x == pytest.approx(0.1)
    assert pose.yaw == pytest.approx(0.2)


def test_body_motion(driver: ReachyMiniDriver, fake_sdk: FakeReachyMini) -> None:
    driver.body_goto(1.0, duration=0.5)
    assert fake_sdk.goto_calls[-1][0] == {"body_rotation": 1.0}
    driver.body_set(0.7)
    assert fake_sdk.set_calls[-1] == {"body_rotation": 0.7}
    assert driver.body_read() == pytest.approx(0.5)


def test_antennas(driver: ReachyMiniDriver, fake_sdk: FakeReachyMini) -> None:
    driver.antennas_goto(AntennaTargets(left=0.2, right=-0.2), duration=0.4)
    target = fake_sdk.goto_calls[-1][0]
    assert target == {"left_antenna": 0.2, "right_antenna": -0.2}


def test_button_pressed(driver: ReachyMiniDriver) -> None:
    assert driver.button_pressed("left") is False
    assert driver.button_pressed("right") is True


def test_button_pressed_invalid_side(driver: ReachyMiniDriver) -> None:
    with pytest.raises(ValueError):
        driver.button_pressed("middle")


def test_camera_frame_returns_ndarray(driver: ReachyMiniDriver) -> None:
    frame = driver.read_camera_frame()
    assert isinstance(frame, np.ndarray)
    assert frame.shape == (480, 640, 3)


def test_calls_before_connect_raise() -> None:
    d = ReachyMiniDriver(sdk_factory=lambda: FakeReachyMini())
    with pytest.raises(RuntimeError):
        d.head_set(HeadPose())
