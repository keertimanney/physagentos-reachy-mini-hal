from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from physagentos_reachy_mini_hal.types import (
    AntennaTargets,
    HeadPose,
    Interpolation,
    RobotState,
)

if TYPE_CHECKING:
    from reachy_mini import ReachyMini


class ReachyMiniDriver:
    """`RobotHAL` implementation backed by the Pollen Robotics `reachy_mini` SDK.

    Wraps `reachy_mini.ReachyMini` so the SDK's safety clamps and Stewart-platform
    inverse kinematics carry through to physagentOS agents. The driver is itself a
    context manager and forwards lifecycle to the SDK.
    """

    def __init__(self, sdk_factory: Any | None = None) -> None:
        self._sdk_factory = sdk_factory
        self._mini: ReachyMini | None = None
        self._state: RobotState = RobotState.DISCONNECTED

    # --- Lifecycle ---------------------------------------------------------

    def connect(self) -> None:
        if self._mini is not None:
            return
        self._state = RobotState.CONNECTING
        try:
            if self._sdk_factory is not None:
                self._mini = self._sdk_factory()
            else:
                from reachy_mini import ReachyMini

                self._mini = ReachyMini()
            if hasattr(self._mini, "__enter__"):
                self._mini.__enter__()
            self._state = RobotState.READY
        except Exception:
            self._state = RobotState.ERROR
            raise

    def disconnect(self) -> None:
        if self._mini is None:
            return
        try:
            if hasattr(self._mini, "__exit__"):
                self._mini.__exit__(None, None, None)
        finally:
            self._mini = None
            self._state = RobotState.DISCONNECTED

    def state(self) -> RobotState:
        return self._state

    def __enter__(self) -> ReachyMiniDriver:
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    # --- Head --------------------------------------------------------------

    def head_goto(
        self,
        pose: HeadPose,
        duration: float,
        interpolation: Interpolation = "minjerk",
    ) -> None:
        target = self._head_pose_to_sdk(pose)
        self._sdk.goto_target(target, duration=duration, interpolation=interpolation)

    def head_set(self, pose: HeadPose) -> None:
        target = self._head_pose_to_sdk(pose)
        self._sdk.set_target(target)

    def head_read(self) -> HeadPose:
        current = self._sdk.get_current_pose()
        return HeadPose(
            x=float(current.get("x", 0.0)),
            y=float(current.get("y", 0.0)),
            z=float(current.get("z", 0.0)),
            roll=float(current.get("roll", 0.0)),
            pitch=float(current.get("pitch", 0.0)),
            yaw=float(current.get("yaw", 0.0)),
        )

    # --- Body --------------------------------------------------------------

    def body_goto(self, yaw: float, duration: float) -> None:
        self._sdk.goto_target({"body_rotation": yaw}, duration=duration)

    def body_set(self, yaw: float) -> None:
        self._sdk.set_target({"body_rotation": yaw})

    def body_read(self) -> float:
        return float(self._sdk.get_current_pose().get("body_rotation", 0.0))

    # --- Antennas ----------------------------------------------------------

    def antennas_goto(self, targets: AntennaTargets, duration: float) -> None:
        self._sdk.goto_target(
            {"left_antenna": targets.left, "right_antenna": targets.right},
            duration=duration,
        )

    def antennas_set(self, targets: AntennaTargets) -> None:
        self._sdk.set_target(
            {"left_antenna": targets.left, "right_antenna": targets.right}
        )

    def button_pressed(self, side: str) -> bool:
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")
        return bool(self._sdk.is_button_pressed(f"{side}_antenna"))

    # --- Vision ------------------------------------------------------------

    def read_camera_frame(self) -> np.ndarray:
        frame = self._sdk.get_camera_frame()
        return np.asarray(frame)

    # --- Recorded moves ----------------------------------------------------

    def play_move(self, name: str, library: str, initial_goto_duration: float = 1.0) -> None:
        from reachy_mini.motion.recorded_move import RecordedMoves

        moves = RecordedMoves(library)
        self._sdk.play_move(moves.get(name), initial_goto_duration=initial_goto_duration)

    # --- Internal ----------------------------------------------------------

    @property
    def _sdk(self) -> ReachyMini:
        if self._mini is None:
            raise RuntimeError("Driver is not connected. Call connect() or use as context manager.")
        return self._mini

    @staticmethod
    def _head_pose_to_sdk(pose: HeadPose) -> dict[str, float]:
        return {
            "x": pose.x,
            "y": pose.y,
            "z": pose.z,
            "roll": pose.roll,
            "pitch": pose.pitch,
            "yaw": pose.yaw,
        }
