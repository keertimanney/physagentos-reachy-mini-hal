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

    Tested against `reachy_mini==1.5.1`. Antenna list ordering on the wire is
    `[right, left]` per `reachy_mini.io.protocol`; the `AntennaTargets` dataclass
    hides this so callers address antennas by side.
    """

    def __init__(
        self,
        sdk_factory: Any | None = None,
        *,
        spawn_daemon: bool = False,
        connection_mode: str = "auto",
        timeout: float = 5.0,
    ) -> None:
        self._sdk_factory = sdk_factory
        self._spawn_daemon = spawn_daemon
        self._connection_mode = connection_mode
        self._timeout = timeout
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

                self._mini = ReachyMini(
                    spawn_daemon=self._spawn_daemon,
                    connection_mode=self._connection_mode,
                    timeout=self._timeout,
                )
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

    def wake_up(self) -> None:
        self._sdk.wake_up()

    def sleep(self) -> None:
        self._sdk.goto_sleep()

    # --- Head --------------------------------------------------------------

    def head_goto(
        self,
        pose: HeadPose,
        duration: float,
        interpolation: Interpolation = "minjerk",
    ) -> None:
        method = self._interp_to_enum(interpolation)
        self._sdk.goto_target(head=pose.to_matrix(), duration=duration, method=method)

    def head_set(self, pose: HeadPose) -> None:
        self._sdk.set_target_head_pose(pose.to_matrix())

    def head_read(self) -> np.ndarray:
        return np.asarray(self._sdk.get_current_head_pose())

    # --- Body --------------------------------------------------------------

    def body_goto(self, yaw: float, duration: float) -> None:
        self._sdk.goto_target(body_yaw=yaw, duration=duration)

    def body_set(self, yaw: float) -> None:
        self._sdk.set_target_body_yaw(yaw)

    # --- Antennas ----------------------------------------------------------

    def antennas_goto(self, targets: AntennaTargets, duration: float) -> None:
        self._sdk.goto_target(antennas=targets.to_sdk_list(), duration=duration)

    def antennas_set(self, targets: AntennaTargets) -> None:
        self._sdk.set_target_antenna_joint_positions(targets.to_sdk_list())

    def antennas_read(self) -> AntennaTargets:
        return AntennaTargets.from_sdk_list(list(self._sdk.get_present_antenna_joint_positions()))

    # --- Vision ------------------------------------------------------------

    def read_camera_frame(self) -> np.ndarray:
        frame = self._sdk.media.get_frame()
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
    def _interp_to_enum(interpolation: Interpolation) -> Any:
        from reachy_mini.utils.interpolation import InterpolationTechnique

        return InterpolationTechnique(interpolation)
