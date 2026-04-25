from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np

from physagentos_reachy_mini_hal.types import (
    AntennaTargets,
    HeadPose,
    Interpolation,
    RobotState,
)


@runtime_checkable
class RobotHAL(Protocol):
    """physagentOS hardware abstraction for a tabletop humanoid robot.

    Drivers wrap a specific robot SDK behind this Protocol. Agents written against
    `RobotHAL` stay portable across hardware. All motion methods are synchronous;
    use `*_goto` for smooth interpolated motion (>=0.5s) and `*_set` for realtime
    control loops (>=10Hz).
    """

    # --- Lifecycle ---------------------------------------------------------

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def state(self) -> RobotState: ...

    def __enter__(self) -> RobotHAL: ...
    def __exit__(self, exc_type, exc, tb) -> None: ...

    def wake_up(self) -> None: ...
    """Move to initial head position and play the wake-up animation."""

    def sleep(self) -> None: ...
    """Move head + antennas to a predefined sleep/park posture."""

    # --- Head (6-DOF Stewart platform) -------------------------------------

    def head_goto(
        self,
        pose: HeadPose,
        duration: float,
        interpolation: Interpolation = "minjerk",
    ) -> None: ...

    def head_set(self, pose: HeadPose) -> None: ...

    def head_read(self) -> np.ndarray: ...
    """Returns the current head pose as a 4x4 homogeneous transform."""

    # --- Body rotation -----------------------------------------------------

    def body_goto(self, yaw: float, duration: float) -> None: ...
    def body_set(self, yaw: float) -> None: ...

    # --- Antennas ----------------------------------------------------------

    def antennas_goto(self, targets: AntennaTargets, duration: float) -> None: ...
    def antennas_set(self, targets: AntennaTargets) -> None: ...
    def antennas_read(self) -> AntennaTargets: ...

    # --- Vision ------------------------------------------------------------

    def read_camera_frame(self) -> np.ndarray: ...

    # --- Recorded moves ----------------------------------------------------

    def play_move(self, name: str, library: str, initial_goto_duration: float = 1.0) -> None: ...
