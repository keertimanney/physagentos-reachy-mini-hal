from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Literal

import numpy as np

Interpolation = Literal["linear", "minjerk", "ease_in_out", "cartoon"]


class RobotState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    READY = "ready"
    ERROR = "error"


@dataclass(frozen=True)
class HeadPose:
    """6-DOF Stewart-platform target. Translations in meters, rotations in radians.

    The SDK clamps roll/pitch to ~±40° and yaw to ±180°. Internally converted
    to a 4x4 homogeneous transform via `reachy_mini.utils.create_head_pose`.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def to_matrix(self) -> np.ndarray:
        from reachy_mini.utils import create_head_pose

        return create_head_pose(
            x=self.x, y=self.y, z=self.z,
            roll=self.roll, pitch=self.pitch, yaw=self.yaw,
            mm=False, degrees=False,
        )


@dataclass(frozen=True)
class AntennaTargets:
    """Antenna joint positions in radians.

    The SDK wire format is `[right, left]` (per `reachy_mini.io.protocol`); this
    dataclass is named-by-side so callers don't have to remember ordering.
    """

    left: float = 0.0
    right: float = 0.0

    def to_sdk_list(self) -> list[float]:
        return [self.right, self.left]

    @classmethod
    def from_sdk_list(cls, antennas: list[float]) -> AntennaTargets:
        if len(antennas) != 2:
            raise ValueError(f"expected length-2 antenna list, got {antennas!r}")
        return cls(left=antennas[1], right=antennas[0])
