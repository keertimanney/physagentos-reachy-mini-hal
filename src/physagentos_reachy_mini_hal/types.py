from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Literal

Interpolation = Literal["linear", "minjerk", "ease_in_out", "cartoon"]


class RobotState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    READY = "ready"
    ERROR = "error"


@dataclass(frozen=True)
class HeadPose:
    """6-DOF Stewart-platform target. Translations in meters, rotations in radians.

    The underlying SDK clamps roll/pitch to ~±40° and yaw to ±180°.
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class AntennaTargets:
    """Antenna positions in radians. Antennas double as input buttons when pushed."""

    left: float = 0.0
    right: float = 0.0
