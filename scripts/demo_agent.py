"""physagentOS-style agent demo: a portable agent function consumes RobotHAL.

The point of this script is the `run(robot: RobotHAL, ...)` signature — any
future driver that implements the same Protocol can drop in unchanged. This
particular agent is a small perception+action loop:

  1. Wake up.
  2. For N cycles: read a camera frame, compute mean brightness, tilt the
     head proportional to brightness, and flick antennas as a heartbeat.
  3. Park the robot in the relaxed sleep posture.

If the camera is busy (e.g. owned by the desktop control app), perception
gracefully degrades to a constant 0.5 brightness so the motion loop still
demonstrates the HAL surface end-to-end.

Run:
    python scripts/demo_agent.py
"""

from __future__ import annotations

import sys
import time

import numpy as np

from physagentos_reachy_mini_hal import (
    AntennaTargets,
    HeadPose,
    RobotHAL,
    connect_with_fallback,
)


def perceive_brightness(frame: np.ndarray) -> float:
    """Mean luminance of `frame` in [0, 1]. Returns 0.5 for an empty/object frame."""
    if frame.size == 0 or frame.dtype == object:
        return 0.5
    gray = frame.mean(axis=-1) if frame.ndim == 3 else frame
    return float(np.clip(gray.mean() / 255.0, 0.0, 1.0))


def run(robot: RobotHAL, n_cycles: int = 5, period: float = 3.5) -> None:
    """Run the demo agent loop against any `RobotHAL` implementation."""
    print("agent: waking up", flush=True)
    robot.wake_up()
    time.sleep(2.0)

    for i in range(n_cycles):
        try:
            frame = robot.read_camera_frame()
            brightness = perceive_brightness(frame)
            source = "camera"
        except Exception as e:
            brightness = 0.5
            source = f"fallback ({type(e).__name__})"

        # Map brightness in [0, 1] to a head pitch in [-0.15, +0.15] rad
        # (~8.6 degrees each way), so a bright scene looks down and a dark
        # scene looks up.
        pitch = float(np.clip((brightness - 0.5) * 0.3, -0.15, 0.15))
        print(
            f"agent: cycle {i + 1}/{n_cycles}: brightness={brightness:.3f} "
            f"({source}) -> pitch={pitch:+.3f}",
            flush=True,
        )

        robot.head_goto(HeadPose(pitch=pitch), duration=period * 0.5)
        robot.antennas_goto(
            AntennaTargets(left=0.15, right=-0.15), duration=period * 0.15
        )
        robot.antennas_goto(AntennaTargets(), duration=period * 0.15)

    print("agent: returning to neutral and parking", flush=True)
    robot.head_goto(HeadPose(), duration=2.0)
    robot.sleep()
    time.sleep(2.0)


def main() -> int:
    print("connecting...", flush=True)
    driver = connect_with_fallback()
    try:
        run(driver)
        return 0
    finally:
        driver.disconnect()


if __name__ == "__main__":
    sys.exit(main())
