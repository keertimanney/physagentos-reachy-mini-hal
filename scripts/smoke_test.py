"""Smoke test against a real Reachy Mini Lite (USB-direct).

Exercises every method of `ReachyMiniDriver` with low-amplitude motion
(<= 0.15 rad head/body, <= 0.2 rad antennas; 3.0s per move with 0.8s settles)
and returns the robot to neutral between steps. Saves one camera frame to
`scripts/smoke_frame.png` for visual verification.

Auto-detects whether a daemon is already running (e.g. the desktop control
app on :8000) and only spawns its own if needed.

Run:
    python scripts/smoke_test.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

from physagentos_reachy_mini_hal import (
    AntennaTargets,
    HeadPose,
    RobotState,
    connect_with_fallback,
)

# Tuned-down motion parameters
HEAD_AMPL = 0.15      # rad (~8.6 deg)
BODY_AMPL = 0.15      # rad (~8.6 deg)
ANTENNA_AMPL = 0.20   # rad (~11.5 deg)
DURATION = 3.0        # seconds per move
SETTLE = 0.8          # seconds between moves


def section(title: str) -> None:
    print(f"\n=== {title} ===", flush=True)


def main() -> int:
    section("Connecting (try existing daemon, fall back to spawning)")
    try:
        driver = connect_with_fallback()
    except Exception as e:
        print(f"FAILED to connect: {e!r}", flush=True)
        return 1

    print(f"state: {driver.state().value}", flush=True)
    if driver.state() is not RobotState.READY:
        driver.disconnect()
        return 1

    try:
        section("Read current head pose (no motion)")
        pose_mat = driver.head_read()
        print(f"translation (x,y,z): {pose_mat[:3, 3]}", flush=True)
        print(f"rotation diag:       {np.diag(pose_mat[:3, :3])}", flush=True)

        section("Read current antenna positions (no motion)")
        antennas = driver.antennas_read()
        print(f"left={antennas.left:.4f} rad, right={antennas.right:.4f} rad", flush=True)

        section("Grab one camera frame")
        try:
            frame = driver.read_camera_frame()
            print(f"frame shape: {frame.shape}, dtype: {frame.dtype}", flush=True)
            if frame.ndim >= 2 and frame.dtype != object:
                try:
                    import cv2
                    out = Path(__file__).parent / "smoke_frame.png"
                    cv2.imwrite(str(out), frame)
                    print(f"saved frame to: {out}", flush=True)
                except ImportError:
                    np.save(Path(__file__).parent / "smoke_frame.npy", frame)
                    print("opencv not available; saved as smoke_frame.npy", flush=True)
            else:
                print("frame is empty/object array (camera owned by another process?)",
                      flush=True)
        except Exception as e:
            print(f"camera frame FAILED (non-fatal): {e!r}", flush=True)

        section(f"Settle to neutral pose ({DURATION}s)")
        driver.head_goto(HeadPose(), duration=DURATION)
        driver.body_goto(0.0, duration=DURATION)
        driver.antennas_goto(AntennaTargets(), duration=DURATION)
        time.sleep(SETTLE)

        section(f"Head pitch +{HEAD_AMPL} rad over {DURATION}s, then back")
        driver.head_goto(HeadPose(pitch=HEAD_AMPL), duration=DURATION)
        time.sleep(SETTLE)
        driver.head_goto(HeadPose(), duration=DURATION)
        time.sleep(SETTLE)

        section(f"Head yaw +{HEAD_AMPL} rad over {DURATION}s, then back")
        driver.head_goto(HeadPose(yaw=HEAD_AMPL), duration=DURATION)
        time.sleep(SETTLE)
        driver.head_goto(HeadPose(), duration=DURATION)
        time.sleep(SETTLE)

        section(f"Body yaw +{BODY_AMPL} -> 0 -> -{BODY_AMPL} -> 0 ({DURATION}s each)")
        driver.body_goto(BODY_AMPL, duration=DURATION)
        time.sleep(SETTLE)
        driver.body_goto(0.0, duration=DURATION)
        time.sleep(SETTLE)
        driver.body_goto(-BODY_AMPL, duration=DURATION)
        time.sleep(SETTLE)
        driver.body_goto(0.0, duration=DURATION)
        time.sleep(SETTLE)

        section(f"Antennas left+{ANTENNA_AMPL}, right-{ANTENNA_AMPL}, then 0 ({DURATION}s)")
        driver.antennas_goto(
            AntennaTargets(left=ANTENNA_AMPL, right=-ANTENNA_AMPL), duration=DURATION
        )
        time.sleep(SETTLE)
        driver.antennas_goto(AntennaTargets(), duration=DURATION)
        time.sleep(SETTLE)

        section("Read pose + antennas after motion")
        final = driver.head_read()
        print(f"final head translation: {final[:3, 3]}", flush=True)
        print(f"final rotation diag:    {np.diag(final[:3, :3])}", flush=True)
        a = driver.antennas_read()
        print(f"final antennas: left={a.left:.4f}, right={a.right:.4f}", flush=True)

        section("Park: goto_sleep() to a relaxed posture")
        driver.sleep()
        time.sleep(2.0)

        section("Smoke test PASSED")
        return 0
    finally:
        section("Disconnecting")
        driver.disconnect()
        print(f"final state: {driver.state().value}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
