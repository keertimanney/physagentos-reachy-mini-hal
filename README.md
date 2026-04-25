# physagentos-reachy-mini-hal

A [physagentOS](https://github.com/keertimanney/physagentos) Hardware Abstraction Layer (HAL) driver for the [Pollen Robotics Reachy Mini](https://github.com/pollen-robotics/reachy_mini).

This package wraps the official `reachy_mini` Python SDK behind a stable, robot-agnostic HAL `Protocol` that physagentOS can target. Multiple robots can implement the same interface; agents written against the HAL stay portable.

## Status

Alpha — scaffolding stage. The HAL `Protocol` is defined and the Reachy Mini driver covers head, body, antennas, camera, and recorded-move playback. Audio I/O is stubbed.

## Install

```bash
pip install -e ".[dev]"
```

The `reachy_mini` SDK pulls in motor-controller and kinematics deps; see [upstream installation docs](https://huggingface.co/docs/reachy_mini/SDK/installation) for hardware setup.

## Quick start

```python
from physagentos_reachy_mini_hal import ReachyMiniDriver, HeadPose

with ReachyMiniDriver() as robot:
    robot.head_goto(HeadPose(pitch=0.3, yaw=0.5), duration=1.0)
    frame = robot.read_camera_frame()
```

## Architecture

```
physagentOS agent
       |
       v
  RobotHAL (Protocol)   <-- defined here, implementation-agnostic
       ^
       |
ReachyMiniDriver        <-- this package
       |
       v
  reachy_mini.ReachyMini  (Pollen Robotics SDK)
       |
       v
  reachy-mini-daemon  /  motor controller  /  hardware
```

The HAL hooks into the SDK's high-level `ReachyMini` class so that joint-range safety clamps and Stewart-platform inverse kinematics come for free.

## License

Apache-2.0 — matches upstream Reachy Mini SDK.
