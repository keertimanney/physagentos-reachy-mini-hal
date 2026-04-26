"""Robust daemon connection helpers.

The Reachy Mini SDK's `spawn_daemon=True` constructor launches the daemon
subprocess but doesn't wait for it to be ready before attempting the
WebSocket handshake — on a cold start the daemon takes ~10-15s to boot
through motor checks and audio init, so the immediate connect fails.

`connect_with_fallback` papers over that race: try to connect to an existing
daemon first, fall back to spawning + polling port :8000 until it's reachable,
then connect for real.
"""

from __future__ import annotations

import socket
import time

from physagentos_reachy_mini_hal.driver import ReachyMiniDriver

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8000


def wait_for_port(host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, timeout: float = 60.0) -> bool:
    """Block until `host:port` accepts a TCP connection or `timeout` elapses."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=1.0):
                return True
        except OSError:
            time.sleep(0.5)
    return False


def connect_with_fallback(
    *,
    boot_timeout: float = 60.0,
    existing_timeout: float = 3.0,
    ready_timeout: float = 10.0,
) -> ReachyMiniDriver:
    """Return a connected `ReachyMiniDriver`, spawning a daemon if needed.

    Order of attempts:
      1. Connect to an already-running daemon on :8000 (e.g. the desktop app).
      2. If nothing is listening, ask the SDK to spawn a daemon. The first
         connect attempt typically fails because the daemon isn't ready yet.
      3. Poll :8000 for up to `boot_timeout` seconds until the daemon binds.
      4. Reconnect with `spawn_daemon=False`.

    Raises:
        RuntimeError: if the daemon never becomes ready within `boot_timeout`.
    """
    driver = ReachyMiniDriver(spawn_daemon=False, timeout=existing_timeout)
    try:
        driver.connect()
        return driver
    except Exception:
        pass

    try:
        ReachyMiniDriver(spawn_daemon=True, timeout=existing_timeout).connect()
    except Exception:
        pass

    if not wait_for_port(timeout=boot_timeout):
        raise RuntimeError(f"daemon did not become ready within {boot_timeout}s")

    driver = ReachyMiniDriver(spawn_daemon=False, timeout=ready_timeout)
    driver.connect()
    return driver
