"""Wake up Reachy via the HAL.

Connects to an existing daemon if one is running (e.g. the desktop control
app on :8000), otherwise spawns a fresh daemon and waits for it to be ready.

Run:
    python scripts/wake.py
"""

from __future__ import annotations

import sys
import time

from physagentos_reachy_mini_hal import connect_with_fallback


def main() -> int:
    print("connecting...", flush=True)
    driver = connect_with_fallback()
    try:
        print("waking up...", flush=True)
        driver.wake_up()
        time.sleep(2.0)
        print("done", flush=True)
        return 0
    finally:
        driver.disconnect()


if __name__ == "__main__":
    sys.exit(main())
