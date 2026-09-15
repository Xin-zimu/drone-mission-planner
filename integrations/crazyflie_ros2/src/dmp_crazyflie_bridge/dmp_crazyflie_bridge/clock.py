from __future__ import annotations

import time


class BridgeClock:
    def monotonic(self) -> float:
        return time.monotonic()

