# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Single-item buffer for as-of sampling with strict anti-future-leakage guarantee."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional
import numpy as np


@dataclass
class BufferItem:
    ts_ns: int
    value: Any


class LastBuffer:
    """Buffer holding the latest sample for as-of retrieval.

    Enforces:
    1. Alignment Guarantee: sample timestamp <= reference timestamp (zero future leakage).
    2. Stale Window: ref_ts - sample_ts <= max_age_ns.
    3. Reservoir Sampling for p95 latency tracking without memory overhead.
    """

    def __init__(
        self,
        *,
        max_age_ns: int,
        collect_p95: bool = False,
        reservoir_cap: int = 2000,
    ) -> None:
        self.max_age_ns = int(max_age_ns)
        self.item: Optional[BufferItem] = None

        # Diagnostic counters
        self.tried = 0
        self.matched = 0
        self.miss_empty = 0
        self.miss_future = 0
        self.miss_stale = 0

        self._dt_ns_sum = 0
        self._dt_ns_max = 0

        # Reservoir sampling for p95
        self.collect_p95 = bool(collect_p95)
        self._reservoir_cap = int(reservoir_cap)
        self._reservoir = (
            np.empty((self._reservoir_cap,), dtype=np.int64)
            if self.collect_p95
            else None
        )
        self._reservoir_n = 0

    def push(self, ts_ns: int, value: Any) -> None:
        """Push a new sample if its timestamp is >= current item's timestamp."""
        ts_ns = int(ts_ns)
        if self.item is None or ts_ns >= self.item.ts_ns:
            self.item = BufferItem(ts_ns=ts_ns, value=value)

    def asof(self, ref_ts_ns: int) -> Optional[Any]:
        """Return latest value if item.ts <= ref_ts and (ref_ts - item.ts) <= max_age_ns."""
        self.tried += 1
        if self.item is None:
            self.miss_empty += 1
            return None

        # Anti-future leakage check
        if self.item.ts_ns > ref_ts_ns:
            self.miss_future += 1
            return None

        # Stale check
        dt = ref_ts_ns - self.item.ts_ns
        if dt > self.max_age_ns:
            self.miss_stale += 1
            return None

        # Matched
        self.matched += 1
        self._dt_ns_sum += dt
        if dt > self._dt_ns_max:
            self._dt_ns_max = dt

        if self.collect_p95 and self._reservoir is not None:
            if self._reservoir_n < self._reservoir_cap:
                self._reservoir[self._reservoir_n] = dt
            else:
                # Algorithm R reservoir sampling
                idx = np.random.randint(0, self._reservoir_n + 1)
                if idx < self._reservoir_cap:
                    self._reservoir[idx] = dt
            self._reservoir_n += 1

        return self.item.value

    def summary(self) -> Dict[str, Any]:
        """Return diagnostic metrics dictionary."""
        match_rate = (self.matched / self.tried) if self.tried > 0 else 0.0
        mean_dt_s = (self._dt_ns_sum / max(1, self.matched)) * 1e-9
        max_dt_s = self._dt_ns_max * 1e-9

        p95_dt_s = None
        if self.collect_p95 and self._reservoir is not None and self._reservoir_n > 0:
            count = min(self._reservoir_n, self._reservoir_cap)
            p95_ns = np.percentile(self._reservoir[:count], 95)
            p95_dt_s = float(p95_ns) * 1e-9

        return {
            "tried": self.tried,
            "matched": self.matched,
            "match_rate": match_rate,
            "miss_empty": self.miss_empty,
            "miss_future": self.miss_future,
            "miss_stale": self.miss_stale,
            "mean_dt_s": mean_dt_s,
            "max_dt_s": max_dt_s,
            "p95_dt_s": p95_dt_s,
        }
