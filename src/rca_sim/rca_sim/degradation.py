"""Runtime degradation primitives shared by all simulation nodes.

Everything here produces *real* runtime symptoms (delayed publication, lost
messages, reduced rate, extra CPU work, corrupted measurements, node exit);
nothing writes a fault label anywhere the monitor could see it. The command
channel is ``/rca/fault_command`` (JSON), which the diagnostic monitor excludes
from graph discovery and never subscribes to.

Fault command payload::

    {"target_node": "<node name>|all", "fault_type": "<type>", "param": <float>,
     "duration": <sec>, "extra": {...optional...}}

    fault_type "clear" removes every active fault on the target.
"""

import heapq
import json
import math
import os
import random
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

# Fault types understood by the simulation (documented in README / scenarios.yaml)
SENSOR_FAULTS = ('noise', 'bias', 'dropout', 'stale', 'rate')
LINK_FAULTS = ('latency', 'jitter', 'loss')
WORKLOAD_FAULTS = ('workload', 'cpu_pressure')
LIFECYCLE_FAULTS = ('crash', 'stop', 'restart')
BEHAVIOUR_FAULTS = ('processing_delay', 'degradation', 'localization_failure', 'navigation_failure',
                    'control_failure', 'topic_change')
ALL_FAULTS = SENSOR_FAULTS + LINK_FAULTS + WORKLOAD_FAULTS + LIFECYCLE_FAULTS + BEHAVIOUR_FAULTS


class FaultState:
    """Active faults of one node: type -> (param, end_time, extra)."""

    def __init__(self, node_name: str):
        self.node_name = node_name
        self.active: Dict[str, Tuple[float, float, Dict[str, Any]]] = {}

    def parse(self, payload: str) -> Optional[Dict[str, Any]]:
        """Returns the command dict when it targets this node (or 'all'), else None."""
        try:
            cmd = json.loads(payload)
        except (ValueError, TypeError):
            return None
        if cmd.get('target_node') not in (self.node_name, 'all'):
            return None
        return cmd

    def apply(self, cmd: Dict[str, Any], now: float) -> Optional[str]:
        """Applies a parsed command. Returns the fault type applied ('clear' included)."""
        f_type = str(cmd.get('fault_type', 'clear'))
        if f_type == 'clear':
            self.active.clear()
            return 'clear'
        param = float(cmd.get('param', 0.0) or 0.0)
        duration = float(cmd.get('duration', 10.0) or 10.0)
        extra = cmd.get('extra') or {}
        self.active[f_type] = (param, now + duration if duration > 0 else float('inf'), dict(extra))
        return f_type

    def expire(self, now: float) -> List[str]:
        expired = [t for t, (_, end, _) in self.active.items() if now > end]
        for t in expired:
            self.active.pop(t, None)
        return expired

    def is_active(self, f_type: str) -> bool:
        return f_type in self.active

    def param(self, f_type: str, default: float = 0.0) -> float:
        p = self.active.get(f_type)
        return p[0] if p is not None and p[0] > 0 else default

    def extra(self, f_type: str) -> Dict[str, Any]:
        p = self.active.get(f_type)
        return p[2] if p is not None else {}

    def types(self) -> List[str]:
        return sorted(self.active)


class DegradableLink:
    """Outbound communication model in front of a publisher.

    publish() enqueues the message with its *real* dispatch time
    (now + latency + jitter) or drops it (loss / rate reduction);
    drain(now) actually publishes everything whose time has come. The message
    keeps the stamp it was created with, so end-to-end latency measured by a
    subscriber is the physical delay through this link.
    """

    def __init__(self, publish_fn: Callable[[Any], None], faults: FaultState, rng: random.Random):
        self._publish = publish_fn
        self._faults = faults
        self._rng = rng
        self._queue: List[Tuple[float, int, Any]] = []
        self._seq = 0
        self._lock = threading.Lock()
        self.sent = 0
        self.dropped = 0
        self._rate_accum = 0.0

    def publish(self, msg: Any, now: Optional[float] = None):
        now = time.time() if now is None else now
        f = self._faults
        if f.is_active('loss') and self._rng.random() < min(1.0, f.param('loss', 0.3)):
            self.dropped += 1
            return
        if f.is_active('rate'):
            # keep only every k-th message: param = keep fraction (0.25 -> quarter rate)
            keep = max(0.01, min(1.0, f.param('rate', 0.25)))
            self._rate_accum += keep
            if self._rate_accum < 1.0:
                self.dropped += 1
                return
            self._rate_accum -= 1.0
        delay = 0.0
        if f.is_active('latency'):
            delay += f.param('latency', 0.3)
        if f.is_active('jitter'):
            delay += abs(self._rng.gauss(0.0, f.param('jitter', 0.15)))
        with self._lock:
            self._seq += 1
            heapq.heappush(self._queue, (now + delay, self._seq, msg))

    def drain(self, now: Optional[float] = None) -> int:
        now = time.time() if now is None else now
        out = []
        with self._lock:
            while self._queue and self._queue[0][0] <= now:
                out.append(heapq.heappop(self._queue)[2])
        for m in out:
            self._publish(m)
            self.sent += 1
        return len(out)

    def pending(self) -> int:
        with self._lock:
            return len(self._queue)

    def clear(self):
        with self._lock:
            self._queue.clear()


def busy_work(milliseconds: float):
    """Burns real CPU time (pure Python arithmetic) for roughly the given duration."""
    if milliseconds <= 0:
        return
    end = time.perf_counter() + milliseconds / 1000.0
    x = 1.0001
    while time.perf_counter() < end:
        for _ in range(200):
            x = math.sqrt(x * x + 1.0)


class CpuPressure:
    """Background busy threads inside the process (GIL contention = real slowdown)."""

    def __init__(self):
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []

    def start(self, n_threads: int = 2):
        self.stop()
        self._stop.clear()
        for _ in range(max(1, int(n_threads))):
            t = threading.Thread(target=self._spin, daemon=True)
            t.start()
            self._threads.append(t)

    def _spin(self):
        while not self._stop.is_set():
            busy_work(5.0)
            time.sleep(0.001)

    def stop(self):
        self._stop.set()
        self._threads = []

    @property
    def running(self) -> bool:
        return bool(self._threads) and not self._stop.is_set()


def process_exit(graceful: bool):
    """Node lifecycle fault for process-based nodes: crash (abrupt) or stop (clean)."""
    if graceful:
        raise SystemExit(0)
    os._exit(1)
