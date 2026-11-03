"""Garbage-collector pauses for the Arm1 server.

On the Jetson (2026-10-27) one full collection over the loaded A/B models + rclpy heap
(~343k tracked objects) took ~0.5 s: longer than the motor bus write timeout (100 ms),
the servo reply timeout (~34 ms) and the 250 ms control deadline. Python 3.12 runs the
collection in whichever thread is executing, including the motor worker mid-transaction.
freeze() moves everything loaded at startup out of later collections (measured 0.03 ms).

The callback only queues timings: it can run inside any thread, even while that thread
is writing the hardware log, so it must not do I/O. The server drains the queue.
"""
import collections
import gc
import threading
import time

LOG_MS = 10  # Shorter pauses cannot reach the 34 ms reply or 100 ms write timeouts.


class GcWatch:
    def __init__(self, *, clock=time.perf_counter, wall=time.time):
        self.clock, self.wall = clock, wall
        self.pending = collections.deque(maxlen=1000)
        self.start = None

    def freeze(self):
        """Collect startup garbage, then exclude all surviving objects from future collections."""
        gc.collect()
        gc.freeze()
        t = self.clock(); gc.collect()
        return {'frozen': gc.get_freeze_count(), 'full_collection_after_freeze_ms': round((self.clock()-t)*1000, 2)}

    def install(self):
        if self.callback not in gc.callbacks: gc.callbacks.append(self.callback)

    def remove(self):
        if self.callback in gc.callbacks: gc.callbacks.remove(self.callback)

    def callback(self, phase, info):
        # Collections never overlap (GIL + gc's own guard), so one start time suffices.
        if phase == 'start':
            self.start = self.clock(); return
        if self.start is None: return
        ms = (self.clock()-self.start)*1000; self.start = None
        if ms >= LOG_MS:
            self.pending.append({'gc_time': self.wall(), 'generation': info['generation'], 'ms': round(ms, 2),
                                 'collected': info['collected'], 'thread': threading.current_thread().name})

    def drain(self):
        out = []
        while self.pending: out.append(self.pending.popleft())
        return out
