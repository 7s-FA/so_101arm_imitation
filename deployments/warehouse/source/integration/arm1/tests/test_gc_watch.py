"""GC pause recording: thresholds, queueing without I/O, and startup freeze."""
import gc
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gc_watch import GcWatch, LOG_MS


class Clock:
    def __init__(self): self.t = 0.0
    def __call__(self): return self.t


class GcWatchTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.w = GcWatch(clock=self.clock, wall=lambda: 1793092413.79)

    def pause(self, ms, generation=2):
        self.w.callback('start', {'generation': generation})
        self.clock.t += ms/1000
        self.w.callback('stop', {'generation': generation, 'collected': 3, 'uncollectable': 0})

    def test_records_only_pauses_that_can_reach_bus_timeouts(self):
        self.pause(LOG_MS-.5); self.pause(520.0); self.pause(LOG_MS, generation=0)
        pauses = self.w.drain()
        self.assertEqual([(p['generation'], p['ms']) for p in pauses], [(2, 520.0), (0, LOG_MS)])
        self.assertEqual(pauses[0]['gc_time'], 1793092413.79)
        self.assertEqual(pauses[0]['thread'], 'MainThread')
        self.assertEqual(self.w.drain(), [])

    def test_stop_without_start_is_ignored(self):
        self.w.callback('stop', {'generation': 2, 'collected': 0, 'uncollectable': 0})
        self.assertEqual(self.w.drain(), [])

    def test_install_and_remove_once(self):
        try:
            self.w.install(); self.w.install()
            self.assertEqual(gc.callbacks.count(self.w.callback), 1)
        finally:
            self.w.remove()
        self.assertNotIn(self.w.callback, gc.callbacks)

    def test_freeze_excludes_startup_objects(self):
        try:
            result = GcWatch().freeze()
            self.assertGreater(result['frozen'], 0)
            self.assertGreaterEqual(result['full_collection_after_freeze_ms'], 0)
        finally:
            gc.unfreeze()


if __name__ == '__main__':
    unittest.main()
