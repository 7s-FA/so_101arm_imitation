"""IK retarget launch: memory guard before the child starts, and the per-set release retreat argument."""
import json
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import backend
from backend import HardwareBackend
from engine import JobRejected

BURGER_LOGS = Path(backend.__file__).resolve().parent/'logs'/'burger'


class PastGuard(Exception):
    """Raised where the retarget work directory would be named: the guard let the job through."""


def measurement(image):
    return {'ok': True, 'camera': {'ok': True},
            'cells': {'A_housing': {'offset_mm': [4.71, -5.21], 'shift_px': [-5.1, 5.15]}},
            'rigid': {'rotation_deg': 1.699, 'translation_mm': [4.14, -8.2], 'pivot_xy_m': [.1345, .2288]}}


class BurgerPlanCase(unittest.TestCase):
    def backend(self, **cfg):
        b = HardwareBackend.__new__(HardwareBackend)
        b.plan_paths = {'A_lower': Path('/base/A_lower.json')}
        b.burger_cfg = {'enabled': True, 'frames': 3, 'max_failures': 5, **cfg}
        b.burger_state = None
        b.jig_cells = Mock(); b.jig_cells.measure.side_effect = measurement
        b.jig_cells.at_reference.return_value = False; b.jig_cells.match_anchor.return_value = None
        b.jig_cells.extra_mm.return_value = (.797, {'A_housing': [0.0, 0.0]})
        return b

    def choose(self, b):
        rgb = np.zeros((480, 640, 3), np.uint8)
        for stamp in range(3): path = b._burger_plan('A_lower', rgb, stamp=stamp)
        return path

    def listing(self):
        return sorted(BURGER_LOGS.iterdir()) if BURGER_LOGS.exists() else []


class RetargetMemoryGuard(BurgerPlanCase):
    def test_low_memory_refuses_before_starting_the_child(self):
        b = self.backend(); before = self.listing()
        with patch.object(backend, 'available_mb', return_value=1200):
            with self.assertRaises(JobRejected) as raised: self.choose(b)
        self.assertIn('RETARGET_LOW_MEMORY', str(raised.exception)); self.assertIn('1200MB < 1500MB', str(raised.exception))
        self.assertIsNone(b.burger_state['thread'])
        self.assertEqual(self.listing(), before)

    def test_configured_threshold_and_enough_memory_continue(self):
        for cfg, free in (({}, 4000), ({'retarget_min_available_mb': 1000}, 1200)):
            b = self.backend(**cfg)
            with patch.object(backend, 'available_mb', return_value=free), patch('time.strftime', side_effect=PastGuard):
                with self.assertRaises(PastGuard): self.choose(b)

    def test_unreadable_meminfo_keeps_previous_behaviour(self):
        b = self.backend()
        with patch.object(backend, 'available_mb', return_value=None), patch('time.strftime', side_effect=PastGuard):
            with self.assertRaises(PastGuard): self.choose(b)

    def test_reads_meminfo(self):
        free = backend.available_mb()
        self.assertIsInstance(free, int); self.assertGreater(free, 0)


class MeasureWait(BurgerPlanCase):
    """measure_wait_s: failed pallet measurements retry on fresh frames before the job is refused."""
    def frames(self, b, results, clock, step=.04):
        rgb = np.zeros((480, 640, 3), np.uint8)
        b.jig_cells.measure.side_effect = results
        with patch('time.monotonic', side_effect=lambda: clock['t']):
            for stamp, _ in enumerate(results):
                clock['t'] += step
                b._burger_plan('A_lower', rgb, stamp=stamp)

    def test_refused_only_after_the_wait(self):
        b = self.backend(measure_wait_s=5.0); clock = {'t': 100.}
        failed = {'ok': False, 'reason': 'camera moved or board hidden'}
        self.frames(b, [failed]*100, clock)   # 4 s of failures: well past max_failures, inside the wait
        self.assertEqual(b.burger_state['failures'], 100)
        with self.assertRaisesRegex(JobRejected, 'JIG_MEASURE_FAILED: camera moved'):
            self.frames(b, [failed]*30, clock)
        self.assertGreaterEqual(clock['t']-100., 5.0)

    def test_only_frames_after_the_last_failure_form_the_plan(self):
        b = self.backend(measure_wait_s=5.0); clock = {'t': 100.}
        failed = {'ok': False, 'reason': 'A_cap: cell match score 0.28'}
        before = dict(measurement(None), marker='before'); after = dict(measurement(None), marker='after')
        self.frames(b, [before, before]+[failed]*8+[after, after], clock)
        self.assertEqual([m['marker'] for m in b.burger_state['measurements']], ['after', 'after'])
        b.jig_cells.measure.side_effect = [after]
        with patch.object(backend, 'available_mb', return_value=4000), patch('time.strftime', side_effect=PastGuard):
            with self.assertRaises(PastGuard): b._burger_plan('A_lower', np.zeros((480, 640, 3), np.uint8), stamp=99)
        self.assertEqual([m['marker'] for m in b.burger_state['measurements']], ['after']*3)

    def test_without_the_setting_rejection_stays_immediate(self):
        b = self.backend(); clock = {'t': 100.}
        with self.assertRaisesRegex(JobRejected, 'JIG_MEASURE_FAILED'):
            self.frames(b, [{'ok': False, 'reason': 'x'}]*5, clock)


class ReleaseRetreatArgument(BurgerPlanCase):
    def launch(self, b):
        """Start the retarget with a stub child; return its command line and the work directory to clean up."""
        calls = []; name = 'unittest_retreat'
        def run(cmd, **kwargs):
            calls.append(cmd); return SimpleNamespace(returncode=1, stdout='', stderr='stub child')
        with patch.object(backend, 'available_mb', return_value=4000), patch('time.strftime', return_value=name), \
             patch('subprocess.run', side_effect=run):
            self.assertIsNone(self.choose(b)); b.burger_state['thread'].join(5)
        return calls[0], BURGER_LOGS/f'{name}_A_lower'

    def test_configured_set_passes_its_retreat(self):
        b = self.backend(retarget_script='retarget.py', release_retreat_mm={'A_lower': {'A_housing': [-5.0, 0.0]}})
        cmd, work = self.launch(b)
        try:
            path = Path(cmd[cmd.index('--retreat')+1])
            self.assertEqual(json.loads(path.read_text()), {'A_housing': [-5.0, 0.0]})
            self.assertEqual(b.burger_state['info']['release_retreat_mm'], {'A_housing': [-5.0, 0.0]})
        finally:
            shutil.rmtree(work, ignore_errors=True)

    def test_other_sets_launch_unchanged(self):
        b = self.backend(retarget_script='retarget.py', release_retreat_mm={'A_upper': {'A_housing': [-5.0, 0.0]}})
        cmd, work = self.launch(b)
        try:
            self.assertNotIn('--retreat', cmd); self.assertFalse((work/'retreat.json').exists())
        finally:
            shutil.rmtree(work, ignore_errors=True)


if __name__ == '__main__':
    unittest.main()
