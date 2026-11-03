"""Ensure overlapping PRECHECK keeps fresh-frame and camera safeguards."""
from pathlib import Path
import sys
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from backend import HardwareBackend
from engine import JobRejected


class PrecheckOverlap(unittest.TestCase):
    def setUp(self):
        b=self.b=HardwareBackend.__new__(HardwareBackend)
        b.p=SimpleNamespace(MAX_INFERENCE_SECONDS=.25)
        b.permit=lambda:True
        b._sample=Mock(return_value={})
        b.bus=Mock();b.armed=False;b.command='SELECT_A'
        b.select_votes=[];b.select_stamp=None;b.plan=None;b.burger_state=None
        b.plan_paths={'A_lower':Path('/plan.json')};b.burger_cfg={'enabled':True,'frames':3,'max_failures':5}
        b.judge=Mock();b.judge.evaluate.return_value={'sets':{},'targets':{}}
        b.judge.select.return_value={'set':'A_lower','error':None}
        b.detector=Mock();b.detector.observe.return_value=False;b.detector.last={'candidate':False}
        b.jig_cells=Mock();b.jig_cells.at_reference.return_value=True
        b.jig_cells.measure.side_effect=lambda image: {
            'ok':True,'camera':{'ok':True},'cells':{'part':{'offset_mm':[0,0],'shift_px':[0,0]}},
            'rigid':{'translation_mm':[0,0],'rotation_deg':0,'pivot_xy_m':[0,0]}}
        b.PlanRunner=lambda path,cal:SimpleNamespace(path=path)
        b.cal={}
        self.stamp=1.0
        b._frames=lambda raw,tick:{'overview':(np.zeros((480,640,3),np.uint8),self.stamp)}

    def test_three_distinct_frames_complete_both_checks(self):
        b=self.b
        self.assertFalse(b.step('PRECHECK'))
        self.assertFalse(b.step('PRECHECK'))  # Same buffered frame: no second vote.
        self.assertEqual(b.jig_cells.measure.call_count,1)
        self.stamp=2.0;self.assertFalse(b.step('PRECHECK'))
        self.assertEqual(b.jig_cells.measure.call_count,2)
        self.stamp=3.0;self.assertTrue(b.step('PRECHECK'))
        self.assertEqual(b.jig_cells.measure.call_count,3)
        self.assertEqual(b.plan.path,Path('/plan.json'))
        b.bus.write.assert_not_called();b.bus.write_position_profile.assert_not_called()

    def test_changed_camera_is_rejected_before_motion(self):
        b=self.b;b.jig_cells.measure.side_effect=None;b.jig_cells.measure.return_value={'ok':False,'reason':'camera moved'}
        for stamp in range(1,5):
            self.stamp=float(stamp);self.assertFalse(b.step('PRECHECK'))
        self.stamp=5.0
        with self.assertRaisesRegex(JobRejected,'JIG_MEASURE_FAILED: camera moved'):
            b.step('PRECHECK')
        b.bus.write.assert_not_called();b.bus.write_position_profile.assert_not_called()

    def test_selection_change_discards_prior_measurements(self):
        b=self.b
        self.assertFalse(b.step('PRECHECK'));self.assertEqual(len(b.burger_state['measurements']),1)
        b.judge.select.return_value={'set':None,'error':'NO_MATERIAL','reason':'missing'}
        self.stamp=2.0;self.assertFalse(b.step('PRECHECK'))
        self.assertIsNone(b.burger_state)
        b.judge.select.return_value={'set':'A_lower','error':None}
        self.stamp=3.0;self.assertFalse(b.step('PRECHECK'))
        self.assertEqual(len(b.burger_state['measurements']),1)
        self.assertIsNone(b.plan)


if __name__=='__main__':unittest.main()
