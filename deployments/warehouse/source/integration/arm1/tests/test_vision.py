"""Vision judge on three recorded D435 scenes: set presence, selection order, destination fill."""
from pathlib import Path
import sys
import time
import unittest

import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from vision_judge import VisionJudge

DATA=Path(__file__).resolve().parent/'data'


def scene(name):return cv2.cvtColor(cv2.imread(str(DATA/f'vision_{name}.png')),cv2.COLOR_BGR2RGB)


class VisionJudgeScenes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.j=VisionJudge()

    def check(self,name,complete,select_a,select_b):
        r=self.j.evaluate(scene(name))
        self.assertEqual({k for k,v in r['sets'].items() if v['complete']},complete,name)
        self.assertEqual(self.j.select('SELECT_A',r)['set'],select_a,name)
        self.assertEqual(self.j.select('SELECT_B',r)['set'],select_b,name)
        return r

    def test_both_purple_sets_take_the_lower_one_and_orange_left(self):
        # 2026-10-24 sequence run start: both purple sets, orange left set, one extra orange lid on the right.
        r=self.check('today',{'A_upper','A_lower','B_left'},'A_lower','B_left')
        self.assertEqual(r['sets']['B_right']['count'],1)

    def test_only_right_orange_set_is_taken(self):
        self.check('ep199_start',{'A_upper','A_lower','B_right'},'A_lower','B_right')

    def test_after_the_lower_set_moved_the_upper_set_is_next(self):
        r=self.check('ep199_end',{'A_upper','B_right'},'A_upper','B_right')
        self.assertEqual(len(r['targets']['A']['filled']),3);self.assertEqual(r['targets']['B']['filled'],[])

    def test_no_material_is_an_error_with_the_partial_count(self):
        img=scene('today')
        for name in ('source_A_housing_1','source_A_ball_2'):       # one part missing from each purple set
            img[self.j.masks[name]]=(235,235,235)
        sel=self.j.select('SELECT_A',self.j.evaluate(img))
        self.assertIsNone(sel['set']);self.assertEqual(sel['error'],'NO_MATERIAL');self.assertIn('2/3',sel['reason'])
        empty=np.full((480,640,3),235,np.uint8)
        self.assertEqual(self.j.select('SELECT_B',self.j.evaluate(empty))['error'],'NO_MATERIAL')

    def test_annotation_marks_the_selected_set(self):
        r=self.j.evaluate(scene('today'));sel=self.j.select('SELECT_A',r);img=self.j.annotate(scene('today'),r,sel)
        self.assertEqual(img.shape,(480,640,3))
        x,y=np.array(self.j.config['cells']['source_A_housing_2']['polygon'],int)[0]
        self.assertTrue((img[max(0,y-2):y+3,max(0,x-2):x+3]==(0,230,255)).all(axis=-1).any())


class PrecheckMaterial(unittest.TestCase):
    """Backend PRECHECK: three agreeing frames, then NO_MATERIAL rejects before any torque or motion."""
    def setUp(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        from backend import HardwareBackend
        b=self.b=HardwareBackend.__new__(HardwareBackend)
        b.p=SimpleNamespace(MAX_INFERENCE_SECONDS=.25,CAMERAS=('overview','wrist'),control=SimpleNamespace(JOINTS=['a']))
        b.permit=lambda:True;b.bus=Mock();b.armed=False;b._sample=Mock(return_value={})
        b.judge=VisionJudge();b.command='SELECT_A';b.select_votes=[];b.vision_job=None
        b.detector=Mock();b.detector.observe.return_value=False;b.detector.last={'candidate':False}
    def frames(self,img):self.b._frames=lambda raw,tick:{'overview':(img,time.monotonic())}
    def test_missing_material_is_rejected_after_three_frames(self):
        from engine import JobRejected
        self.frames(np.full((480,640,3),235,np.uint8))
        self.assertFalse(self.b.step('PRECHECK'));self.assertFalse(self.b.step('PRECHECK'))
        with self.assertRaisesRegex(JobRejected,'NO_MATERIAL'):self.b.step('PRECHECK')
        self.b.bus.write.assert_not_called();self.b.bus.write_position_profile.assert_not_called()
        self.assertEqual(self.b.vision_job['selection']['error'],'NO_MATERIAL')
    def test_complete_set_proceeds_to_the_existing_completion_check(self):
        self.frames(scene('today'))
        for _ in range(2):self.assertFalse(self.b.step('PRECHECK'))
        self.assertTrue(self.b.step('PRECHECK'));self.assertEqual(self.b.vision_job['selection']['set'],'A_lower')
        self.b.detector.observe.assert_called_once()

    def test_a_set_with_a_plan_runs_the_plan_and_others_the_model(self):
        from test_plan import plan_file,CAL
        from plan_runner import PlanRunner
        b=self.b;b.cal=CAL;b.PlanRunner=PlanRunner;b.plan_paths={'B_left':plan_file([[2000]*6]*3)};self.frames(scene('today'))
        b.command='SELECT_B';b.plan=None
        for _ in range(3):b.step('PRECHECK')
        self.assertIsNotNone(b.plan);self.assertTrue(b.vision_job['executor'].startswith('plan'))
        b.command='SELECT_A';b.plan=None;b.select_votes=[]
        for _ in range(3):b.step('PRECHECK')
        self.assertIsNone(b.plan);self.assertEqual(b.vision_job['executor'],'imitation model')


if __name__=='__main__':unittest.main()
