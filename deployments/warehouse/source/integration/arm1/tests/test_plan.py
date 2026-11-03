"""Plan replay in the Arm1 server: timing, squeeze and tracking rules, backend phases, verification after return."""
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from plan_runner import PlanRunner,JOINTS
from engine import Controller,MockBackend,JobRejected

CAL={j:{'range_min':1000,'range_max':3000} for j in JOINTS}


def plan_file(raw,**extra):
    d=Path(tempfile.mkdtemp());cal=d/'cal.json';cal.write_text(json.dumps(CAL))
    import hashlib
    p=d/'plan.json';p.write_text(json.dumps(dict(joint_order=list(JOINTS),raw=raw,command_hz=15.,robot_calibration=str(cal),
        robot_calibration_sha256=hashlib.sha256(cal.read_bytes()).hexdigest(),part_label='test',**extra)));return p


class Runner(unittest.TestCase):
    def test_clock_interpolates_and_pauses(self):
        r=PlanRunner(plan_file([[2000]*6,[2030]*6,[2060]*6]),CAL);m={j:2000 for j in JOINTS}
        self.assertEqual(r.next(m,10.)[0]['shoulder_pan'],2000)
        self.assertEqual(r.next(m,10.+1/30)[0]['shoulder_pan'],2015)          # half a 15Hz frame
        r.pause();self.assertEqual(r.next(m,99.)[0]['shoulder_pan'],2015)      # stopped time does not count
        for k in range(10):r.next(m,99.+(k+1)/30)
        self.assertTrue(r.done);self.assertEqual(r.next(m,200.)[0]['shoulder_pan'],2060)
    def test_squeeze_limits_the_closing_gripper_lead(self):
        raw=[[2000]*5+[1500]]*3
        r=PlanRunner(plan_file(raw,gripper_squeeze_limit_units=2.5),CAL)
        goal,_=r.next({**{j:2000 for j in JOINTS},'gripper':1600},1.)
        self.assertEqual(goal['gripper'],1600-50)                              # 2.5 points * 20 ticks
    def test_tracking_watchdog(self):
        r=PlanRunner(plan_file([[2000]*6]*3),CAL);far={**{j:2000 for j in JOINTS},'shoulder_lift':2000-200}
        r.next(far,1.);r.next(far,1.3)
        with self.assertRaisesRegex(RuntimeError,'tracking error'):r.next(far,1.6)
    def test_joint_tracking_bias_is_bounded_and_clipped_inside_range(self):
        r=PlanRunner(plan_file([[2990]*6]*40,joint_tracking=dict(kp=.25,ki=2.,max_deg=3.,start_index=0)),CAL)
        m={j:2950 for j in JOINTS}
        for k in range(30):goal,_=r.next(m,k/30)
        self.assertEqual(goal['shoulder_pan'],3000-2);self.assertEqual(goal['gripper'],2990)
    def test_rejects_a_plan_for_another_calibration(self):
        with self.assertRaisesRegex(ValueError,'calibration'):PlanRunner(plan_file([[2000]*6]),{**CAL,'gripper':{'range_min':0,'range_max':1}})


class VerifyAfterReturn(unittest.TestCase):
    """Engine: a plan job is confirmed only after the return; a failed check is an ERROR result with the arm held."""
    def run_job(self,verdicts):
        b=MockBackend(steps=3);b.verify_after_return=True;seq=iter(verdicts)
        b.verify_placement=lambda:next(seq,verdicts[-1]);b.verify_failure='PLACEMENT_NOT_CONFIRMED: test'
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup)
        c=Controller(b,log_path=Path(temp.name)/'e.jsonl',period=.002,completion_delay=.01);self.addCleanup(c.close)
        t=c.reserve('SELECT_B');c.activate(t);self.assertTrue(t.done.wait(3));return t,c
    def test_success_waits_for_the_check(self):
        t,c=self.run_job([None,None,True]);self.assertTrue(t.success)
    def test_failed_check_reports_error(self):
        t,c=self.run_job([None,False]);self.assertFalse(t.success);self.assertIn('PLACEMENT_NOT_CONFIRMED',t.detail)
        deadline=time.monotonic()+2
        while c.status()['state']!='IDLE' and time.monotonic()<deadline:time.sleep(.01)
        self.assertEqual(c.status()['state'],'IDLE')


class BackendPlanPhases(unittest.TestCase):
    def setUp(self):
        from backend import HardwareBackend
        b=self.b=HardwareBackend.__new__(HardwareBackend)
        class Missing(Exception):pass
        b.p=SimpleNamespace(MAX_INFERENCE_SECONDS=.25,CAMERAS=('overview','wrist'),CameraUnavailable=Missing,
            control=SimpleNamespace(JOINTS=list(JOINTS),SERVO_PROFILE_SPEED=0,RETURN_PROFILE_SPEED=112,RETURN_TIMEOUT_S=120))
        b.bus=Mock();b.armed=True;b.cal=CAL;b.permit=lambda:True
        b.current={j:2000 for j in JOINTS};b._sample=Mock(side_effect=lambda:dict(b.current))
        b.cameras=Mock();b.cameras.snapshot.side_effect=lambda:{'overview':(None,time.monotonic()),'wrist':(None,time.monotonic())}
        b.camera_since=b.camera_stable=b.camera_previous=None
        b.predictor=Mock();b.detector=Mock();b.contact=Mock()
        b.planner=Mock();b.planner.next.side_effect=lambda requested,current:dict(requested)
        b._write=Mock();b.hold_pose=None;b.hold_since=None;b.arrival_since=None;b.home_started=None
        b.aligned=lambda a,c,*_:a==c;b.pose={'raw':{j:2100 for j in JOINTS}};b.phase=None;b.done_since=None
        b.plan=PlanRunner(plan_file([[2000]*6,[2010]*6]),CAL)
        b.vision_job={'selection':{'set':'B_left'}}
    def test_homing_goes_to_the_plan_start_and_work_replays_it(self):
        self.b.step('HOMING');self.assertEqual(self.b._write.call_args.args[0],self.b.plan.start)
        from unittest.mock import patch
        clock=iter(np.arange(100,101,1/30))
        with patch('backend.time.monotonic',side_effect=lambda:float(next(clock))):
            self.b.step('WORKING');self.b.predictor.predict.assert_not_called();self.b.contact.next.assert_not_called()
            for _ in range(4):last=self.b.step('WORKING')
        self.assertTrue(last);self.assertTrue(self.b.verify_after_return)
    def test_return_goes_to_the_model_start_pose(self):
        self.b.bus.read.side_effect=lambda j,r:112 if r=='Goal_Velocity' else 0
        self.b.step('RETURNING');self.assertEqual(self.b._write.call_args.args[0],self.b.pose['raw'])


class RecoverWithCamerasOpen(unittest.TestCase):
    """CLEAR_ERROR after a fault: the always-open vision cameras must not block the motor verification."""
    def test_open_cameras_do_not_block_recovery(self):
        from backend import HardwareBackend
        b=HardwareBackend.__new__(HardwareBackend);b.bus=None;b.armed=False;b.cameras=Mock();b.cal=CAL;b.guard=None;b.cfg={'robot':{'port':'/dev/null'}}
        bus=Mock();b.p=SimpleNamespace(control=SimpleNamespace(SerialBus=Mock(return_value=bus),PortConfig=Mock(),verify=Mock(),feedback=Mock()))
        b.recover_cleanup();bus.open.assert_called_once();bus.close.assert_called_once()
        b.bus=Mock()
        with self.assertRaisesRegex(RuntimeError,'not closed'):b.recover_cleanup()


class BurgerPlanSelection(unittest.TestCase):
    """PRECHECK plan choice from the measured target pallet: m2 base plan, verified m1 plan, or a retargeted plan."""
    def backend(self,frame_bgr):
        from backend import HardwareBackend
        from jig_cells import JigCells
        from vision_judge import VisionJudge
        import cv2
        b=HardwareBackend.__new__(HardwareBackend);cfg=json.loads((Path(__file__).resolve().parents[1]/'model-map.json').read_text())['burger_correction']
        cfg['measure_wait_s']=0   # These calls take no real time; test_retarget_memory.MeasureWait covers the wait.
        b.burger_cfg=cfg;b.jig_cells=JigCells(cfg,VisionJudge().config['cells']);b.burger_state=None
        b.plan_paths={'A_lower':Path('/base/A_lower.json'),'B_left':Path('/base/B_left.json')}
        self.rgb=cv2.cvtColor(frame_bgr,cv2.COLOR_BGR2RGB);return b
    def choose(self,b,key):
        for _ in range(3):
            path=b._burger_plan(key,self.rgb)
        return path
    def test_reference_burger_keeps_the_base_plan(self):
        import cv2
        b=self.backend(cv2.imread(str(Path(__file__).resolve().parents[1]/'jig-reference-m2.png')))
        self.assertEqual(self.choose(b,'A_lower'),Path('/base/A_lower.json'));self.assertEqual(b.burger_state['info']['mode'],'m2 기준 계획')
    def test_m1_uses_its_verified_plan(self):
        import cv2
        f=Path(__file__).resolve().parent/'data'/'m1_live_20261025_160823.png'
        if not f.exists():self.skipTest('m1 frame not on this machine')
        b=self.backend(cv2.imread(str(f)));path=self.choose(b,'B_left')
        self.assertTrue(str(path).endswith('B_left_three_parts_v9_fast_A3_m1_tpu_up7_left7.json'));self.assertEqual(b.burger_state['info']['mode'],'m1 검증 계획')
    def test_moved_camera_is_rejected(self):
        import cv2
        ref=cv2.imread(str(Path(__file__).resolve().parents[1]/'jig-reference-m2.png'))
        moved=cv2.warpAffine(ref,np.float32([[1,0,3],[0,1,2]]),(640,480),borderMode=cv2.BORDER_REPLICATE)   # whole view shifted: camera, not pallet
        b=self.backend(moved)
        with self.assertRaisesRegex(JobRejected,'JIG_MEASURE_FAILED: camera moved'):
            for _ in range(6):b._burger_plan('A_lower',self.rgb)
    def test_unreadable_pallet_is_rejected(self):
        b=self.backend(np.full((480,640,3),128,np.uint8))
        with self.assertRaisesRegex(JobRejected,'JIG_MEASURE_FAILED'):
            for _ in range(6):b._burger_plan('A_lower',self.rgb)


class ExtraByPallet(unittest.TestCase):
    """B_TPU needs unrelated corrections on the two pallets; other parts keep the m1 extra scaled along m2 -> m1."""
    def extra(self,offset,tmp):
        from jig_cells import JigCells
        anchors=Path(tmp)/'anchors.json'
        anchors.write_text(json.dumps({'burgers':{'m2':{'extra_mm':{'B_TPU':[-12.82,9.8],'B_lid':[0.,0.]}},
                                                  'm1':{'cell_offset_mm':{'B_TPU':[0.,-10.]},'extra_mm':{'B_TPU':[-13.07,-.93],'B_lid':[2.,0.]}}}}))
        j=JigCells.__new__(JigCells);j.c={'anchors':str(anchors)}
        return j.extra_mm({'cells':{'B_TPU':{'offset_mm':offset}}})
    def test_m2_side_uses_its_own_value_unscaled(self):
        with tempfile.TemporaryDirectory() as tmp:
            for y in (-2.,-5.):
                alpha,extra=self.extra([0.,y],tmp)
                self.assertLess(alpha,.6);self.assertEqual(extra['B_TPU'],[-12.82,9.8])
                self.assertEqual(extra['B_lid'],[round(2.*alpha,2),0.])
    def test_m1_side_keeps_the_scaled_m1_value(self):
        with tempfile.TemporaryDirectory() as tmp:
            alpha,extra=self.extra([0.,-9.],tmp)
        self.assertAlmostEqual(alpha,.9);self.assertEqual(extra['B_TPU'],[-11.76,-.84])


if __name__=='__main__':unittest.main()
