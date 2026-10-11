"""Actual hardware-adapter control flow with fake transport and camera frames."""
from pathlib import Path
import sys
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from backend import HardwareBackend
from engine import MotionBlocked


class AdapterFlow(unittest.TestCase):
    def setUp(self):
        b=self.b=HardwareBackend.__new__(HardwareBackend)
        class Missing(Exception):pass
        b.p=SimpleNamespace(MAX_INFERENCE_SECONDS=.25,CAMERAS=('overview','wrist'),CameraUnavailable=Missing,
            control=SimpleNamespace(JOINTS=['a','gripper'],SERVO_PROFILE_SPEED=0,
                                    RETURN_PROFILE_SPEED=56,RETURN_TIMEOUT_S=120),
            vector=lambda raw,cal:raw,raw_action=lambda action,cal:action,release=Mock(return_value=[]))
        b.bus=Mock();b.armed=True;b.cal={};b.permit=lambda:True
        b.bus.read.side_effect=lambda j,r:56 if r=='Goal_Velocity' else 0 if r=='Goal_Time' else 1
        b.current={'a':2000,'gripper':1800};b._sample=Mock(side_effect=lambda:dict(b.current))
        b.cameras=Mock();b.cameras.snapshot.side_effect=lambda:{'overview':(None,time.monotonic()),'wrist':(None,time.monotonic())}
        b.camera_since=b.camera_stable=b.camera_previous=None
        b.predictor=Mock();b.predictor.predict.return_value={'a':2010,'gripper':1790}
        b.detector=Mock();b.detector.observe.return_value=False;b.detector.last={'candidate':False}
        b.pose={'raw':dict(b.current)};b.phase='WORKING';b.candidate_pose=None
        b.planner=Mock();b.planner.next.side_effect=lambda requested,current:dict(requested)
        b.contact=Mock();b.contact.next.side_effect=lambda requested,*_:requested
        b._write=Mock();b.hold_pose=None;b.hold_since=None;b.arrival_since=None;b.home_started=None
        b.aligned=lambda a,b,*_:a==b
    def test_stop_received_during_inference_discards_prediction(self):
        b=self.b
        def inference(*_):
            b.permit=lambda:False
            return {'a':2100,'gripper':1800}
        b.predictor.predict.side_effect=inference
        with self.assertRaises(MotionBlocked):b.step('WORKING')
        b._write.assert_not_called()
    def test_completion_prevents_another_policy_action(self):
        b=self.b;b.detector.observe.return_value=True
        self.assertTrue(b.step('WORKING'))
        b._write.assert_not_called();b.predictor.predict.assert_not_called()
    def test_candidate_hold_does_not_follow_drifting_measurement(self):
        b=self.b;b.detector.last={'candidate':True}
        self.assertFalse(b.step('WORKING'))
        b.current={'a':2005,'gripper':1804}
        self.assertFalse(b.step('WORKING'))
        self.assertEqual(b._write.call_args.args[0],{'a':2000,'gripper':1800})
        b.predictor.predict.assert_not_called()
    def test_return_requires_stable_arrival(self):
        b=self.b
        with patch('backend.time.monotonic',return_value=100):
            self.assertFalse(b.step('RETURNING'))
        with patch('backend.time.monotonic',return_value=100.51):
            self.assertTrue(b.step('RETURNING'))
        b.predictor.predict.assert_not_called()
        self.assertEqual(b._write.call_args.kwargs['speed'],56)
    def test_return_speed_failure_is_not_success_and_resume_rechecks(self):
        b=self.b
        b.bus.read.side_effect=lambda j,r:0
        with self.assertRaisesRegex(RuntimeError,'return speed profile readback'):
            b.step('RETURNING')
        self.assertFalse(b.return_profile_verified)
        b.return_profile_verified=True
        b.resume()
        self.assertFalse(b.return_profile_verified)
    def test_work_and_initial_home_keep_normal_profile(self):
        for phase in ('HOMING','WORKING'):
            b=self.b;b.step(phase)
            self.assertNotIn('speed',b._write.call_args.kwargs)
    def test_resume_resets_policy_queue_but_keeps_gripper_contact_guard(self):
        b=self.b;contact=b.contact
        b.resume()
        b.predictor.reset.assert_called_once();b.detector.reset.assert_called_once()
        self.assertIs(b.contact,contact);self.assertIsNotNone(b.camera_since)
        b._write.assert_not_called()
    def test_outage_blocks_old_inference_then_times_out(self):
        b=self.b;b.cameras.snapshot.side_effect=b.p.CameraUnavailable('stale')
        self.assertFalse(b.step('WORKING'))
        b.predictor.predict.assert_not_called()
        self.assertTrue(b._write.call_args.kwargs['holding'])
        b.camera_since=time.monotonic()-4
        with self.assertRaisesRegex(RuntimeError,'Camera recovery'):b.step('WORKING')
    def test_outage_after_prediction_does_not_send_prediction(self):
        b=self.b
        b.cameras.snapshot.side_effect=[{'overview':(None,0),'wrist':(None,0)},b.p.CameraUnavailable('stale')]
        self.assertFalse(b.step('WORKING'))
        self.assertEqual(b._write.call_count,1)
        self.assertEqual(b._write.call_args.args[0],b.current)
        self.assertTrue(b._write.call_args.kwargs['holding'])
    def test_shutdown_releases_and_closes_exactly_once(self):
        b=self.b;bus=b.bus;camera=b.cameras
        b.close();b.close()
        b.p.release.assert_called_once_with(bus);bus.close.assert_called_once();camera.close.assert_called_once()


if __name__=='__main__':unittest.main()
