import tempfile
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from engine import Controller,MockBackend,MotionBlocked,JobRejected
from backend import HardwareBackend,load_map


def until(predicate,timeout=3):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if predicate():return
        time.sleep(.002)
    raise AssertionError('Timed out waiting for test condition')


class EngineTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.backend=MockBackend(steps=30)
        self.c=Controller(self.backend,log_path=Path(self.temp.name)/'events.jsonl',period=.002)
        self.addCleanup(self.c.close)
    def issue(self,command):
        t=self.c.reserve(command);self.assertIsNotNone(t);self.c.activate(t);return t
    def finish(self,t):
        self.assertTrue(t.done.wait(3));return t.success
    def test_one_command_runs_one_set_then_returns_and_holds(self):
        t=self.issue('SELECT_A');self.assertTrue(self.finish(t))
        self.assertEqual([x for x in self.backend.history if x[0]=='begin'],[('begin','SELECT_A')])
        self.assertEqual(self.c.status()['state'],'IDLE')
        n=len(self.backend.history);time.sleep(.05);self.assertEqual(len(self.backend.history),n)
        self.assertIn(('step','RETURNING'),self.backend.history)
    def test_sequential_a_then_b(self):
        self.assertTrue(self.finish(self.issue('SELECT_A')))
        self.assertTrue(self.finish(self.issue('SELECT_B')))
        self.assertEqual([x[1] for x in self.backend.history if x[0]=='begin'],['SELECT_A','SELECT_B'])
    def test_already_complete_returns_before_success_for_both_models(self):
        self.backend.begin=Mock(return_value='PRECHECK')
        self.backend.already_complete=True
        for command in ('SELECT_A','SELECT_B'):
            self.backend.history.clear()
            t=self.issue(command)
            until(lambda:self.c.status()['phase']=='RETURNING')
            self.assertFalse(t.done.is_set())
            self.assertTrue(self.finish(t))
            phases={p for action,p in self.backend.history if action=='step'}
            self.assertEqual(phases,{'PRECHECK','RETURNING'})
    def test_incomplete_precheck_still_runs_normal_task(self):
        self.backend.begin=Mock(return_value='PRECHECK')
        self.backend.already_complete=False
        self.assertTrue(self.finish(self.issue('SELECT_B')))
        phases={p for action,p in self.backend.history if action=='step'}
        self.assertEqual(phases,{'PRECHECK','HOMING','WORKING','RETURNING'})
    def test_unknown_and_duplicate_rejected(self):
        self.assertIsNone(self.c.reserve('SELECT_C'))
        t=self.issue('SELECT_A');self.assertIsNone(self.c.reserve('SELECT_B'))
        self.assertTrue(self.finish(t))
    def test_completed_goal_can_be_requested_again_for_both_models(self):
        self.backend.begin=Mock(return_value='PRECHECK')
        self.backend.already_complete=True
        for command in ('SELECT_A','SELECT_B'):
            for _ in range(2):
                self.assertTrue(self.finish(self.issue(command)))
                self.assertEqual(self.c.status()['state'],'IDLE')
        self.assertEqual(self.backend.begin.call_count,4)
    def test_estop_ack_only_after_pose_confirmed_and_resume_same_job(self):
        task=self.issue('SELECT_B');until(lambda:self.backend.phase=='WORKING')
        ready=Mock(return_value=False);self.backend.hold_ready=ready
        stop=self.issue('EMER_STOP');until(lambda:self.c.status()['state']=='STOPPING')
        self.assertFalse(stop.done.is_set());self.assertFalse(task.done.is_set())
        ready.return_value=True;self.assertTrue(self.finish(stop))
        before=len(self.backend.history);time.sleep(.04);self.assertEqual(before,len(self.backend.history))
        self.assertIsNone(self.c.reserve('SELECT_A'))
        resume=self.issue('RESTART');self.assertTrue(self.finish(resume));self.assertTrue(self.finish(task))
        self.assertEqual(len([x for x in self.backend.history if x[0]=='begin']),1)
        self.assertIn(('resume','WORKING'),self.backend.history)
    def test_stop_during_return_resumes_return_not_work(self):
        task=self.issue('SELECT_A');until(lambda:self.c.status()['phase']=='RETURNING')
        self.assertTrue(self.finish(self.issue('EMER_STOP')))
        mark=len(self.backend.history)
        self.assertTrue(self.finish(self.issue('RESTART')));self.assertTrue(self.finish(task))
        self.assertFalse(any(x==('step','WORKING') for x in self.backend.history[mark:]))
    def test_stop_latches_before_execute_activation(self):
        stop=self.c.reserve('EMER_STOP');self.assertFalse(self.c.permitted())
        self.assertIsNone(self.c.reserve('SELECT_A'))
        until(lambda:self.c.status()['state']=='PAUSED');self.assertFalse(stop.done.is_set())
        self.c.activate(stop);self.assertTrue(self.finish(stop))
        self.assertTrue(self.finish(self.issue('RESTART')))
        self.assertEqual(self.c.status()['state'],'IDLE')
    def test_cancel_holds_and_reports_failure_not_success(self):
        task=self.issue('SELECT_A');until(lambda:self.c.status()['phase']=='WORKING')
        self.assertTrue(self.c.cancel(task));self.assertFalse(self.finish(task))
        self.assertEqual(self.c.status()['state'],'PAUSED')
        self.assertTrue(self.finish(self.issue('RESTART')))
        self.assertEqual(self.c.status()['state'],'IDLE')
    def test_fault_aborts_task_and_blocks_new_jobs(self):
        self.backend.step=Mock(side_effect=RuntimeError('motor fault'))
        t=self.issue('SELECT_B');self.assertFalse(self.finish(t))
        self.assertEqual(self.c.status()['state'],'ERROR');self.assertIsNone(self.c.reserve('SELECT_A'))
    def test_failed_precondition_preserves_existing_hold(self):
        self.backend.begin=Mock(side_effect=JobRejected('already complete'))
        self.backend.close=Mock()
        self.assertFalse(self.finish(self.issue('SELECT_A')))
        self.backend.close.assert_not_called();self.assertEqual(self.c.status()['state'],'IDLE')
    def test_watchdog_does_not_call_job_complete(self):
        self.c.timeout=.01
        self.assertFalse(self.finish(self.issue('SELECT_A')))
        self.assertEqual(self.c.status()['state'],'ERROR')
    def test_restart_without_pause_rejected(self):
        self.assertIsNone(self.c.reserve('RESTART'))
    def test_reserved_unactivated_goal_does_not_move(self):
        self.assertIsNotNone(self.c.reserve('SELECT_A'));time.sleep(.04)
        self.assertEqual(self.backend.history,[])


class AdapterTests(unittest.TestCase):
    def test_cleanup_error_is_reported_to_server_caller(self):
        with tempfile.TemporaryDirectory() as folder:
            backend=MockBackend();backend.close=Mock(side_effect=RuntimeError('torque-off failed'))
            c=Controller(backend,log_path=Path(folder)/'log.jsonl')
            with self.assertRaisesRegex(RuntimeError,'torque-off not confirmed'):c.close()
    def bare(self):
        b=HardwareBackend.__new__(HardwareBackend)
        b.p=Mock();b.p.control.JOINTS=['a','b','c'];b.p.MAX_INFERENCE_SECONDS=.25
        b.p.control.SERVO_PROFILE_SPEED=0;b.bus=Mock();b.permit=lambda:True
        return b
    def test_stop_discards_remaining_joint_writes(self):
        b=self.bare();b.permit=Mock(side_effect=[True,False])
        with self.assertRaises(MotionBlocked):b._write({'a':1,'b':2,'c':3},time.monotonic())
        self.assertEqual(b.bus.write_position_profile.call_count,1)
    def test_precheck_waits_for_stability_without_enabling_motors(self):
        b=self.bare();b._sample=Mock(return_value={})
        b._frames=Mock(return_value={'overview':('frame',1.0)})
        b.detector=Mock();b.detector.last={'candidate':True}
        b.detector.observe.side_effect=[False,True]
        self.assertFalse(b.step('PRECHECK'))
        self.assertFalse(b.already_complete)
        self.assertTrue(b.step('PRECHECK'))
        self.assertTrue(b.already_complete)
        b.bus.write.assert_not_called();b.bus.write_position_profile.assert_not_called()
    def test_precheck_false_candidate_does_not_report_already_complete(self):
        b=self.bare();b._sample=Mock(return_value={})
        b._frames=Mock(return_value={'overview':('frame',1.0)})
        b.detector=Mock();b.detector.last={'candidate':False}
        b.detector.observe.return_value=False
        self.assertTrue(b.step('PRECHECK'))
        self.assertFalse(b.already_complete)
    def test_hold_can_override_motion_inhibition(self):
        b=self.bare();b.permit=lambda:False
        b._write({'a':1,'b':2,'c':3},time.monotonic(),holding=True)
        self.assertEqual(b.bus.write_position_profile.call_count,3)
    def test_expired_inference_sends_no_goal(self):
        b=self.bare()
        with self.assertRaisesRegex(RuntimeError,'deadline'):b._write({},time.monotonic()-1)
        b.bus.write_position_profile.assert_not_called()
    def test_model_selection_is_pinned(self):
        c=load_map(Path(__file__).resolve().parents[1]/'model-map.json', require_assets=False)
        self.assertIn('model_A_newmask_300k/checkpoints/150000',c['models']['SELECT_A']['model'])
        self.assertIn('model_B2_300k_20261010/checkpoints/250000',c['models']['SELECT_B']['model'])
    def test_camera_outage_freezes_once_and_never_reuses_old_frames(self):
        b=self.bare()
        class Missing(Exception):pass
        b.p.CameraUnavailable=Missing;b.cameras=Mock();b.cameras.snapshot.side_effect=Missing()
        b.camera_since=b.camera_stable=b.camera_previous=None
        b.predictor=Mock();b.detector=Mock();b._freeze_at=Mock()
        now=time.monotonic()
        self.assertIsNone(b._frames({},now));self.assertIsNone(b._frames({},now))
        b._freeze_at.assert_called_once();self.assertEqual(b.predictor.reset.call_count,2)
        b.camera_since=time.monotonic()-4
        with self.assertRaisesRegex(RuntimeError,'Camera recovery'):b._frames({},time.monotonic())


if __name__=='__main__':unittest.main()
