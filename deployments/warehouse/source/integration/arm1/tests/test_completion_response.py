"""Deterministic result timing and continued-return tests, without hardware."""
import tempfile
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from engine import Controller, MockBackend


class CompletionResponseTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.clock = [100.0]
        clock = patch('engine.time.monotonic', side_effect=lambda: self.clock[0])
        clock.start(); self.addCleanup(clock.stop)
        self.backend = MockBackend(steps=1)
        with patch('engine.threading.Thread.start'):
            self.c = Controller(self.backend, log_path=Path(self.tmp.name)/'events.jsonl')
        self.addCleanup(self.c.log.close)

    def issue(self, command):
        ticket = self.c.reserve(command)
        self.assertIsNotNone(ticket)
        self.c.activate(ticket)
        return ticket

    def tick(self, at):
        self.clock[0] = at
        self.c._cycle(.01)

    def placement(self, command):
        job = self.issue(command)
        self.tick(100); self.tick(101); self.tick(102)
        self.assertEqual(self.c.phase, 'RETURNING')
        self.assertEqual(self.c.placement_confirmed_at, 102)
        return job

    def test_five_seconds_after_placement_even_if_return_is_still_running(self):
        for command in ('SELECT_A', 'SELECT_B'):
            with self.subTest(command=command):
                job = self.placement(command)
                original = self.backend.step
                self.backend.step = lambda phase: False
                self.tick(106.999); self.assertFalse(job.done.is_set())
                self.tick(107); self.assertTrue(job.done.is_set()); self.assertTrue(job.success)
                self.assertIs(self.c.job, job)
                self.assertEqual(self.c.state, 'RUNNING')
                self.assertTrue(self.c.status()['delivery_reported'])
                self.assertFalse(self.c.status()['ready_for_new_job'])
                self.assertIsNone(self.c.reserve('SELECT_B'))
                self.backend.step = original
                self.tick(108)
                self.assertEqual(self.c.state, 'IDLE'); self.assertIsNone(self.c.job)

    def test_early_home_arrival_waits_for_five_seconds_without_more_motion(self):
        job = self.placement('SELECT_B')
        self.tick(103)
        self.assertEqual(self.c.phase, 'COMPLETION_WAIT')
        count = len(self.backend.history)
        self.tick(106.99); self.assertFalse(job.done.is_set())
        self.tick(107); self.assertTrue(job.success)
        self.assertEqual(len(self.backend.history), count)
        self.assertEqual(self.c.state, 'IDLE')

    def test_estop_defers_result_until_resume_and_keeps_return_phase(self):
        job = self.placement('SELECT_A')
        stop = self.issue('EMER_STOP')
        self.tick(103); self.tick(104)
        self.assertTrue(stop.success)
        self.tick(110); self.assertFalse(job.done.is_set())
        resume = self.issue('RESTART'); self.tick(111)
        self.assertTrue(resume.success); self.assertFalse(job.done.is_set())
        self.tick(112); self.assertTrue(job.success)
        self.assertEqual([a for a in self.backend.history if a[0]=='begin'], [('begin', 'SELECT_A')])

    def test_cancel_before_response_never_reports_success(self):
        job = self.placement('SELECT_A')
        self.assertTrue(self.c.cancel(job))
        self.tick(108); self.tick(109)
        self.assertTrue(job.done.is_set()); self.assertFalse(job.success)

    def test_estop_and_resume_still_control_return_after_result_was_sent(self):
        job = self.placement('SELECT_B')
        step = self.backend.step
        self.backend.step = lambda phase: False
        self.tick(107); self.assertTrue(job.success)
        stop = self.issue('EMER_STOP')
        self.tick(108); self.tick(109)
        self.assertTrue(stop.success); self.assertEqual(self.c.state, 'PAUSED')
        self.assertIsNone(self.c.reserve('SELECT_A'))
        resume = self.issue('RESTART'); self.tick(110)
        self.assertTrue(resume.success); self.assertEqual(self.c.phase, 'RETURNING')
        self.backend.step = step
        self.tick(111)
        self.assertIsNone(self.c.job); self.assertEqual(self.c.state, 'IDLE')

    def test_home_only_goal_still_waits_for_arrival_without_placement_delay(self):
        job = self.issue('HOME_A')
        self.tick(100); self.assertFalse(job.done.is_set())
        self.tick(101); self.assertTrue(job.success)
        self.assertIsNone(self.c.placement_confirmed_at)

    def test_ready_status_excludes_pending_control_requests(self):
        self.assertTrue(self.c.status()['ready_for_new_job'])
        control = self.issue('CLEAR_ERROR')
        self.assertFalse(self.c.status()['ready_for_new_job'])
        self.tick(100)
        self.assertTrue(control.success)
        self.assertTrue(self.c.status()['ready_for_new_job'])

    def test_failure_cleanup_can_release_job_after_an_early_success(self):
        job = self.placement('SELECT_A')
        self.backend.step = lambda phase: False
        self.tick(107); self.assertTrue(job.success)
        self.c._state('ERROR')
        self.c._finish(job, False, 'return failed after delivery report')
        self.assertIsNone(self.c.job)
        self.assertTrue(job.success)  # Already sent results cannot be retracted.
        self.assertIsNone(self.c.reserve('SELECT_B'))


if __name__ == '__main__':
    unittest.main()
