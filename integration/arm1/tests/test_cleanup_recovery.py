"""No hardware: a past cleanup failure clears only after fresh verification."""
import tempfile
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from engine import Controller,MockBackend
from backend import HardwareBackend

class RecoveryTests(unittest.TestCase):
    def test_latch_survives_restart_and_failed_check_then_clears(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'events.jsonl'
            marker=path.parent/'MockBackend-cleanup-required.json';marker.write_text('{}')
            backend=MockBackend();backend.recover_cleanup=Mock(side_effect=RuntimeError('motor missing'))
            c=Controller(backend,log_path=path,period=.002)
            try:
                self.assertEqual(c.state,'ERROR');self.assertIsNone(c.reserve('HOME_A'))
                def clear():
                    ticket=c.reserve('CLEAR_ERROR');self.assertIsNotNone(ticket);c.activate(ticket)
                    self.assertTrue(ticket.done.wait(2));return ticket
                self.assertFalse(clear().success);self.assertTrue(marker.exists());self.assertEqual(c.state,'ERROR')
                backend.recover_cleanup.side_effect=None
                self.assertTrue(clear().success);self.assertFalse(marker.exists());self.assertEqual(c.state,'IDLE')
                self.assertFalse(backend.history);self.assertFalse(c.cleanup_errors)
            finally:
                c.cleanup_errors.clear();c.close()
            c=Controller(MockBackend(),log_path=path,period=.002)
            self.assertEqual(c.state,'IDLE');c.close()

    def make_hardware(self):
        h=HardwareBackend.__new__(HardwareBackend);h.bus=h.cameras=None;h.armed=False
        h.cfg={'robot':{'port':'fake'}};h.cal={};h.guard=object()
        bus=Mock();control=Mock();control.SerialBus.return_value=bus
        h.p=SimpleNamespace(control=control)
        return h,bus,control

    def test_probe_read_only_and_closes(self):
        h,b,c=self.make_hardware();h.recover_cleanup()
        self.assertEqual(c.verify.call_count,2);c.feedback.assert_called_once()
        b.open.assert_called_once();b.close.assert_called_once();b.write.assert_not_called();b.write_position_profile.assert_not_called()
        self.assertIsNone(h.bus);self.assertFalse(h.armed)

    def test_failures_retain_block_without_motor_writes(self):
        for target in ['open','verify','feedback','close']:
            h,b,c=self.make_hardware()
            getattr(b if target in ['open','close'] else c,target).side_effect=RuntimeError(target)
            with self.assertRaises(RuntimeError):h.recover_cleanup()
            b.close.assert_called_once();b.write.assert_not_called()
        h,b,c=self.make_hardware();h.armed=True
        with self.assertRaises(RuntimeError):h.recover_cleanup()
        b.open.assert_not_called()

if __name__=='__main__':unittest.main()
