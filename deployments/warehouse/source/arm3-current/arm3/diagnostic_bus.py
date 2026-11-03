"""Arm3 diagnostics with a scoped persistence window for leader gripper undervoltage."""
import time

from player.hardware import SerialBus as BaseSerialBus
from player.program import JOINTS, PlayerError

LOW_VOLTAGE_POLICY = {"role":"leader", "joint":"gripper", "threshold_V":4.0,
                      "duration_s":5.0, "max_sample_gap_s":1.5}


class DiagnosticBus(BaseSerialBus):
    def __init__(self, config):
        super().__init__(config)
        self.last_transaction = None
        self.last_feedback = {}
        self.fault_events = []
        self.low_voltage_since = None
        self.low_voltage_last_seen = None

    def _exchange(self, function, *args):
        motor_id, address = args[:2]
        joint = JOINTS[motor_id - 1]
        event = {'monotonic_s': time.monotonic(), 'joint': joint, 'motor_id': motor_id,
                 'operation': function.__name__, 'address': address,
                 'port': self.config.port, 'role': self.config.role}
        self.last_transaction = event
        try:
            result = super()._exchange(function, *args)
        except Exception as exc:
            event['exception'] = repr(exc)
            self.fault_events.append(event)
            raise PlayerError(f'{self.config.role}/{joint}: {function.__name__} '
                              f'address={address}: {exc}') from exc
        comm, error = result[-2:]
        event.update(comm=comm, error_bits=error)
        if len(result) == 3:
            # Keep the returned data even when a valid packet carries an alarm.
            event['response'] = result[0]
        if comm == 0 and function.__name__ == 'readTxRx' and address == 56 and len(result[0]) == 10:
            data = result[0]
            event.update(voltage_raw=data[6], voltage_V=data[6] / 10,
                         temperature=data[7], status=data[9])
        if comm != 0 or error != 0:
            if joint in self.last_feedback:
                # A write acknowledgement has no voltage: this is an earlier reading.
                event['previous_feedback'] = dict(self.last_feedback[joint])
            self.fault_events.append(event)
        return result

    def _observe_low_voltage(self, event):
        now = event['monotonic_s']
        if event['voltage_V'] > LOW_VOLTAGE_POLICY['threshold_V']:
            self.low_voltage_since = self.low_voltage_last_seen = None
            return False
        previous = self.low_voltage_last_seen
        if previous is not None and (now < previous or now-previous > LOW_VOLTAGE_POLICY['max_sample_gap_s']):
            raise PlayerError('gripper: low-voltage monitoring gap; continuous readings unavailable')
        if self.low_voltage_since is None: self.low_voltage_since = now
        self.low_voltage_last_seen = now
        elapsed = now-self.low_voltage_since
        event['low_voltage_elapsed_s'] = elapsed
        event['low_voltage_pending'] = elapsed < LOW_VOLTAGE_POLICY['duration_s']
        if not event['low_voltage_pending']:
            raise PlayerError(f'gripper: continuous low voltage {elapsed:.2f}s >= '
                              f'{LOW_VOLTAGE_POLICY["duration_s"]:g}s')
        return True

    def _check(self, joint, comm, error):
        event = self.last_transaction or {}
        try:
            scoped = self.config.role == LOW_VOLTAGE_POLICY['role'] and joint == LOW_VOLTAGE_POLICY['joint']
            if scoped and comm == 0:
                if 'voltage_V' in event and not (error & ~1 or event['status'] & ~1):
                    if self._observe_low_voltage(event):
                        # Defer only the voltage bit on a valid, currently low-voltage packet.
                        event['voltage_alarm_deferred'] = bool(error & 1 or event['status'] & 1)
                        super()._check(joint,comm,error & ~1)
                        return
                elif error == 1 and event.get('operation') in ('read1ByteTxRx','read2ByteTxRx'):
                    # Calibration/register reads have no voltage. Confirm with a fresh full
                    # feedback packet; never defer using a stale cached voltage or a write ACK.
                    confirmed = self.feedback(joint)
                    self.last_transaction = event
                    if confirmed.get('low_voltage_pending'):
                        event['voltage_alarm_deferred'] = True
                        event['voltage_confirmation'] = dict(self.last_feedback[joint])
                        return
            super()._check(joint,comm,error)
        except PlayerError as exc:
            event['check_error'] = str(exc)
            if not self.fault_events or self.fault_events[-1] is not event:
                self.fault_events.append(event)
            details = f'operation={event.get("operation")} address={event.get("address")} error_bits=0x{error:02x}'
            if 'voltage_V' in event:
                details += f' voltage={event["voltage_V"]:.1f}V'
            raise PlayerError(f'{self.config.role}/{exc} ({details})') from exc

    def feedback(self, joint):
        value = super().feedback(joint)
        event = self.last_transaction
        value['voltage_V'] = event['voltage_V']
        if event.get('low_voltage_pending'):
            value.update(status_raw=value['status'], error_bits_raw=event['error_bits'],
                         low_voltage_pending=True, low_voltage_elapsed_s=event['low_voltage_elapsed_s'])
            value['status'] &= ~1
        self.last_feedback[joint] = {'monotonic_s': event['monotonic_s'], **value}
        return value
