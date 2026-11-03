"""Single owner of robot hardware; ROS threads only reserve/control jobs."""
from dataclasses import dataclass, field
from pathlib import Path
import json
import math
import threading
import time
import uuid

JOBS = {'SELECT_A', 'SELECT_B', 'HOME_A', 'HOME_B'}
CONTROLS = {'EMER_STOP', 'RESTART', 'CLEAR_ERROR'}


class MotionBlocked(Exception):
    pass


class JobRejected(Exception):
    """A precondition failed; keep an already held arm powered and stationary."""


@dataclass
class Ticket:
    command: str
    id: str = field(default_factory=lambda: uuid.uuid4().hex)
    activated: bool = False
    cancelled: bool = False
    done: threading.Event = field(default_factory=threading.Event)
    success: bool = False
    detail: str = ''


class Controller:
    def __init__(self, backend, *, log_path, timeout=600, period=1/30, completion_delay=5.0):
        if not math.isfinite(completion_delay) or completion_delay < 0:
            raise ValueError('completion_delay must be finite and nonnegative')
        self.backend, self.timeout, self.period = backend, timeout, period
        self.completion_delay = completion_delay
        self.placement_confirmed_at = None
        self.return_arrived = False
        self.lock = threading.RLock()
        self.pause = threading.Event()
        self.closing = threading.Event()
        self.state = 'IDLE'
        self.phase = None
        self.job = self.control = None
        self.active_seconds = 0
        self.stop_since = None
        self.cleanup_marker = Path(log_path).parent / (type(backend).__name__+'-cleanup-required.json')
        self.cleanup_errors = []
        if self.cleanup_marker.exists():
            self.cleanup_errors = ['previous hardware cleanup requires fresh verification']
            self.state = 'ERROR'
        self.log = open(log_path, 'a', buffering=1)
        self.backend.permit = self.permitted
        self.thread = threading.Thread(target=self._run, name='arm1-hardware', daemon=True)
        self.thread.start()

    def event(self, name, **data):
        self.log.write(json.dumps({'time':time.time(), 'event':name, **data}, ensure_ascii=False)+'\n')

    def permitted(self):
        return not self.pause.is_set() and not self.closing.is_set()

    def reserve(self, command):
        with self.lock:
            if self.closing.is_set() or (self.state == 'ERROR' and command != 'CLEAR_ERROR'): return None
            if command in JOBS:
                if self.job or self.control or self.state != 'IDLE' or self.pause.is_set(): return None
                ticket = self.job = Ticket(command)
            elif command in CONTROLS:
                if self.control: return None
                if command == 'RESTART' and self.state != 'PAUSED': return None
                if command == 'CLEAR_ERROR' and (self.state not in ('IDLE','PAUSED','ERROR')
                        or self.state == 'IDLE' and self.job is not None): return None
                ticket = self.control = Ticket(command)
                # Latch at goal reception, before waiting for execute_callback.
                if command == 'EMER_STOP': self.pause.set()
            else:
                return None
            self.event('reserved', command=command, ticket=ticket.id)
            return ticket

    def activate(self, ticket):
        with self.lock: ticket.activated = True

    def cancel(self, ticket):
        with self.lock:
            if ticket is not self.job or ticket.done.is_set(): return False
            ticket.cancelled = True
            self.pause.set()
            return True

    def status(self):
        with self.lock:
            return {'state': self.state, 'phase': self.phase,
                    'command': self.job.command if self.job else None,
                    'delivery_reported': bool(self.job and self.job.done.is_set()),
                    'ready_for_new_job': (self.state == 'IDLE' and self.job is None
                                          and self.control is None and self.permitted()),
                    'supports_error_reset': True, 'supports_home': True}

    def _state(self, state):
        with self.lock:
            if self.state != state:
                self.state = state
                self.event('state', **self.status())

    def _finish(self, ticket, success, detail, *, retain_job=False):
        if ticket is None: return
        if ticket.done.is_set():
            if ticket is self.job and not retain_job: self.job = None
            return
        ticket.success, ticket.detail = bool(success), detail
        self.event('result', ticket=ticket.id, command=ticket.command, success=bool(success), detail=detail)
        ticket.done.set()
        if ticket is self.job and not retain_job: self.job = None
        if ticket is self.control: self.control = None

    def _placement_confirmed(self):
        self.placement_confirmed_at = time.monotonic()
        self.event('placement_confirmed', command=self.job.command,
                   response_delay_s=self.completion_delay)

    def _report_delivery(self, job):
        # Keep ownership until arrival even after the ROS action result is sent.
        if (self.placement_confirmed_at is not None and self.permitted()
                and time.monotonic()-self.placement_confirmed_at >= self.completion_delay
                and not job.done.is_set()):
            self._finish(job, True, 'placement confirmed; completion delay elapsed; '
                         + ('home reached' if self.return_arrived else 'return continues; new jobs blocked'),
                         retain_job=True)
        if self.return_arrived and job.done.is_set():
            self.phase = None
            self._finish(job, True, 'return complete')
            self._state('IDLE')
            self.event('return_complete', ticket=job.id, completed_command=job.command, **self.status())

    def _cycle(self, elapsed):
        with self.lock:
            control = self.control
            if control and control.activated and control.command == 'CLEAR_ERROR':
                # Explicit cleanup must also release sessions retained in IDLE/PAUSED.
                # Only the hardware worker may close/reopen devices.
                try:
                    if self.state != 'ERROR' and not self.cleanup_errors:
                        self.backend.monitor()
                        if not self.backend.hold_ready():
                            self._finish(control, False, 'stopped pose not confirmed')
                            return
                    reset_resources=getattr(self.backend, 'reset_resources', None)
                    if reset_resources is not None:
                        reset_resources()
                    elif self.cleanup_errors or self.state == 'ERROR':
                        recover=getattr(self.backend, 'recover_cleanup', None)
                        if recover is None and self.cleanup_errors:
                            raise RuntimeError('hardware cleanup unconfirmed; inspect locally')
                        if recover is not None:recover()
                        else:self.backend.close()
                except Exception as exc:
                    self.cleanup_errors.append(repr(exc))
                    self.cleanup_marker.write_text(json.dumps({'reason':repr(exc)},ensure_ascii=False))
                    self._state('ERROR')
                    self.event('recovery_failed', detail=repr(exc))
                    self._finish(control, False, 'hardware recovery verification failed: '+str(exc))
                    return
                self.cleanup_marker.unlink(missing_ok=True)
                self.cleanup_errors.clear()
                self.event('cleanup_verified', detail='owned resources released; fresh motor checks passed; no movement requested')
                clear_state=getattr(self.backend, 'clear_recovery_state', None)
                if clear_state is not None:clear_state()
                self._finish(self.job, False, 'discarded by CLEAR_ERROR; no resume or home motion')
                self.phase = None
                self.placement_confirmed_at = None
                self.return_arrived = False
                self.active_seconds = 0
                self.stop_since = None
                self.pause.clear()
                self._state('IDLE')
                self._finish(control, True, 'error cleared; no movement requested')
                return
        if self.pause.is_set() and self.state not in ('STOPPING','PAUSED'):
            self.backend.freeze()
            self.stop_since = time.monotonic()
            self._state('STOPPING')
        if self.state == 'STOPPING':
            self.backend.monitor()
            if self.backend.hold_ready():
                self._state('PAUSED')
            elif time.monotonic()-self.stop_since > 2:
                raise RuntimeError('Could not confirm stopped pose within 2 seconds')
            return
        if self.state == 'PAUSED':
            self.backend.monitor()
            with self.lock:
                if self.job and self.job.cancelled:
                    self._finish(self.job, False, 'cancelled; held with torque on')
                    self.phase = None
                control = self.control
                if control and control.activated:
                    if control.command == 'EMER_STOP':
                        self._finish(control, True, 'stopped pose confirmed; torque retained')
                    elif control.command == 'RESTART':
                        self.backend.resume()
                        self.pause.clear()
                        self._state('RUNNING' if self.phase else 'IDLE')
                        self._finish(control, True, 'resume acknowledged; original job result still pending')
            return
        with self.lock:
            job = self.job
        if not job or not job.activated:
            self.backend.monitor()
            return
        if self.phase is None:
            self._state('RUNNING')
            initial_phase=self.backend.begin(job.command)  # Read only; never enable torque here.
            self.phase = initial_phase or 'HOMING'
            self.active_seconds = 0
            self.placement_confirmed_at = None
            self.return_arrived = False
            return
        self.active_seconds += elapsed
        if self.active_seconds > self.timeout:
            raise RuntimeError('Job watchdog expired before completion and return')
        if self.phase == 'COMPLETION_WAIT':
            self.backend.monitor()
            if self.placement_confirmed_at is None and getattr(self.backend, 'verify_after_return', False):
                # Plan jobs: confirm the placement with the arm back home (camera view unobstructed).
                verdict = self.backend.verify_placement()
                if verdict is False: raise JobRejected(self.backend.verify_failure)
                if verdict is True:
                    with self.lock: self._placement_confirmed()
            with self.lock:
                self._report_delivery(job)
            return
        if self.backend.step(self.phase):
            with self.lock:
                if not self.permitted(): return  # A stop wins over completion/result.
                if self.phase == 'PRECHECK':
                    self.phase = 'RETURNING' if self.backend.already_complete else 'HOMING'
                    if self.backend.already_complete: self._placement_confirmed()
                elif self.phase == 'HOMING': self.phase = 'WORKING'
                elif self.phase == 'WORKING':
                    if not getattr(self.backend, 'verify_after_return', False): self._placement_confirmed()
                    self.phase = 'RETURNING'
                else:
                    self.backend.freeze()
                    if job.command.startswith('HOME_'):
                        self.phase = None
                        self._state('IDLE')
                        self._finish(job, True, 'saved start pose reached; holding torque')
                    else:
                        self.return_arrived = True
                        self.phase = 'COMPLETION_WAIT'
                self.event('phase', **self.status())
        with self.lock:
            self._report_delivery(job)

    def _run(self):
        previous = time.monotonic()
        try:
            while not self.closing.is_set():
                tick = time.monotonic()
                try:
                    if self.state != 'ERROR' or (self.control and self.control.command == 'CLEAR_ERROR'):
                        self._cycle(tick-previous)
                except MotionBlocked:
                    pass  # Drop stale inference/goals; next iteration freezes once.
                except JobRejected as exc:
                    with self.lock:
                        self.phase = None
                        self._state('IDLE')
                        self._finish(self.job, False, str(exc))
                except Exception as exc:
                    self._state('ERROR')
                    self.event('fault', detail=repr(exc))
                    try: self.backend.close()
                    except Exception as cleanup:
                        self.cleanup_errors.append(repr(cleanup))
                        self.event('cleanup_error', detail=repr(cleanup))
                    self.cleanup_marker.write_text(json.dumps({'reason':repr(exc)},ensure_ascii=False))
                    with self.lock:
                        self._finish(self.job, False, repr(exc))
                        self._finish(self.control, False, repr(exc))
                previous = tick
                self.closing.wait(max(0, self.period-(time.monotonic()-tick)))
        finally:
            try: self.backend.close()
            except Exception as exc:
                self.cleanup_errors.append(repr(exc))
                self.event('cleanup_error', detail=repr(exc))
            with self.lock:
                self._finish(self.job, False, 'server shutdown')
                self._finish(self.control, False, 'server shutdown')
            self.event('shutdown')

    def close(self):
        self.closing.set()
        self.thread.join(timeout=30)
        if self.thread.is_alive(): raise RuntimeError('Hardware worker did not stop; inspect robot locally')
        self.log.close()
        if self.cleanup_errors:
            raise RuntimeError('Robot cleanup/torque-off not confirmed: '+repr(self.cleanup_errors))


class MockBackend:
    """No robot/camera/ML imports; supports real ROS round-trip tests."""
    def __init__(self, steps=30):
        self.steps, self.phase, self.count = steps, None, 0
        self.permit = lambda:True
        self.history = []
    def begin(self, command):
        self.history.append(('begin',command));self.phase=None;self.count=0
        if command.startswith('HOME_'):return 'RETURNING'
    def monitor(self): pass
    def freeze(self): self.history.append(('freeze',self.phase))
    def hold_ready(self): return True
    def resume(self): self.history.append(('resume',self.phase))
    def close(self): pass
    def step(self, phase):
        if not self.permit(): raise MotionBlocked()
        if phase != self.phase: self.phase=phase;self.count=0
        self.count+=1
        self.history.append(('step',phase))
        return self.count >= self.steps
