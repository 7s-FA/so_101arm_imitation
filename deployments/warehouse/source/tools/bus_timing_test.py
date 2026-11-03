"""Read-only motor-bus timing test. Never enables torque or writes goals.

Repeats the server's armed-idle monitor at 30 Hz (6 feedback packets + 6
Torque_Enable reads per cycle) through the same DiagnosticBus/BoundedPort
(pyserial write_timeout 0.1 s). Phases separate USB load from in-process load:
  motor   motor bus only
  usb     both cameras streaming in a separate process (shared USB, no shared GIL)
  inproc  both cameras + the server's 2 Hz vision work in this process
A 1 ms sleep probe thread measures how late threads of this process get to run.

Stop the server first: this holds the same hardware locks. Report: logs/bus_timing_*.json
"""
import argparse
import bisect
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
ARM3 = ROOT/'arm3-current/arm3'
ARM1 = ROOT/'integration/arm1'
sys.path[:0] = [str(ARM3), str(ARM1)]
PERIOD = 1/30
DEADLINE_MS = 250  # Server monitor deadline (run_policy.MAX_INFERENCE_SECONDS).
SLOW_MS = 20
MAX_CONSECUTIVE_ERRORS = 30


def hardware_locks():
    held = []
    for path in (ROOT/'.hardware-session.lock', ARM1/'hardware.lock'):
        f = path.open('a')
        try: fcntl.flock(f, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError: sys.exit(f'관제 서버 또는 수동 실행이 켜져 있습니다 ({path.name}). 서버를 끈 뒤 실행하세요.')
        held.append(f)
    return held


def stats(values):
    if not values: return None
    s = sorted(values); n = len(s)
    q = lambda p: round(s[min(n-1, int(p*n))], 2)
    return {'n': n, 'p50': q(.5), 'p99': q(.99), 'p999': q(.999), 'max': round(s[-1], 2)}


def over(values, *limits):
    return {f'>={limit}ms': sum(v >= limit for v in values) for limit in limits}


def frame_counter(cameras):
    seen = {}
    def observe(name, rgb, stamp, metadata):
        c = seen.setdefault(name, {'frames': 0, 'max_gap_ms': 0.0, 'last': None})
        if c['last'] is not None: c['max_gap_ms'] = round(max(c['max_gap_ms'], (stamp-c['last'])*1000), 1)
        c['frames'] += 1; c['last'] = stamp
    cameras.add_listener(observe)
    return seen


def camera_summary(seen, seconds):
    return {name: {'frames': c['frames'], 'fps': round(c['frames']/seconds, 1), 'max_gap_ms': c['max_gap_ms']}
            for name, c in seen.items()}


def open_cameras():
    from record_purple import Cameras
    cameras = Cameras(json.loads((ARM3/'devices.json').read_text()), 30, reconnect=True)
    seen = frame_counter(cameras)
    cameras.start()
    return cameras, seen


def camera_worker():
    """Child process for the usb phase: same Cameras class as the server; runs until stdin closes."""
    cameras, seen = open_cameras()
    for c in seen.values(): c['frames'] = 0
    start = time.monotonic()
    print('READY', flush=True)
    try: sys.stdin.read()
    finally: cameras.close()
    print(json.dumps(camera_summary(seen, time.monotonic()-start)), flush=True)


class Probe:
    """Lateness of a 1 ms sleep: how long a thread of this process waits to run again."""
    def __init__(self):
        self.stop = threading.Event(); self.late = []; self.marks = []
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        while not self.stop.is_set():
            t = time.monotonic(); time.sleep(.001); end = time.monotonic()
            late = (end-t-.001)*1000
            self.late.append(late)
            if late >= 2: self.marks.append((end, late))

    def annotate(self, events):
        """After a phase: worst probe lateness ending during each event (+50 ms, the stalled sleep wakes after it)."""
        marks = list(self.marks); times = [t for t, _ in marks]
        for e in events:
            start, end = e.pop('_t0'), e.pop('_t1')
            i = bisect.bisect_left(times, start)
            worst = 0.0
            while i < len(marks) and marks[i][0] <= end+.05:
                worst = max(worst, marks[i][1]); i += 1
            e['probe_late_ms'] = round(worst, 1)


class VisionLoad:
    """The server's 2 Hz /arm1/vision callback (judge, jig cells, annotate, JPEG, JSON), without ROS."""
    def __init__(self, cameras):
        import cv2
        from vision_judge import VisionJudge
        from jig_cells import JigCells
        self.cv2, self.cameras = cv2, cameras
        self.judge = VisionJudge()
        self.jig = JigCells(json.loads((ARM1/'model-map.json').read_text())['burger_correction'], self.judge.config['cells'])
        self.stop = threading.Event(); self.ms = []; self.errors = []
        self.thread = threading.Thread(target=self.run, daemon=True)

    def run(self):
        cv2 = self.cv2
        while not self.stop.wait(.5):
            t = time.monotonic()
            try:
                frame = self.cameras.snapshot()['overview'][0]
                result = self.judge.evaluate(frame)
                preview = {c: self.judge.select(c, result) for c in ('SELECT_A', 'SELECT_B')}
                try: live = self.jig.measure(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                except Exception as exc: live = {'ok': False, 'reason': repr(exc)}
                cv2.imencode('.jpg', self.judge.annotate(frame, result, None), [cv2.IMWRITE_JPEG_QUALITY, 80])
                json.dumps({'sets': result['sets'], 'targets': result['targets'], 'cells': result['cells'],
                            'preview': preview, 'burger_live': live}, ensure_ascii=False, default=str)
            except Exception as exc:
                self.errors.append(repr(exc))
            self.ms.append((time.monotonic()-t)*1000)


def motor_phase(bus, seconds, probe):
    from player.program import JOINTS
    timing = {'write_ms': None}
    original = bus.port.writePort
    def timed_write(packet):
        t = time.monotonic()
        try: return original(packet)
        finally: timing['write_ms'] = (time.monotonic()-t)*1000
    bus.port.writePort = timed_write
    writes, txns, cycles, periods, errors, slow, torque_on, status, volts = [], [], [], [], [], [], [], [], []
    consecutive = 0; previous = None
    start = time.monotonic()
    try:
        while time.monotonic()-start < seconds:
            tick = time.monotonic()
            if previous is not None: periods.append((tick-previous)*1000)
            previous = tick
            for op, joint in [('feedback', j) for j in JOINTS]+[('torque', j) for j in JOINTS]:
                timing['write_ms'] = None; error = None
                t0 = time.monotonic()
                try:
                    if op == 'feedback':
                        value = bus.feedback(joint)
                        volts.append(value['voltage_V'])
                        if value['status']: status.append({'t': round(t0-start, 3), 'joint': joint, 'status': value['status']})
                    elif bus.read(joint, 'Torque_Enable') != 0:
                        torque_on.append({'t': round(t0-start, 3), 'joint': joint})
                except Exception as exc:
                    error = repr(exc)
                t1 = time.monotonic(); ms = (t1-t0)*1000
                txns.append(ms)
                if timing['write_ms'] is not None: writes.append(timing['write_ms'])
                if error or ms >= SLOW_MS:
                    event = {'t': round(t0-start, 3), 'joint': joint, 'op': op, 'ms': round(ms, 1),
                             'write_ms': None if timing['write_ms'] is None else round(timing['write_ms'], 1),
                             'error': error, '_t0': t0, '_t1': t1}
                    (errors if error else slow).append(event)
                consecutive = consecutive+1 if error else 0
                if consecutive >= MAX_CONSECUTIVE_ERRORS:
                    raise RuntimeError(f'{consecutive} consecutive bus errors; last: {error}')
            cycles.append((time.monotonic()-tick)*1000)
            time.sleep(max(0, PERIOD-(time.monotonic()-tick)))
    finally:
        bus.port.writePort = original
    elapsed = time.monotonic()-start
    time.sleep(.1)  # Let the probe record a sleep that was stalled together with the last event.
    probe.annotate(errors+slow)
    return {'seconds': round(elapsed, 1), 'transactions': len(txns), 'errors': errors[:500],
            'error_count': len(errors), 'write_timeouts': sum('Write timeout' in e['error'] for e in errors),
            'write_ms': stats(writes), 'write_over': over(writes, 5, 20, 50, 100),
            'transaction_ms': stats(txns), 'transaction_over': over(txns, 20, 34, 50, 100),
            'cycle_ms': stats(cycles), 'cycle_over': over(cycles, 33, 100, DEADLINE_MS),
            'cycle_period_ms': stats(periods), 'slow_transactions': slow[:200], 'slow_count': len(slow),
            'torque_on': torque_on, 'status_nonzero': status[:50],
            'servo_voltage_V': {'min': min(volts), 'max': max(volts)} if volts else None}


def run_phase(name, bus, seconds):
    probe = Probe(); cameras = worker = vision = None; seen = None
    usage = resource.getrusage(resource.RUSAGE_SELF); child_usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    try:
        if name == 'usb':
            worker = subprocess.Popen([sys.executable, '-u', __file__, '--camera-worker'], cwd=ROOT,
                                      stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
            lines = []; ready = threading.Event()
            def read():
                for line in worker.stdout:
                    lines.append(line.rstrip())
                    if line.startswith('READY'): ready.set()
                ready.set()
            reader = threading.Thread(target=read, daemon=True); reader.start()
            if not ready.wait(25) or worker.poll() is not None:
                raise RuntimeError('camera worker did not start: '+' | '.join(lines[-5:]))
        elif name == 'inproc':
            cameras, seen = open_cameras()
            vision = VisionLoad(cameras)
            for c in seen.values(): c['frames'] = 0; c['max_gap_ms'] = 0.0
            vision.thread.start()
        probe.thread.start()
        print(f'[{name}] {seconds:.0f}s 측정 시작', flush=True)
        result = motor_phase(bus, seconds, probe)
    finally:
        probe.stop.set()
        if vision: vision.stop.set(); vision.thread.join(timeout=5)
        if cameras: cameras.close()
        if worker:
            try: worker.stdin.close()
            except OSError: pass
            try: worker.wait(timeout=15)
            except subprocess.TimeoutExpired: worker.kill(); worker.wait()
    after = resource.getrusage(resource.RUSAGE_SELF)
    cpu = after.ru_utime+after.ru_stime-usage.ru_utime-usage.ru_stime
    result.update(phase=name, process_cpu_cores=round(cpu/result['seconds'], 2),
                  probe_late_ms=stats(probe.late), probe_over=over(probe.late, 5, 20, 50, 100))
    if worker:
        child = resource.getrusage(resource.RUSAGE_CHILDREN)
        result['camera_process_cpu_cores'] = round((child.ru_utime+child.ru_stime-child_usage.ru_utime-child_usage.ru_stime)/result['seconds'], 2)
        reader.join(timeout=5)
        summary = next((l for l in reversed(lines) if l.startswith('{')), None)
        result['cameras'] = json.loads(summary) if summary else None
        result['camera_worker_log'] = [l for l in lines if not l.startswith(('{', 'READY'))][-20:]
    if seen is not None:
        result['cameras'] = camera_summary(seen, result['seconds'])
        result['vision_ms'] = stats(vision.ms); result['vision_errors'] = vision.errors[:20]
    return result


def summary_line(r):
    return (f"[{r['phase']}] 거래 {r['transactions']} | 오류 {r['error_count']} (Write timeout {r['write_timeouts']}) | "
            f"쓰기 max {r['write_ms']['max'] if r['write_ms'] else '-'}ms | 거래 p99.9 {r['transaction_ms']['p999']}ms max {r['transaction_ms']['max']}ms | "
            f"주기>250ms {r['cycle_over'][f'>={DEADLINE_MS}ms']} | 탐침 max {r['probe_late_ms']['max']}ms | CPU {r['process_cpu_cores']}코어")


def interrupt(*_):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seconds', type=float, default=300, help='seconds per phase')
    parser.add_argument('--phases', default='motor,usb,inproc')
    parser.add_argument('--camera-worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.camera_worker: return camera_worker()
    phases = args.phases.split(',')
    if set(phases)-{'motor', 'usb', 'inproc'}: parser.error('phases: motor, usb, inproc')
    signal.signal(signal.SIGTERM, interrupt)
    locks = hardware_locks()
    import teleoperate_slow as control
    cfg = json.loads((ARM3/'devices.json').read_text())
    calibration = json.loads((ARM3/'calibration/arm3.json').read_text())
    out = ROOT/'logs'/f'bus_timing_{datetime.now():%Y%m%d_%H%M%S}.json'
    report = {'started': datetime.now().astimezone().isoformat(), 'seconds_per_phase': args.seconds,
              'phases': [], 'read_only': True,
              'usb_topology': subprocess.run(['lsusb', '-t'], capture_output=True, text=True).stdout}
    began = time.time()
    bus = control.SerialBus(control.PortConfig(cfg['robot']['port'], '5B3D046080', 'robot'))
    bus.open()
    try:
        # Calibration/model registers and Torque_Enable == 0 on all joints; raises before any timing if torque is on.
        control.verify(bus, calibration)
        print('모터 버스 열림, 6관절 토크 OFF 확인. 읽기만 합니다.', flush=True)
        for name in phases:
            try: result = run_phase(name, bus, args.seconds)
            except KeyboardInterrupt: raise
            except Exception as exc: result = {'phase': name, 'aborted': repr(exc)}
            report['phases'].append(result)
            out.write_text(json.dumps(report, ensure_ascii=False, indent=1))
            print(summary_line(result) if 'aborted' not in result else f"[{name}] 중단: {result['aborted']}", flush=True)
    finally:
        bus.close()
        kernel = subprocess.run(['journalctl', '-k', '--since', f'@{int(began)}', '--no-pager', '-q'],
                                capture_output=True, text=True).stdout.splitlines()
        report['kernel_usb_events'] = [l for l in kernel if any(k in l.lower() for k in ('usb', 'tty', 'uvc', 'xhci', 'acm'))][-50:]
        report['finished'] = datetime.now().astimezone().isoformat()
        out.write_text(json.dumps(report, ensure_ascii=False, indent=1))
        print(f'보고서: {out}', flush=True)
        del locks


if __name__ == '__main__':
    sys.exit(main())
