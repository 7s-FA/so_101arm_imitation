"""Replay of a verified real-robot plan (factory_simulation real_*_plan.py output) inside the Arm1 server.

The plan is a raw-tick command list at `command_hz`. The server's 30Hz worker asks for the command at the
elapsed plan time (linear interpolation between frames); the clock only runs while the worker is stepping,
so EMER_STOP/RESTART resume where the plan stopped. Same real-robot rules as real_lid_run.py:
  joint_tracking: Isaac step86 PI bias on the five arm joints from start_index, clipped 2 ticks inside range
  gripper_squeeze_limit_units: the closing gripper command never leads the measured gripper by more than this
  tracking watchdog: an arm joint more than 12 deg from the plan for more than 0.5s is a fault
The server's own TeachingTargets bound (10 deg / 7 points from measured) is still applied after this.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')
TRACK_LIMIT_DEG, TRACK_GRACE_S = 12., .5


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class PlanRunner:
    def __init__(self, path, calibration):
        self.path = Path(path)
        plan = json.loads(self.path.read_text())
        if tuple(plan['joint_order']) != JOINTS: raise ValueError(f'{path}: unexpected joint order')
        if json.loads(Path(plan['robot_calibration']).read_text()) != calibration:
            raise ValueError(f'{path}: built for a different robot calibration')
        if sha(plan['robot_calibration']) != plan['robot_calibration_sha256']:
            raise ValueError(f'{path}: calibration file changed since the plan was built')
        self.raw = np.array(plan['raw'], float)
        lo = np.array([calibration[j]['range_min'] for j in JOINTS]); hi = np.array([calibration[j]['range_max'] for j in JOINTS])
        if np.any(self.raw < lo) or np.any(self.raw > hi): raise ValueError(f'{path}: commands outside the calibrated range')
        self.lo, self.hi = lo, hi
        self.hz = float(plan['command_hz'])
        if not 5 <= self.hz <= 30: raise ValueError(f'{path}: command rate {self.hz}')
        self.scale = np.array([4095/360]*5 + [(hi[5]-lo[5])/100])
        self.tracking = plan.get('joint_tracking')
        squeeze = plan.get('gripper_squeeze_limit_units')
        self.squeeze = None if squeeze is None else (squeeze if isinstance(squeeze, dict)
                                                     else {'units': float(squeeze), 'start_index': 0, 'end_index': len(self.raw)})
        self.label = plan.get('part_label', self.path.stem); self.frames = len(self.raw)
        self.seconds = self.frames / self.hz
        self.reset()

    def reset(self):
        self.t = 0.; self.last = None; self.integral = np.zeros(5); self.since = None

    def pause(self):
        """Call on resume: the time spent stopped never advances the plan."""
        self.last = None; self.since = None

    @property
    def start(self): return {j: int(v) for j, v in zip(JOINTS, self.raw[0])}

    @property
    def done(self): return self.t * self.hz >= self.frames - 1

    def next(self, measured, now):
        """Requested raw goals for this worker cycle (before the server's TeachingTargets bound)."""
        if self.last is not None: self.t += min(max(0., now - self.last), 2/30)
        self.last = now
        f = min(self.t * self.hz, self.frames - 1); k = int(f); a = f - k
        target = self.raw[k] * (1 - a) + self.raw[min(k + 1, self.frames - 1)] * a
        m = np.array([measured[j] for j in JOINTS], float)
        error_deg = (target[:5] - m[:5]) / self.scale[:5]
        if np.abs(error_deg).max() > TRACK_LIMIT_DEG:
            if self.since is None: self.since = now
            elif now - self.since > TRACK_GRACE_S:
                raise RuntimeError(f'plan tracking error {np.abs(error_deg).max():.1f} deg for more than {TRACK_GRACE_S}s at frame {k}')
        else: self.since = None
        out = target.copy()
        if self.tracking and k >= self.tracking['start_index']:
            cap = self.tracking['max_deg']
            self.integral = np.clip(self.integral + self.tracking['ki'] * error_deg / 15, -cap, cap)
            bias = np.clip(self.tracking['kp'] * error_deg + self.integral, -cap, cap)
            out[:5] = np.clip(target[:5] + bias * self.scale[:5], self.lo[:5] + 2, self.hi[:5] - 2)
        s = self.squeeze
        if s and s['start_index'] <= k < s['end_index']:
            out[5] = max(out[5], m[5] - s['units'] * self.scale[5])
        return {j: int(round(v)) for j, v in zip(JOINTS, out)}, k
