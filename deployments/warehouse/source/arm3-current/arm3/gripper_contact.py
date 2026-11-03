"""Limit further closing when position feedback indicates sustained contact.

This is not force sensing. Motor overload/temperature/communication faults must
still stop the session. A latched position does not guarantee a secure grasp.
"""
import math

POLICY = {'closing_error_points':2.0, 'stationary_band_points':.5,
          'stationary_seconds':.3, 'opening_release_points':.5,
          'max_sample_gap_seconds':.25}


class GripperContactGuard:
    def __init__(self, calibration, max_relative_points=7):
        self.scale=(calibration['range_max']-calibration['range_min'])/100
        if self.scale <= 0: raise ValueError('Invalid gripper calibration')
        self.max_relative=max_relative_points*self.scale
        self.since=self.low=self.high=self.last_stamp=None
        self.hold=None
        self.events=[]

    def next(self, requested, measured, stamp):
        if not all(math.isfinite(x) for x in (requested,measured,stamp)):
            raise ValueError('Invalid gripper contact feedback')
        if self.last_stamp is not None and (stamp < self.last_stamp or stamp-self.last_stamp > POLICY['max_sample_gap_seconds']):
            self.since=None
        self.last_stamp=stamp
        if self.hold is not None:
            if requested > self.hold + POLICY['opening_release_points']*self.scale:
                self.hold=None
                self.since=None
            else:
                return max(0,min(4095,max(math.ceil(measured-self.max_relative),
                       min(math.floor(measured+self.max_relative),self.hold))))
        closing = requested < measured-POLICY['closing_error_points']*self.scale
        if not closing:
            self.since=None
            return requested
        if self.since is None:
            self.since=stamp
            self.low=self.high=measured
        self.low=min(self.low,measured)
        self.high=max(self.high,measured)
        if self.high-self.low > POLICY['stationary_band_points']*self.scale:
            self.since=stamp
            self.low=self.high=measured
        if stamp-self.since >= POLICY['stationary_seconds']:
            self.hold=round(measured)
            self.events.append({'monotonic_s':stamp,'requested_raw':requested,'hold_raw':self.hold})
            print('그리퍼 접촉 추정: 추가 닫힘을 제한하고 현재 위치를 유지합니다. 열기 명령으로 해제됩니다.',flush=True)
            return self.hold
        return requested
