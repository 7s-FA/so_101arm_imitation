"""Per-channel, monotonic temperature persistence checks. No hardware writes.

Canonical source; build_player_bundle.py copies this module into the Pi bundle.
"""

import math
import time


class TemperatureFault(RuntimeError):
    pass


class TemperatureGuard:
    def __init__(self, duration=5.0, clock=None):
        if type(duration) not in (int, float) or not math.isfinite(duration) or not 0 <= duration <= 5:
            raise ValueError('temperature_duration_s must be finite and within 0..5')
        self.duration = float(duration)
        self.clock = clock or time.monotonic
        self.pending = {}

    def observe(self, joint, value, limit, active, *, source='primary'):
        key = joint if source == 'primary' else f'{joint}/{source}'
        if type(value) is not int or not 0 <= value <= 150:
            raise TemperatureFault(f'{joint}: invalid Present_Temperature={value}')
        now = self.clock()
        if not math.isfinite(now):
            raise TemperatureFault(f'{joint}: invalid temperature clock')
        if value < limit:
            self.pending.pop(key, None)
            return
        if not active:
            raise TemperatureFault(f'{joint}: Present_Temperature={value}, limit={limit}; hot before activation')
        old = self.pending.get(key)
        if old and now < old['last_seen']:
            raise TemperatureFault(f'{joint}: temperature clock moved backwards')
        if old and now - old['last_seen'] > 1.5:
            raise TemperatureFault(f'{joint}: temperature monitoring gap > 1.5s while hot')
        # A changed threshold starts a new observation window.
        start = old['since'] if old and old['limit'] == limit else now
        elapsed = now - start
        self.pending[key] = {'since': start, 'last_seen': now, 'elapsed_s': elapsed,
                             'value': value, 'limit': limit, 'duration_s': self.duration}
        if elapsed >= self.duration:
            raise TemperatureFault(f'{joint}: Present_Temperature={value}, limit={limit}; '
                                   f'continuous high temperature {elapsed:.2f}s >= {self.duration:g}s')

    def snapshot(self):
        return {key: dict(value) for key, value in self.pending.items()}
