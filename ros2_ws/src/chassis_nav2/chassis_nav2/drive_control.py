"""Chassis gear interlock, independent of ROS and hardware drivers."""
import math


class GearInterlock:
    def __init__(self, stop_speed=.03, stop_duration=.25, settle_duration=.20):
        if any(not math.isfinite(v) or v <= 0 for v in
               (stop_speed, stop_duration, settle_duration)):
            raise ValueError('gear interlock limits must be positive')
        self.stop_speed, self.stop_duration = stop_speed, stop_duration
        self.settle_duration = settle_duration
        self.gear, self.pending, self.stopped_since, self.settle_until = 1, None, None, 0.

    def update(self, requested, speed, now, active=True):
        """Return (gear, drive permitted); zeros retain the selected gear.

        A new direction must be requested continuously while measured speed
        stays near zero. Braking commands and stale feedback cannot change gear.
        """
        if not active or requested not in (1, 3) or not math.isfinite(speed):
            self.pending, self.stopped_since = None, None
            return self.gear, False
        if requested == self.gear:
            self.pending, self.stopped_since = None, None
            return self.gear, now >= self.settle_until
        if requested != self.pending:
            self.pending, self.stopped_since = requested, None
        if abs(speed) > self.stop_speed:
            self.stopped_since = None
        elif self.stopped_since is None:
            self.stopped_since = now
        elif now - self.stopped_since >= self.stop_duration:
            self.gear = requested
            self.pending, self.stopped_since = None, None
            self.settle_until = now + self.settle_duration
        return self.gear, False
