"""Planar odometry from measured JD03 speed/steering, at a calibrated base origin."""
import math
import numpy as np


class WheelOdometry:
    def __init__(self, wheel_base, rear_axle, speed_stddev=.05,
                 steering_stddev_deg=1., lateral_stddev=.02, max_gap=.5):
        self.wheel_base = wheel_base
        self.rear_axle = np.asarray(rear_axle, dtype=float)
        self.speed_variance = speed_stddev**2
        self.steering_variance = math.radians(steering_stddev_deg)**2
        self.lateral_variance = lateral_stddev**2
        self.max_gap = max_gap
        # Start base_link at (0,0,0), even when rear axle is offset from base_link.
        self.state = np.array([*self.rear_axle, 0.])
        self.covariance = np.diag([.01**2, .01**2, .01**2])
        self.previous = None
        self.direction = None

    def update(self, stamp, speed, steering, gear):
        if (not all(math.isfinite(x) for x in (stamp, speed, steering))
                or abs(steering) >= math.pi/2 or gear not in (1, 2, 3)):
            return None
        if self.previous and stamp <= self.previous[0]:
            return None
        direction = -1 if gear == 3 else 1 if gear == 1 else self.direction
        # Neutral can still coast; don't invent a zero speed or unknown direction.
        if direction is None and abs(speed) > 1e-6:
            return None
        self.direction = direction
        v = abs(speed) * (direction or 1)
        tangent = math.tan(steering)
        w = v*tangent/self.wheel_base
        noise_jacobian = np.array([[1., 0.], [tangent/self.wheel_base,
                                     v*(1+tangent*tangent)/self.wheel_base]])
        noise = noise_jacobian @ np.diag([self.speed_variance, self.steering_variance]) @ noise_jacobian.T
        gap = False
        if self.previous:
            old_stamp, old_v, old_w = self.previous
            dt = stamp-old_stamp
            if dt > self.max_gap:
                # The motion during a feedback outage is unknown; don't integrate
                # the newly received speed over the whole outage.
                self.covariance += np.diag([.25, .25, .09])
                gap = True
            else:
                average_v, average_w = (v+old_v)/2, (w+old_w)/2
                self._integrate(average_v, average_w, dt, noise)
        self.previous = stamp, v, w
        x, y, yaw = self.state
        rx, ry = self.rear_axle
        c, s = math.cos(yaw), math.sin(yaw)
        base_pose = np.array([x-c*rx+s*ry, y-s*rx-c*ry, yaw])
        pose_jacobian = np.array([[1., 0., s*rx+c*ry], [0., 1., -c*rx+s*ry], [0., 0., 1.]])
        pose_cov = pose_jacobian @ self.covariance @ pose_jacobian.T
        velocity_jacobian = np.array([[1., ry], [0., -rx], [0., 1.]])
        velocity_cov = velocity_jacobian @ noise @ velocity_jacobian.T
        velocity_cov[1, 1] += self.lateral_variance
        # A turning base origin away from the rear axle has lateral velocity.
        velocity = np.array([v+ry*w, -rx*w, w])
        return base_pose, velocity, pose_cov, velocity_cov, gap

    def _integrate(self, v, w, dt, noise):
        half = w*dt/2
        if abs(half) < 1e-5:
            sinc = 1-half*half/6
            derivative = -half/3
        else:
            sinc = math.sin(half)/half
            derivative = (half*math.cos(half)-math.sin(half))/(half*half)
        angle = self.state[2]+half
        c, s = math.cos(angle), math.sin(angle)
        dx, dy = v*dt*sinc*c, v*dt*sinc*s
        f = np.array([[1., 0., -dy], [0., 1., dx], [0., 0., 1.]])
        g = np.array([[dt*sinc*c, v*dt*dt/2*(derivative*c-sinc*s)],
                      [dt*sinc*s, v*dt*dt/2*(derivative*s+sinc*c)], [0., dt]])
        self.covariance = f @ self.covariance @ f.T + g @ noise @ g.T
        self.state[:2] += [dx, dy]
        self.state[2] = math.atan2(math.sin(self.state[2]+w*dt), math.cos(self.state[2]+w*dt))
