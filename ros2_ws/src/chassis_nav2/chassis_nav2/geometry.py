"""Rigid transforms and Ackermann conversions, without ROS dependencies."""
import math
import numpy as np


def transform(translation, rpy):
    x, y, z = translation
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll),
                              math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    result = np.eye(4)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]]
    result[:3, 3] = [x, y, z]
    return result


def inverse(matrix):
    result = np.eye(4)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ matrix[:3, 3]
    return result


def quaternion(matrix):
    """Rotation matrix to normalized ROS x/y/z/w quaternion."""
    r = matrix[:3, :3]
    trace = np.trace(r)
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        q = [(r[2, 1]-r[1, 2])/s, (r[0, 2]-r[2, 0])/s,
             (r[1, 0]-r[0, 1])/s, s/4]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(max(0, 1+r[i, i]-r[j, j]-r[k, k])) * 2
        q = [0., 0., 0., 0.]
        q[i], q[j], q[k], q[3] = s/4, (r[j, i]+r[i, j])/s, (r[k, i]+r[i, k])/s, (r[k, j]-r[j, k])/s
    return (np.asarray(q)/np.linalg.norm(q)).tolist()


def ackermann_command(v, w, wheel_base, max_steering_deg, max_speed,
                      steering_sign=-1.0):
    """Twist (m/s, rad/s) to JD03 Ecu (km/h, degrees, gear, brake)."""
    if not all(math.isfinite(float(a)) for a in (v, w)):
        return 0., 0., 1, True
    v = max(-max_speed, min(max_speed, v))
    if abs(v) < .01:
        return 0., 0., 1, True
    angle = math.degrees(math.atan(wheel_base * w / v))
    angle = steering_sign * max(-max_steering_deg, min(max_steering_deg, angle))
    return abs(v) * 3.6, angle, (1 if v > 0 else 3), False
