"""Pure navigation math, shared by the robot runner and offline tests."""
import math
import re


def valid_name(name):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', name):
        raise ValueError('Bag name must be 1–80 letters/digits/underscore/hyphen; no paths')
    return name


def yaw(q):
    values = [q.x, q.y, q.z, q.w]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Invalid odometry quaternion')
    norm = math.sqrt(sum(v*v for v in values))
    if norm < .5 or norm > 1.5:
        raise ValueError('Invalid odometry quaternion norm')
    x, y, z, w = [v/norm for v in values]
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def relative(start, pose):
    dx, dy = pose[0]-start[0], pose[1]-start[1]
    heading = math.atan2(math.sin(pose[2]-start[2]), math.cos(pose[2]-start[2]))
    return (dx*math.cos(start[2])+dy*math.sin(start[2]),
            -dx*math.sin(start[2])+dy*math.cos(start[2]), heading)


def command(distance, speed, progress, lateral, heading, previous, dt):
    if not all(math.isfinite(v) for v in [distance, speed, progress, lateral, heading, previous, dt]):
        raise ValueError('Non-finite navigation value')
    if abs(lateral) > .15 or abs(heading) > math.radians(25):
        raise RuntimeError('Exceeded lateral/heading safety limit')
    remaining = distance-progress
    if remaining <= .01:
        return 0., 0., True
    wanted = min(speed, math.sqrt(2*.15*max(0., remaining-.01)))
    linear = min(wanted, previous+.15*min(.1, max(0., dt)))
    angular = max(-.5, min(.5, -1.2*heading-.5*lateral))
    return linear, angular, False
