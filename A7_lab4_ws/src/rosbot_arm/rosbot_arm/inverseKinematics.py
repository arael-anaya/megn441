import numpy as np


l0 = 0.05 + 0.0654868 + 0.0338648 + 0.0772047
l1 = 0.130
l2 = 0.130
l3 = 0.055
tool_link = 0.117


def solveIK(x,y,z,roll,pitch, upDownConstraint):
    jointAngles = [0 , 0 , 0, 0, 0]

    roll = np.deg2rad(roll)
    pitch = np.deg2rad(pitch)

    jointAngles[0] = np.arctan2(y,x)

    h = z - l0
    r = np.sqrt(x**2 + y**2)

    rw = r - (l3+tool_link) * np.cos(pitch)
    hw = h - (l3+tool_link) * np.sin(pitch)
    alpha = np.arctan2(hw, rw)

    d = np.sqrt(rw**2+hw**2)

    if d > (l1+l2):
        return None

    cosGamma =  (l1**2+l2**2-d**2) / (2*l1*l2)
    cosGamma =  max(-1, min(1, cosGamma))
    sinGamma =  np.sqrt(1-cosGamma**2)
    gamma = np.arctan2(sinGamma, cosGamma)

    beta = np.arctan2(l2*np.sin(gamma) , l1 - l2*np.cos(gamma))

    if upDownConstraint > 0:
        jointAngles[1] = alpha + beta
        jointAngles[2] = -(np.pi - gamma)
    else:
        jointAngles[1] = alpha - beta
        jointAngles[2] = (np.pi - gamma)


    jointAngles[3] = pitch - jointAngles[2] - jointAngles[1]
    jointAngles[4] = roll

    jointAngles  = np.rad2deg(jointAngles)

    jointAngles[1] = jointAngles[1] - 90

    return jointAngles


# Only joint 2 is limited to protect the screen
min_pulses = [0, 125, 0, 0, 0]
max_pulses = [1000, 775, 1000, 1000, 1000]


def angle_to_pulse(angle):
    return 500 + angle * 500 / 120


def pulse_to_angle(pulse):
    return (pulse - 500) * 120 / 500


def solvePulses(x, y, z, roll, pitch):
    """Servo pulses that put the tool at the pose with exactly this pitch, or None if it is out of reach or a joint
    would leave its limits. Tries the elbow-up solution first, like ik_node."""
    for upDownConstraint in (1, -1):
        jointAngles = solveIK(x, y, z, roll, pitch, upDownConstraint)
        if jointAngles is None:
            continue
        pulses = [int(round(angle_to_pulse(a))) for a in jointAngles]
        if all(lo <= p <= hi for p, lo, hi in zip(pulses, min_pulses, max_pulses)):
            return pulses
    return None


def solveFK(jointAngles):
    """Inverse of solveIK: servo angles (deg, as solveIK returns them) -> (x, y, z, roll, pitch), angles in deg."""
    a = np.deg2rad(jointAngles)
    t1 = a[1] + np.pi / 2
    t2 = a[2]
    pitch = t1 + t2 + a[3]
    r = l1 * np.cos(t1) + l2 * np.cos(t1 + t2) + (l3 + tool_link) * np.cos(pitch)
    h = l1 * np.sin(t1) + l2 * np.sin(t1 + t2) + (l3 + tool_link) * np.sin(pitch)
    return (r * np.cos(a[0]), r * np.sin(a[0]), h + l0, np.rad2deg(a[4]), np.rad2deg(pitch))


def highestReach(heading, rMin, rMax, roll, pitch, margin=0.0, rStep=0.01, zStep=0.005, zTop=0.45):
    """Highest tool z reachable at exactly this roll and pitch, over radii rMin..rMax along `heading` (rad) from the
    joint 1 axis. The heading is kept so the arm only moves in and up. `margin` shortens the reach limit (m) so the
    arm is not asked to fully straighten. Returns (x, y, z) or None."""
    best = None
    r = rMax
    while r >= rMin - 1e-9:
        z = zTop
        while z > 0.0 and (best is None or z > best[2]):
            x, y = r * np.cos(heading), r * np.sin(heading)
            if solvePulses(x, y, z, roll, pitch) is not None and \
                    np.hypot(r - (l3 + tool_link) * np.cos(np.deg2rad(pitch)),
                             z - l0 - (l3 + tool_link) * np.sin(np.deg2rad(pitch))) <= l1 + l2 - margin:
                best = (x, y, z)
                break
            z -= zStep
        r -= rStep
    return best
