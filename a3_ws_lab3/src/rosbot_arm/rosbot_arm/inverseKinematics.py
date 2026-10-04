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

    jointAngles = np.rad2deg(jointAngles)

    return jointAngles
