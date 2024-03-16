import numpy as np


def accel_calibrate(up, down, left, right, forward, backward):
    # calculate the offset and scale
    b = np.array([0, 0, 0])
    A_1 = np.eye(3)

    # calculate the offset
    b[0] = (up[0] + down[0]) / 2
    b[1] = (left[1] + right[1]) / 2
    b[2] = (forward[2] + backward[2]) / 2

    # calculate the scale
    A_1[0, 0] = (up[0] - down[0]) / 2
    A_1[1, 1] = (left[1] - right[1]) / 2
    A_1[2, 2] = (forward[2] - backward[2]) / 2

    return A_1, b
