import numpy as np


def accel_calibrate(xplus, xneg, yplus, yneg, zplus, zneg, g=9.8):
    # calculate the offset and scale
    b = np.zeros(3)
    A = np.eye(3)

    # calculate the offset
    b[0] = (xplus + xneg) / 2
    b[1] = (yplus + yneg) / 2
    b[2] = (zplus + zneg) / 2

    # calculate the scale
    A[0, 0] = 2 / (xplus - xneg) * g
    A[1, 1] = 2 / (yplus - yneg) * g
    A[2, 2] = 2 / (zplus - zneg) * g
    return A, b
