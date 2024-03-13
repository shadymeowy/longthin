import numpy as np


def angle(v1, v2, n):
    dot = np.dot(v1, v2)
    cross = np.dot(n, np.cross(v1, v2))
    a = np.arctan2(cross, dot)
    if a < 0:
        a += 2*np.pi
    return a
