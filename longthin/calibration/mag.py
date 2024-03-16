import numpy as np
from scipy import linalg


def mag_calibrate(data):
    F = 1000
    b = np.zeros([3, 1])
    A_1 = np.eye(3)

    # ellipsoid fit
    s = np.array(data).T
    M, n, d = ellipsoid_fit(s)

    # calibration parameters
    M_1 = linalg.inv(M)
    b = -np.dot(M_1, n)
    A_1 = np.real(F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))

    result = []
    for row in data:
        # subtract the hard iron offset
        xm_off = row[0]-b[0]
        ym_off = row[1]-b[1]
        zm_off = row[2]-b[2]

        # multiply by the inverse soft iron offset
        xm_cal = xm_off * A_1[0, 0] + ym_off * A_1[0, 1] + zm_off * A_1[0, 2]
        ym_cal = xm_off * A_1[1, 0] + ym_off * A_1[1, 1] + zm_off * A_1[1, 2]
        zm_cal = xm_off * A_1[2, 0] + ym_off * A_1[2, 1] + zm_off * A_1[2, 2]

        result = np.append(result, np.array([xm_cal, ym_cal, zm_cal]))
        result = result.reshape(-1, 3)

    return A_1, b, result

def ellipsoid_fit(s):
    # D (samples)
    D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                  2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                  2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

    # S, S_11, S_12, S_21, S_22 (eq. 11)
    S = np.dot(D, D.T)
    S_11 = S[:6, :6]
    S_12 = S[:6, 6:]
    S_21 = S[6:, :6]
    S_22 = S[6:, 6:]

    # C (Eq. 8, k=4)
    C = np.array([[-1,  1,  1,  0,  0,  0],
                  [1, -1,  1,  0,  0,  0],
                  [1,  1, -1,  0,  0,  0],
                  [0,  0,  0, -4,  0,  0],
                  [0,  0,  0,  0, -4,  0],
                  [0,  0,  0,  0,  0, -4]])

    # v_1 (eq. 15, solution)
    E = np.dot(linalg.inv(C),
               S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

    E_w, E_v = np.linalg.eig(E)

    v_1 = E_v[:, np.argmax(E_w)]
    if v_1[0] < 0:
        v_1 = -v_1

    # v_2 (eq. 13, solution)
    v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

    # quadratic-form parameters, parameters h and f swapped as per correction by Roger R on Teslabs page
    M = np.array([[v_1[0], v_1[5], v_1[4]],
                  [v_1[5], v_1[1], v_1[3]],
                  [v_1[4], v_1[3], v_1[2]]])
    n = np.array([[v_2[0]],
                  [v_2[1]],
                  [v_2[2]]])
    d = v_2[3]

    return M, n, d
