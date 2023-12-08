from scipy.spatial.transform import Rotation as R
import numpy as np
from dataclasses import dataclass


NORMAL_XY = np.array([0., 0., 1.])
NORMAL_YZ = np.array([1., 0., 0.])
NORMAL_XZ = np.array([0., 1., 0.])

E_X = np.array([1., 0., 0.])
E_Y = np.array([0., 1., 0.])
E_Z = np.array([0., 0., 1.])


def as_rotation(att):
    if isinstance(att, (list, tuple)):
        att = np.array(att, np.float64)
    if isinstance(att, np.ndarray):
        if att.shape == (3, 3):
            att = R.from_matrix(att)
        elif att.shape == (4,):
            att = R.from_quat(att)
        elif att.shape == (3,):
            att = R.from_euler('xyz', att, degrees=True)
        else:
            raise ValueError(f'Invalid shape for att: {att.shape}')
    elif isinstance(att, R):
        pass
    else:
        raise ValueError(f'Invalid type for att: {type(att)}')
    return att


def as_euler(att, degrees=True):
    if isinstance(att, (list, tuple)):
        att = np.array(att, np.float64)
    if isinstance(att, np.ndarray) and att.shape == (3,):
        return att
    att = as_rotation(att)
    return att.as_euler('xyz', degrees=degrees)


def plane_from_points(p1, p2, p3):
    n = np.cross(p2 - p1, p3 - p1)
    n /= np.linalg.norm(n)
    return (p1, n)


def line_from_points(p1, p2):
    v = p2 - p1
    v /= np.linalg.norm(v)
    return (p1, v)


def intersection_plane_line(plane, line):
    p, n = plane
    p1, v = line
    n = n / np.linalg.norm(n)
    v = v / np.linalg.norm(v)
    t = np.dot(p - p1, n) / np.dot(v, n)
    return p1 + v * t


def orthogonal_vector(v):
    best_i = 0
    best_d = np.linalg.norm(v)
    for i in range(v.shape[0]):
        unit = np.zeros(v.shape[0])
        unit[i] = 1
        d = np.linalg.norm(np.dot(v, unit))
        if d >= best_d:
            best_i = i
            best_d = d
    v1 = np.zeros(v.shape[0])
    v1[best_i] = 1
    v1 -= v * np.dot(v, v1)
    v1 /= np.linalg.norm(v1)
    return v1


def orthogonal_vectors(v, n):
    v = np.array(v, np.float64)
    v = v / np.linalg.norm(v)
    i_ = []
    d_ = []
    for i in range(v.shape[0]):
        unit = np.zeros(v.shape[0])
        unit[i] = 1
        d = np.abs(np.dot(v, unit))
        i_.append(i)
        d_.append(d)
    i_ = np.array(i_)
    d_ = np.array(d_)
    sort_i = np.argsort(d_, axis=0)
    i_ = i_[sort_i][:n]
    d_ = d_[sort_i][:n]
    vs = []
    for i in i_:
        v1 = np.zeros(v.shape[0])
        v1[i] = 1
        v1 -= v * np.dot(v, v1)
        v1 /= np.linalg.norm(v1)
        vs.append(v1)
    return vs


def orthogonal_vectors2(v):
    d1 = np.dot(v, E_X)
    d2 = np.dot(v, E_Y)
    d3 = np.dot(v, E_Z)
    if d1 < d2:
        if d1 < d3:
            v1 = np.cross(v, E_X)
        else:
            v1 = np.cross(v, E_Z)
    else:
        if d2 < d3:
            v1 = np.cross(v, E_Y)
        else:
            v1 = np.cross(v, E_Z)
    v1 /= np.linalg.norm(v1)
    v2 = np.cross(v, v1)
    v2 /= np.linalg.norm(v2)
    return v1, v2
