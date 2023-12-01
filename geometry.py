from scipy.spatial.transform import Rotation as R
import numpy as np

NORMAL_XY = np.array([0., 0., 1.])
NORMAL_YZ = np.array([1., 0., 0.])
NORMAL_XZ = np.array([0., 1., 0.])

E_X = np.array([1., 0., 0.])
E_Y = np.array([0., 1., 0.])
E_Z = np.array([0., 0., 1.])


def norm(v):
    return np.sqrt(np.dot(v, v))


def plane_from_points(p1, p2, p3):
    n = np.cross(p2 - p1, p3 - p1)
    n /= norm(n)
    return (p1, n)


def line_from_points(p1, p2):
    v = p2 - p1
    v /= norm(v)
    return (p1, v)


def intersection_plane_line(plane, line):
    p, n = plane
    p1, v = line
    n = n / norm(n)
    v = v / norm(v)
    t = np.dot(p - p1, n) / np.dot(v, n)
    return p1 + v * t


def orthogonal_vector(v):
    best_i = 0
    best_d = norm(v)
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
    v1 /= norm(v1)
    return v1


def orthogonal_vectors(v, n):
    v = np.array(v, np.float64)
    v = v / norm(v)
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
        v1 /= norm(v1)
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
    v1 /= norm(v1)
    v2 = np.cross(v, v1)
    v2 /= norm(v2)
    return v1, v2


def camera_get_vecs(pos, att, hfov, vfov, vec=E_X, vec_h=E_Y, vec_v=E_Z):
    if not isinstance(att, R):
        att = R.from_euler('xyz', att, degrees=True)
    vfov = np.deg2rad(vfov)
    hfov = np.deg2rad(hfov)
    vfov_half = vfov / 2
    hfov_half = hfov / 2
    vec = att.apply(vec)
    vec_v = att.apply(vec_v)
    vec_h = att.apply(vec_h)
    vec_v = vec_v * np.tan(vfov_half)
    vec_h = vec_h * np.tan(hfov_half)
    return (vec, vec_v, vec_h)


def camera_get_bpoints(pos, att, hfov, vfov, max_dist=2.):
    vec, vec_v, vec_h = camera_get_vecs(pos, att, hfov, vfov)
    vecs = [
        vec + vec_v + vec_h,
        vec + vec_v - vec_h,
        vec - vec_v + vec_h,
        vec - vec_v - vec_h,
    ]
    pg = np.array([0., 0., 0.])
    ng = np.array([0., 0., 1.])
    bpoints = [intersection_plane_line((pg, ng), (pos, v)) for v in vecs]
    for i, point in enumerate(bpoints):
        if np.dot(point - pos, vec) < 0:
            p = pos + max_dist * vecs[i]
            # project onto plane
            p = intersection_plane_line((pg, ng), (p, ng))
            bpoints[i] = p

    return bpoints


def camera_rays(img_points, pos, att, hfov, vfov, width, height):
    vec, vec_v, vec_h = camera_get_vecs(pos, att, hfov, vfov)
    rays = []
    for p in img_points:
        p = np.array(p, np.float64)
        p[0] = (p[0] / width - 0.5) * 2
        p[1] = (p[1] / height - 0.5) * 2
        p = p[0] * vec_h + p[1] * vec_v + vec
        p /= norm(p)
        rays.append(p)
    return rays
