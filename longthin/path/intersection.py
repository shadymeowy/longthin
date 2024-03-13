import numpy as np


def intersection_arc_line(arc, line, tol=1e-6):
    l = arc.p - line.q1
    h = l - np.dot(l, line.u1) * line.u1
    h2 = np.dot(h, h)
    tmp = arc.r**2 - h2
    if tmp < 0:
        return []
    delta = np.sqrt(tmp) * line.u1
    p1 = arc.p - h + delta
    p2 = arc.p - h - delta
    result = []
    if arc.is_on(p1, tol=tol) and line.is_on(p1, tol=tol):
        result.append(p1)
    if arc.is_on(p2, tol=tol) and line.is_on(p2, tol=tol):
        result.append(p2)
    return np.array(result, np.float32)


def intersection_line_line(line1, line2):
    if np.linalg.norm(np.cross(line1.u1, line2.u1)) < 1e-6:
        return []
    l = line2.q1 - line1.q1
    # assuming line1.u1 and line2.u1 are normalized
    t1 = (np.dot(l, line1.u1) - np.dot(l, line2.u1) * np.dot(line1.u1, line2.u1)) / \
        (1 - np.dot(line1.u1, line2.u1)**2)
    p = line1.point(t1)
    if p is not None and line2.is_on(p):
        return np.array([p], np.float32)
    else:
        return []


def intersection_arc_arc(arc1, arc2, tol=1e-6):
    l = arc2.p - arc1.p
    d = np.linalg.norm(l)
    if d < tol:
        return []
    if d > arc1.r + arc2.r:
        return []
    if d < np.abs(arc1.r - arc2.r):
        return []
    d2 = d * d
    a1 = (arc1.r**2 - arc2.r**2 + d2) / (2 * d2)
    tmp = (arc1.r / d)**2 - a1**2
    if tmp < 0:
        return []
    h1 = np.sqrt(tmp)
    p1 = arc1.p + a1 * l + h1 * np.cross(arc1.n, l)
    p2 = arc1.p + a1 * l - h1 * np.cross(arc1.n, l)
    result = []
    if arc1.is_on(p1, tol=tol) and arc2.is_on(p1, tol=tol):
        result.append(p1)
    if arc1.is_on(p2, tol=tol) and arc2.is_on(p2, tol=tol):
        result.append(p2)
    return np.array(result, np.float32)
