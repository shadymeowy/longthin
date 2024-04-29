import numpy as np

from .common import EZ
from .shapes import Arc, Line, Path
from .plot import plot_path


def path_rsr(p1, p2, v1, v2, R=1., n=EZ):
    r1 = np.cross(v1, n) * R
    r2 = np.cross(v2, n) * R

    c1 = p1 - r1
    c2 = p2 - r2
    d = c2 - c1
    dl = np.linalg.norm(d)
    d_n = d / dl
    e_n = np.cross(d_n, n)

    rp1 = e_n * R
    rp2 = e_n * R
    q1 = c1 + rp1
    q2 = c2 + rp2

    path = Path(
        Arc(c1, r1, rp1, n),
        Line(q1, q2),
        Arc(c2, rp2, r2, n),
    )
    return path


def path_rsrc(p1, c2, v1, v2, R=1., n=EZ):
    r1 = np.cross(v1, n) * R

    c1 = p1 - r1
    d = c2 - c1
    dl = np.linalg.norm(d)
    d_n = d / dl
    e_n = np.cross(d_n, n)

    rp1 = e_n * R
    rp2 = e_n * R
    q1 = c1 + rp1
    q2 = c2 + rp2

    path = Path(
        Arc(c1, r1, rp1, n),
        Line(q1, q2),
        Arc(c2, rp2, rp2, n),
    )
    return path


def path_rsr2(p1, p2, v1, v2, R1=1., R2=1., n=EZ):
    r1 = np.cross(v1, n) * R1
    r2 = np.cross(v2, n) * R2

    c1 = p1 - r1
    c2 = p2 - r2
    d = c2 - c1
    dl = np.linalg.norm(d)
    d_n = d / dl
    e_n = np.cross(d_n, n)

    rp1_parallel = (R1**2 - R2*R1) / dl
    if R1**2 < rp1_parallel**2:
        return None
    rp1_perpendicular = np.sqrt(R1**2 - rp1_parallel**2)
    rp1 = rp1_parallel * d_n + rp1_perpendicular * e_n
    rp2 = rp1 * R2 / R1

    q1 = c1 + rp1
    q2 = c2 + rp2

    path = Path(
        Arc(c1, r1, rp1, n),
        Line(q1, q2),
        Arc(c2, rp2, r2, n),
    )
    return path


def path_rsr2c(p1, c2, v1, v2, R1=1., R2=1., n=EZ):
    r1 = np.cross(v1, n) * R1

    c1 = p1 - r1
    d = c2 - c1
    dl = np.linalg.norm(d)
    d_n = d / dl
    e_n = np.cross(d_n, n)

    rp1_parallel = (R1**2 - R2*R1) / dl
    if R1**2 < rp1_parallel**2:
        return None
    rp1_perpendicular = np.sqrt(R1**2 - rp1_parallel**2)
    rp1 = rp1_parallel * d_n + rp1_perpendicular * e_n
    rp2 = rp1 * R2 / R1

    q1 = c1 + rp1
    q2 = c2 + rp2

    path = Path(
        Arc(c1, r1, rp1, n),
        Line(q1, q2),
        Arc(c2, rp2, rp2, n),
    )
    return path


def path_rsl(p1, p2, v1, v2, R=1., n=EZ):
    r1 = np.cross(v1, n) * R
    r2 = -np.cross(v2, n) * R

    c1 = p1 - r1
    c2 = p2 - r2
    d = c2 - c1
    dl = np.linalg.norm(d)
    d_n = d / dl
    e_n = np.cross(d_n, n)

    rp1_parallel = 2*R**2 / dl
    if R**2 < rp1_parallel**2:
        return None
    rp1_perpendicular = np.sqrt(R**2 - rp1_parallel**2)
    rp1 = rp1_parallel * d_n + rp1_perpendicular * e_n
    rp2 = -rp1

    q1 = c1 + rp1
    q2 = c2 + rp2

    path = Path(
        Arc(c1, r1, rp1, n),
        Line(q1, q2),
        Arc(c2, rp2, r2, -n),
    )
    return path


def path_rslc(p1, c2, v1, v2, R=1., n=EZ):
    r1 = np.cross(v1, n) * R

    c1 = p1 - r1
    d = c2 - c1
    dl = np.linalg.norm(d)
    d_n = d / dl
    e_n = np.cross(d_n, n)

    rp1_parallel = 2*R**2 / dl
    if R**2 < rp1_parallel**2:
        return None
    rp1_perpendicular = np.sqrt(R**2 - rp1_parallel**2)
    rp1 = rp1_parallel * d_n + rp1_perpendicular * e_n
    rp2 = -rp1

    q1 = c1 + rp1
    q2 = c2 + rp2

    path = Path(
        Arc(c1, r1, rp1, n),
        Line(q1, q2),
        Arc(c2, rp2, rp2, -n),
    )
    return path


def path_rsl2(p1, p2, v1, v2, R1=1., R2=1., n=EZ):
    r1 = np.cross(v1, n) * R1
    r2 = -np.cross(v2, n) * R2

    c1 = p1 - r1
    c2 = p2 - r2
    d = c2 - c1
    dl = np.linalg.norm(d)
    d_n = d / dl
    e_n = np.cross(d_n, n)

    rp1_parallel = (R1**2 + R2*R1) / dl
    if R1**2 < rp1_parallel**2:
        return None
    rp1_perpendicular = np.sqrt(R1**2 - rp1_parallel**2)
    rp1 = rp1_parallel * d_n + rp1_perpendicular * e_n
    rp2 = -rp1 * R2 / R1

    q1 = c1 + rp1
    q2 = c2 + rp2

    path = Path(
        Arc(c1, r1, rp1, n),
        Line(q1, q2),
        Arc(c2, rp2, r2, -n),
    )
    return path


def path_rsl2c(p1, c2, v1, v2, R1=1., R2=1., n=EZ):
    r1 = np.cross(v1, n) * R1

    c1 = p1 - r1
    d = c2 - c1
    dl = np.linalg.norm(d)
    d_n = d / dl
    e_n = np.cross(d_n, n)

    rp1_parallel = (R1**2 + R2*R1) / dl
    if R1**2 < rp1_parallel**2:
        return None
    rp1_perpendicular = np.sqrt(R1**2 - rp1_parallel**2)
    rp1 = rp1_parallel * d_n + rp1_perpendicular * e_n
    rp2 = -rp1 * R2 / R1

    q1 = c1 + rp1
    q2 = c2 + rp2

    path = Path(
        Arc(c1, r1, rp1, n),
        Line(q1, q2),
        Arc(c2, rp2, rp2, -n),
    )
    return path


def path_rrr(p1, p2, v1, v2, R=1., n=EZ):
    r1 = np.cross(v1, n) * R
    r2 = np.cross(v2, n) * R
    c1 = p1 - r1
    c2 = p2 - r2
    d = c2 - c1
    dl = np.linalg.norm(d)
    d_n = d / dl
    e_n = np.cross(d_n, n)

    rp1_parallel = dl / 4
    if R**2 < rp1_parallel**2:
        return None
    rp1_perpendicular = np.sqrt(R**2 - rp1_parallel**2)
    rp1 = rp1_parallel * d_n + rp1_perpendicular * e_n
    rp2 = rp1 - d / 2
    rp3 = -rp1
    rp4 = -rp2

    c3 = c1 + rp1 * 2

    path = Path(
        Arc(c1, r1, rp1, n),
        Arc(c3, rp3, rp4, -n),
        Arc(c2, rp2, r2, n)
    )
    return path


def path_rrr3(p1, p2, v1, v2, R1=1., R2=1., R3=1., n=EZ):
    r1 = np.cross(v1, n) * R1
    r2 = np.cross(v2, n) * R2
    c1 = p1 - r1
    c2 = p2 - r2
    d = c2 - c1
    dl = np.linalg.norm(d)
    d_n = d / dl
    e_n = np.cross(d_n, n)

    rpp1_parallel = (dl**2 + (R1 + R3)**2 - (R2 + R3)**2) / (2 * dl)
    if (R1 + R3)**2 < rpp1_parallel**2:
        return None
    rpp1_perpendicular = np.sqrt((R1 + R3)**2 - rpp1_parallel**2)
    rpp1 = rpp1_parallel * d_n + rpp1_perpendicular * e_n
    rpp2 = rpp1 - d
    rp1 = rpp1 * R1 / (R1 + R3)
    rp2 = rpp2 * R2 / (R2 + R3)
    rp3 = -rpp1 * R3 / (R1 + R3)
    rp4 = -rpp2 * R3 / (R2 + R3)

    c3 = c1 + rpp1

    path = Path(
        Arc(c1, r1, rp1, n),
        Arc(c3, rp3, rp4, -n),
        Arc(c2, rp2, r2, n),
    )
    return path


def path_rs(p1, p2, v1, v2, R=1., n=EZ):
    r1 = np.cross(v1, n) * R

    c1 = p1 - r1
    d = p2 - c1
    dl = np.linalg.norm(d)
    d_n = d / dl
    e_n = np.cross(d_n, n)

    rp1_parallel = R**2 / dl
    if R**2 < rp1_parallel**2:
        return None
    rp1_perpendicular = np.sqrt(R**2 - rp1_parallel**2)
    rp1 = rp1_parallel * d_n + rp1_perpendicular * e_n

    q1 = c1 + rp1

    path = Path(
        Arc(c1, r1, rp1, n),
        Line(q1, p2),
    )
    return path


def path_sr(p1, p2, v1, v2, R=1., n=np.array([0, 0, -1])):
    r2 = -np.cross(v2, n) * R

    c2 = p2 - r2
    d = p1 - c2
    dl = np.linalg.norm(d)
    d_n = d / dl
    e_n = np.cross(d_n, n)

    rp2_parallel = R**2 / dl
    if R**2 < rp2_parallel**2:
        return None
    rp2_perpendicular = np.sqrt(R**2 - rp2_parallel**2)
    rp2 = rp2_parallel * d_n + rp2_perpendicular * e_n

    q2 = c2 + rp2

    path = Path(
        Line(p1, q2),
        Arc(c2, rp2, r2, -n)
    )
    return path


def calculate_penalty(path, box, size):
    area_w, area_h = size
    intrs = box.intersection(path)
    if len(intrs) == 0:
        return 0

    p0 = path.point(0)
    pend = path.point(path.length)
    intrs = np.concatenate([[p0], intrs, [pend]])
    params = []
    for intr in intrs:
        param = path.param(intr)
        if param is not None:
            params.append(param)
        else:
            # TODO: investigate why this happens
            print('param is None', intr)
            return 1e7
    params = np.array(params)
    intrs = intrs[np.argsort(params)]
    params = np.sort(params)

    inside = (abs(p0[0]) < area_w/2 and abs(p0[1]) < area_h/2)
    penalty = 0
    param_prev = 0
    for intr in intrs:
        param = path.param(intr)
        if inside:
            penalty += param - param_prev
        param_prev = param
        inside = not inside
    return penalty


def _dubins(method, p1, p2, v1, v2, R, reverse, vps):
    n = np.array([0, 0, 1], dtype=np.float32) * (-1 if reverse else 1)
    path = method(p1, p2, v1, v2, R, n)
    if path is None:
        return None, None

    # TODO: do not hardcode this?
    area_w, area_h = 3, 3
    box = Path(
        Line(np.array([-area_w/2, -area_h/2, 0.]), np.array([area_w/2, -area_h/2, 0.])),
        Line(np.array([area_w/2, -area_h/2, 0.]), np.array([area_w/2, area_h/2, 0.])),
        Line(np.array([area_w/2, area_h/2, 0.]), np.array([-area_w/2, area_h/2, 0.])),
        Line(np.array([-area_w/2, area_h/2, 0.]), np.array([-area_w/2, -area_h/2, 0.]))
    )

    intrs = []
    total_penalty = 0
    for path_ in path.paths_of(vps):
        intr = box.intersection(path_)
        penalty = calculate_penalty(path_, box, (area_w, area_h))
        total_penalty += penalty
        intrs.extend(intr)
    intrs.extend(path.intersection(box))
    intrs = np.array(intrs)

    """plot_path(path, p1, p2)
    import matplotlib.pyplot as plt
    plt.show()"""

    cost = (total_penalty, path.length)
    return path, cost


def dubins(p1, p2, theta1, theta2, R, vps, methods):
    p1 = np.array(p1, dtype=np.float32)
    p2 = np.array(p2, dtype=np.float32)
    theta1 = np.deg2rad(theta1)
    theta2 = np.deg2rad(theta2)
    v1 = np.array([np.cos(theta1), np.sin(theta1), 0.], dtype=np.float32)
    v2 = np.array([np.cos(theta2), np.sin(theta2), 0.], dtype=np.float32)

    min_cost = (np.inf, np.inf)
    min_path = None
    for method in methods:
        for reverse in [False, True]:
            path, cost = _dubins(method, p1, p2, v1, v2, R, reverse, vps)
            if path is not None and cost < min_cost:
                min_cost = cost
                min_path = path
    print(min_cost)
    return min_path
