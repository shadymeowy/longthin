from dataclasses import dataclass
import numpy as np

from .common import *
from .intersection import *


@dataclass
class Arc:
    p: np.ndarray  # center
    v1: np.ndarray  # start vector
    v2: np.ndarray  # end vector
    n: np.ndarray  # normal vector

    theta: float = None  # angle
    q1: np.ndarray = None  # start point
    q2: np.ndarray = None  # end point
    u1: np.ndarray = None  # start tangent vector
    u2: np.ndarray = None  # end tangent vector
    r: float = None  # radius
    length: float = None  # length

    def __post_init__(self):
        # TODO: fix this hack
        if np.allclose(self.v1, self.v2):
            self.theta = 2*np.pi
        else:
            self.theta = angle(self.v1, self.v2, self.n)
        self.n = self.n / np.linalg.norm(self.n)
        self.q1 = self.p + self.v1
        self.q2 = self.p + self.v2
        self.u1 = np.cross(self.n, self.v1)
        self.u1 /= np.linalg.norm(self.u1)
        self.u2 = np.cross(self.n, self.v2)
        self.u2 /= np.linalg.norm(self.u2)
        self.r = np.linalg.norm(self.v1)
        self.length = self.r * self.theta

    def __repr__(self):
        return f'Arc(p={self.p}, v={self.v1}, n={self.n}, theta={self.theta})'

    def point(self, t):
        e1 = np.cross(self.n, self.v1)
        theta = t / self.r
        if t < 0:
            return self.q1
        elif t > self.length:
            return self.q2
        p = self.p + np.cos(theta) * self.v1 + np.sin(theta) * e1
        return p

    def tangent(self, t):
        e1 = np.cross(self.n, self.v1)
        theta = t / self.r
        if t < 0:
            return self.u1
        elif t > self.length:
            return self.u2
        v = -np.sin(theta) * self.v1 + np.cos(theta) * e1
        v /= np.linalg.norm(v)
        return v

    def points(self, N=100):
        e1 = np.cross(self.n, self.v1)
        theta = np.linspace(0, self.theta, N)
        ps = self.p + np.outer(np.cos(theta), self.v1) + np.outer(np.sin(theta), e1)
        return ps

    def is_on(self, p, tol=1e-6):
        v = p - self.p
        v_closest = self._closest(p)
        if np.linalg.norm(v - v_closest) > tol:
            return False
        a = angle(self.v1, v, self.n)
        return 0 <= a <= self.theta

    def param(self, p):
        # TODO combine with is_on
        if not self.is_on(p):
            return None
        v = p - self.p
        a = angle(self.v1, v, self.n)
        return a * self.r

    def _closest(self, p):
        # calculate the closest without checking if the point is on the arc
        v = p - self.p
        v *= self.r / np.linalg.norm(v)
        return v

    def closest(self, p):
        p_c = self.p + self._closest(p)
        if self.is_on(p_c):
            return p_c, np.linalg.norm(p - p_c)
        d1 = np.linalg.norm(p - self.q1)
        d2 = np.linalg.norm(p - self.q2)
        if d1 < d2:
            return self.q1, d1
        else:
            return self.q2, d2

    def intersection(self, obj):
        if isinstance(obj, Line):
            return intersection_arc_line(self, obj)
        elif isinstance(obj, Arc):
            return intersection_arc_arc(self, obj)
        else:
            raise TypeError('Intersection is only implemented for Line and Arc objects')

    def path_of(self, v, E=EX, En=EZ):
        c1 = np.dot(v, E)
        c2 = np.dot(v, np.cross(En, E))
        v1_ = self.v1 + c1*self.u1 + c2*np.cross(En, self.u1)
        v2_ = self.v2 + c1*self.u2 + c2*np.cross(En, self.u2)
        return Arc(self.p, v1_, v2_, self.n)


@dataclass
class Line:
    q1: np.ndarray  # start point
    q2: np.ndarray  # end point

    u1: np.ndarray = None  # start tangent vector
    u2: np.ndarray = None  # end tangent vector
    length: float = None  # length

    def __post_init__(self):
        self.u1 = self.q2 - self.q1
        self.length = np.linalg.norm(self.u1)
        self.u1 /= self.length
        self.u2 = self.u1

    def __repr__(self):
        return f'Line(q1={self.q1}, q2={self.q2})'

    def point(self, t):
        if t < 0:
            return self.q1
        elif t > self.length:
            return self.q2
        p = self.q1 + t * self.u1
        return p

    def tangent(self, t):
        return self.u1

    def points(self, N=100):
        t = np.linspace(0, self.length, N)
        ps = self.q1 + np.outer(t, self.u1)
        return ps

    def is_on(self, p, tol=1e-6):
        v = p - self.q1
        v_closest = self._closest(p)
        if np.linalg.norm(v - v_closest) > tol:
            return False
        dot = np.dot(v, self.u1)
        return 0 <= dot <= self.length

    def param(self, p):
        # TODO combine with is_on
        if not self.is_on(p):
            return None
        v = p - self.q1
        dot = np.dot(v, self.u1)
        return dot

    def _closest(self, p):
        # calculate the closest without checking if the point is on the line
        v = p - self.q1
        dot = np.dot(v, self.u1)
        return dot * self.u1

    def closest(self, p):
        p_c = self.q1 + self._closest(p)
        if self.is_on(p_c):
            return p_c, np.linalg.norm(p - p_c)
        d1 = np.linalg.norm(p - self.q1)
        d2 = np.linalg.norm(p - self.q2)
        if d1 < d2:
            return self.q1, d1
        else:
            return self.q2, d2

    def intersection(self, obj):
        if isinstance(obj, Line):
            return intersection_line_line(self, obj)
        elif isinstance(obj, Arc):
            return intersection_arc_line(obj, self)
        else:
            raise TypeError('Intersection is only implemented for Line and Arc objects')

    def path_of(self, v, E=EX, En=EZ):
        c1 = np.dot(v, E)
        c2 = np.dot(v, np.cross(En, E))
        t = c1*self.u1 + c2*np.cross(En, self.u1)
        return Line(self.q1 + t, self.q2 + t)

    def paths_of(self, vs, E=EX, En=EZ):
        return [self.path_of(v, E, En) for v in vs]


class Path:
    def __init__(self, *args):
        self.path = args
        self.length = sum(obj.length for obj in self.path)

    def __getitem__(self, i):
        return self.path[i]

    def __len__(self):
        return len(self.path)

    def __iter__(self):
        return iter(self.path)

    def __repr__(self):
        return f'Path({self.path})'

    def point(self, t):
        if t < 0:
            return self.path[0].q1
        for obj in self.path:
            if t <= obj.length:
                return obj.point(t)
            t -= obj.length
        return self.path[-1].q2

    def tangent(self, t):
        if t < 0:
            return self.path[0].u1
        for obj in self.path:
            if t <= obj.length:
                return obj.tangent(t)
            t -= obj.length
        return self.path[-1].u2

    def points(self, N=100, uniform=False):
        if not uniform:
            ps = [obj.points(N) for obj in self.path]
            return np.vstack(ps)
        # TODO: make it efficient
        ps = []
        t = np.linspace(0, self.length, N)
        acc = 0
        i = 0
        for t_ in t:
            if t_ > acc + self.path[i].length:
                acc += self.path[i].length
                i += 1
            ps.append(self.path[i].point(t_ - acc))
        return np.vstack(ps)

    def is_on(self, p, tol=1e-6):
        for obj in self.path:
            if obj.is_on(p, tol):
                return True
        return False

    def closest(self, p):
        p_c, d = self.path[0].closest(p)
        for obj in self.path[1:]:
            p_c_, d_ = obj.closest(p)
            if d_ < d:
                p_c = p_c_
                d = d_
        return p_c, d

    def param(self, p):
        acc = 0
        for obj in self.path:
            if obj.is_on(p):
                return obj.param(p) + acc
            acc += obj.length
        return None

    def intersection(self, obj):
        intersections = []
        for obj_ in self.path:
            intersections_ = obj.intersection(obj_)
            if intersections_ is not None:
                intersections.extend(intersections_)
        return np.array(intersections, np.float32)

    def path_of(self, v, E=EX, En=EZ):
        return Path(*(obj.path_of(v, E, En) for obj in self.path))

    def paths_of(self, vs, E=EX, En=EZ):
        return [self.path_of(v, E, En) for v in vs]
