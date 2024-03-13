from dataclasses import dataclass
import numpy as np

from .common import angle
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
        if t < 0 or t > self.length:
            raise ValueError('t must be between 0 and the length of the arc')
        p = self.p + np.cos(theta) * self.v1 + np.sin(theta) * e1
        return p

    def points(self, t, N=100):
        e1 = np.cross(self.n, self.v1)
        if t < 0 or t > self.length:
            raise ValueError('t must be between 0 and the length of the arc')
        theta = np.linspace(0, t / self.r, N)
        ps = self.p + np.outer(np.cos(theta), self.v1) + np.outer(np.sin(theta), e1)
        return ps

    def is_on(self, p, tol=1e-6):
        v = p - self.p
        v_closest = self._closest(p)
        print(v_closest, v)
        if np.linalg.norm(v - v_closest) > tol:
            return False
        a = angle(self.v1, v, self.n)
        return 0 <= a <= self.theta

    def _closest(self, p):
        # calculate the closest without checking if the point is on the arc
        v = p - self.p
        v *= self.r / np.linalg.norm(v)
        return v

    def closest(self, p):
        p_c = self.p + self._closest(p)
        if self.is_on(p_c):
            return p_c
        d1 = np.linalg.norm(p - self.q1)
        d2 = np.linalg.norm(p - self.q2)
        if d1 < d2:
            return self.q1
        else:
            return self.q2

    def intersection(self, obj):
        if isinstance(obj, Line):
            return intersection_arc_line(self, obj)
        elif isinstance(obj, Arc):
            return intersection_arc_arc(self, obj)
        else:
            raise TypeError('Intersection is only implemented for Line and Arc objects')


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
        if t < 0 or t > self.length:
            return None
        p = self.q1 + t * self.u1
        return p

    def points(self, t, N=100):
        if t < 0 or t > self.length:
            return None
        t = np.linspace(0, t, N)
        ps = self.q1 + np.outer(t, self.u1)
        return ps

    def is_on(self, p, tol=1e-6):
        v = p - self.q1
        v_closest = self._closest(p)
        if np.linalg.norm(v - v_closest) > tol:
            return False
        dot = np.dot(v, self.u1)
        return 0 <= dot <= self.length

    def _closest(self, p):
        # calculate the closest without checking if the point is on the line
        v = p - self.q1
        dot = np.dot(v, self.u1)
        return dot * self.u1

    def closest(self, p):
        p_c = self.q1 + self._closest(p)
        if self.is_on(p_c):
            return p_c
        d1 = np.linalg.norm(p - self.q1)
        d2 = np.linalg.norm(p - self.q2)
        if d1 < d2:
            return self.q1
        else:
            return self.q2

    def intersection(self, obj):
        if isinstance(obj, Line):
            return intersection_line_line(self, obj)
        elif isinstance(obj, Arc):
            return intersection_arc_line(obj, self)
        else:
            raise TypeError('Intersection is only implemented for Line and Arc objects')
