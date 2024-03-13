from dataclasses import dataclass
import numpy as np

from .common import angle


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

    def __repr__(self):
        return f'Arc(p={self.p}, v={self.v1}, n={self.n}, theta={self.theta})'
