from dataclasses import dataclass
import numpy as np


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
            raise ValueError('t must be between 0 and the length of the line')
        p = self.q1 + t * self.u1
        return p

    def points(self, t, N=100):
        if t < 0 or t > self.length:
            raise ValueError('t must be between 0 and the length of the line')
        t = np.linspace(0, t, N)
        ps = self.q1 + np.outer(t, self.u1)
        return ps
