import numpy as np
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R

from .geometry import as_rotation, as_euler


@dataclass
class Pose:
    pos: np.ndarray = None
    att: np.ndarray = None

    def __post_init__(self):
        if self.pos is None:
            self.pos = np.zeros(3)
        if self.att is None:
            self.att = np.zeros(3)
        self.pos = np.array(self.pos, np.float64)
        self.att = as_euler(self.att)

    def __str__(self):
        return f'Pose(pos={self.pos}, att={self.att})'

    def __repr__(self):
        return str(self)

    def copy(self):
        return Pose(self.pos.copy(), self.att.copy())

    def from_frame(self, other):
        # Takes a vector/pose from the this frame and transforms it to the standard frame
        att_self = as_rotation(self.att)
        if isinstance(other, Pose):
            pos = att_self.apply(other.pos)
            pos += self.pos
            att = att_self * as_rotation(other.att)
            return Pose(pos, att)
        elif isinstance(other, np.ndarray):
            pos = att_self.apply(other)
            pos += self.pos
            return pos
        elif isinstance(other, R):
            att = att_self * as_rotation(other)
            return att
        else:
            raise ValueError(f'Invalid type for other: {type(other)}')

    def to_frame(self, other):
        # Takes a vector/pose from the standard frame and transforms it to this frame
        att_self = as_rotation(self.att)
        if isinstance(other, Pose):
            att_other = as_rotation(other.att)
            pos = att_self.inv().apply(other.pos - self.pos)
            att = att_self.inv() * att_other
            return Pose(pos, att)
        elif isinstance(other, np.ndarray):
            pos = att_self.inv().apply(other - self.pos)
            return pos
        elif isinstance(other, R):
            att = att_self.inv() * other
            return att
        else:
            raise ValueError(f'Invalid type for other: {type(other)}')


if __name__ == '__main__':
    ref = Pose([1, 2, 3], [10, -20, 30])
    rel = Pose([4, 5, 6], [-90, -45, 45])
    print(ref)
    print(rel)
    rel_in_std = ref.from_frame(rel)
    print(rel_in_std)
    rel_in_ref = ref.to_frame(rel_in_std)
    print(rel_in_ref)
