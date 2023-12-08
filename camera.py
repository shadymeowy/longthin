from dataclasses import dataclass
import numpy as np

from pose import Pose
from geometry import *


@dataclass
class Camera:
    pose: Pose
    hfov: float
    vfov: float
    width: float
    height: float

    def copy(self):
        return Camera(self.pose.copy(), self.hfov, self.vfov, self.width, self.height)

    @property
    def att(self):
        return self.pose.att

    @att.setter
    def att(self, att):
        self.pose.att = att

    @property
    def pos(self):
        return self.pose.pos

    @pos.setter
    def pos(self, pos):
        self.pose.pos = pos

    @property
    def fx(self):
        return self.width / (2 * np.tan(np.deg2rad(self.hfov / 2)))

    @property
    def fy(self):
        return self.height / (2 * np.tan(np.deg2rad(self.vfov / 2)))

    @property
    def cx(self):
        return self.width / 2

    @property
    def cy(self):
        return self.height / 2

    def image_plane_vecs(self, vec=E_X, vec_h=E_Y, vec_v=E_Z):
        vfovh = np.deg2rad(self.vfov) / 2
        hfovh = np.deg2rad(self.hfov) / 2
        rot = as_rotation(self.att)
        vec = rot.apply(vec)
        vec_v = rot.apply(vec_v)
        vec_h = rot.apply(vec_h)
        vec_v = vec_v * np.tan(vfovh)
        vec_h = vec_h * np.tan(hfovh)
        return (vec, vec_v, vec_h)

    def ground_points(self, max_dist=2., ground_alt=0.):
        vec, vec_v, vec_h = self.image_plane_vecs()
        vecs = [
            vec + vec_v + vec_h,
            vec + vec_v - vec_h,
            vec - vec_v + vec_h,
            vec - vec_v - vec_h,
        ]
        pg = np.array([0., 0., ground_alt])
        ng = np.array([0., 0., 1.])
        bpoints = [intersection_plane_line(
            (pg, ng), (self.pose.pos, v)) for v in vecs]
        for i, point in enumerate(bpoints):
            if np.dot(point - self.pose.pos, vec) < 0:
                p = self.pose.pos + max_dist * vecs[i]
                # project onto plane
                p = intersection_plane_line((pg, ng), (p, ng))
                bpoints[i] = p

        return bpoints

    def rays(self, img_points):
        vec, vec_v, vec_h = self.image_plane_vecs()
        rays = []
        for p in img_points:
            p = np.array(p, np.float64)
            p[0] = (p[0] / self.width - 0.5) * 2
            p[1] = (p[1] / self.height - 0.5) * 2
            p = p[0] * vec_h + p[1] * vec_v + vec
            p /= norm(p)
            rays.append(p)
        return rays

    def with_pose(self, pose):
        return Camera(pose, self.hfov, self.vfov, self.width, self.height)
