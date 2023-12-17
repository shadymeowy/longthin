from dataclasses import dataclass
import numpy as np

from .geometry import *


@dataclass(frozen=True)
class CameraParams:
    hfov: float
    vfov: float
    width: float
    height: float

    def copy(self):
        return CameraParams(self.hfov, self.vfov, self.width, self.height)

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

    def image_plane_vecs(self, att, vec=E_X, vec_h=E_Y, vec_v=E_Z):
        vfovh = np.deg2rad(self.vfov) / 2
        hfovh = np.deg2rad(self.hfov) / 2
        rot = as_rotation(att)
        vec = rot.apply(vec)
        vec_v = rot.apply(vec_v)
        vec_h = rot.apply(vec_h)
        vec_v = vec_v * np.tan(vfovh)
        vec_h = vec_h * np.tan(hfovh)
        return (vec, vec_v, vec_h)

    def ground_points(self, pose, max_dist=2., ground_alt=0.):
        vec, vec_v, vec_h = self.image_plane_vecs(pose.att)
        vecs = [
            vec + vec_v + vec_h,
            vec + vec_v - vec_h,
            vec - vec_v + vec_h,
            vec - vec_v - vec_h,
        ]
        pg = np.array([0., 0., ground_alt])
        ng = np.array([0., 0., 1.])
        bpoints = [intersection_plane_line(
            (pg, ng), (pose.pos, v)) for v in vecs]
        for i, point in enumerate(bpoints):
            if np.dot(point - pose.pos, vec) < 0:
                p = pose.pos + max_dist * vecs[i]
                # project onto plane
                p = intersection_plane_line((pg, ng), (p, ng))
                bpoints[i] = p

        return bpoints

    def rays(self, att, img_points):
        vec, vec_v, vec_h = self.image_plane_vecs(att)
        rays = np.array(img_points, np.float64)
        rays[:, 0] = (rays[:, 0] / self.width - 0.5) * 2
        rays[:, 1] = (rays[:, 1] / self.height - 0.5) * 2
        rays = rays[:, 0:1] * vec_h + rays[:, 1:2] * vec_v + vec
        rays /= np.linalg.norm(rays, axis=1, keepdims=True)
        return rays