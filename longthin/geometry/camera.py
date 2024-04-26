from dataclasses import dataclass
import numpy as np

from .common import *


@dataclass(frozen=True)
class CameraParams:
    hfov: float
    vfov: float
    width: float
    height: float
    cxf: float = None
    cyf: float = None

    def copy(self):
        return CameraParams(self.hfov, self.vfov, self.width, self.height, self.cxf, self.cyf)

    @property
    def fx(self):
        return self.width / (2 * np.tan(np.deg2rad(self.hfov / 2)))

    @property
    def fy(self):
        return self.height / (2 * np.tan(np.deg2rad(self.vfov / 2)))

    @property
    def cx(self):
        return self.width * self.cxf

    @property
    def cy(self):
        return self.height * self.cyf

    @property
    def image_size(self):
        return np.array([self.width, self.height])

    @staticmethod
    def from_params(fx, fy, cx, cy, width, height):
        hfov = np.rad2deg(2 * np.arctan(width / (2 * fx)))
        vfov = np.rad2deg(2 * np.arctan(height / (2 * fy)))
        cxf = cx / width
        cyf = cy / height
        return CameraParams(hfov, vfov, width, height, cxf, cyf)

    @staticmethod
    def from_matrix(mat, width=None, height=None):
        fx = mat[0, 0]
        fy = mat[1, 1]
        cx = mat[0, 2]
        cy = mat[1, 2]
        return CameraParams.from_params(fx, fy, cx, cy, width, height)

    def scale(self, scale):
        return CameraParams(
            self.hfov, self.vfov, int(self.width * scale), int(self.height * scale), self.cxf, self.cyf)

    def to_params(self):
        return self.fx, self.fy, self.cx, self.cy, self.width, self.height

    def to_matrix(self):
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])

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
        rays[:, 0] = (rays[:, 0] / self.width - self.cxf) * 2
        rays[:, 1] = (rays[:, 1] / self.height - self.cyf) * 2
        rays = rays[:, 0:1] * vec_h + rays[:, 1:2] * vec_v + vec
        rays /= np.linalg.norm(rays, axis=1, keepdims=True)
        return rays
