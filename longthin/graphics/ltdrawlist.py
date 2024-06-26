import drawing3d
import numpy as np
from scipy.spatial.transform import Rotation as R

from ..geometry import *


class LTDrawList(drawing3d.DrawList):
    def plane(self, x, y, z, w, h, dw, dh):
        nw = int(np.ceil(w / dw))
        nh = int(np.ceil(h / dh))
        dw = w / nw
        dh = h / nh
        x -= w / 2
        y -= h / 2
        # draw a multi segment rectangle
        for i in range(nw):
            for j in range(nh):
                self.polygon([
                    [x + i * dw, y + j * dh, z],
                    [x + (i + 1) * dw, y + j * dh, z],
                    [x + (i + 1) * dw, y + (j + 1) * dh, z],
                    [x + i * dw, y + (j + 1) * dh, z],
                ])

    def cuboid(self, x, y, z, l, w, h, wireframe=False):
        # draw 6 rectangles centered at (x, y, z)
        polygons = []
        points_tb = np.array([
            [x, y, z],
            [x + l, y, z],
            [x + l, y + w, z],
            [x, y + w, z],
        ])
        points_tb -= np.array([l, w, h]) / 2
        polygons.append(points_tb.copy())
        points_tb[:, 2] += h
        polygons.append(points_tb)

        points_fb = np.array([
            [x, y, z],
            [x + l, y, z],
            [x + l, y, z + h],
            [x, y, z + h],
        ])
        points_fb -= np.array([l, w, h]) / 2
        polygons.append(points_fb.copy())
        points_fb[:, 1] += w
        polygons.append(points_fb)

        points_lr = np.array([
            [x, y, z],
            [x, y + w, z],
            [x, y + w, z + h],
            [x, y, z + h],
        ])
        points_lr -= np.array([l, w, h]) / 2
        polygons.append(points_lr.copy())
        points_lr[:, 0] += l
        polygons.append(points_lr)

        if wireframe:
            lines = []
            for polygon in polygons:
                lines.append(polygon[[0, 1]])
                lines.append(polygon[[1, 2]])
                lines.append(polygon[[2, 3]])
                lines.append(polygon[[3, 0]])
            self.lines(lines)
        else:
            for polygon in polygons:
                self.polygon(polygon)

    def draw_axis(self, x, y, z, l, w=1.0):
        self.style2(1.0, 0.0, 0.0, 1.0, w)
        self.line(x, y, z, x + l, y, z)
        self.style2(0.0, 1.0, 0.0, 1.0, w)
        self.line(x, y, z, x, y + l, z)
        self.style2(0.0, 0.0, 1.0, 1.0, w)
        self.line(x, y, z, x, y, z + l)

    def draw_camera(self, pose, camera, l=7.5e-2):
        vec, vec_v, vec_h = camera.image_plane_vecs(pose.att)
        pos = pose.pos
        vec *= l
        vec_v *= l
        vec_h *= l
        p1 = pos + vec + vec_v + vec_h
        p2 = pos + vec + vec_v - vec_h
        p3 = pos + vec - vec_v - vec_h
        p4 = pos + vec - vec_v + vec_h
        self.style2(1., 0., 0., 1., 2.)
        lines = np.array([
            [pos, pos + vec],
            [pos, p1],
            [pos, p2],
            [pos, p3],
            [pos, p4],
        ])
        self.lines(lines)
        self.style2(0., 0., 1., 1., 2.)
        lines = np.array([
            [p1, p2],
            [p2, p3],
            [p3, p4],
            [p4, p1],
        ])
        self.lines(lines)
        self.style2(0., 1., 0., 1., 2.)
        lines = np.array([
            [pos + vec, pos + vec + vec_v],
            [pos + vec, pos + vec + vec_h],
        ])
        self.lines(lines)
        # image plane
        self.style2(.43, .56, .95, 1., 2.)
        self.polyline([p1, p2, p3, p4, p1])
        self.style2(.43, .56, .95, .2, 1.)
        self.polygon([p1, p2, p3, p4])

    def draw_camera_field(self, pose, camera):
        bpoints = camera.ground_points(pose)
        points = np.array([
            bpoints[3],
            bpoints[1],
            bpoints[0],
            bpoints[2]
        ])
        self.style2(0., 1., 0., 0.1, 2.)
        self.polygon(points)
        self.style2(0., 1., 0., 0.2, 2.)
        self.polyline(points)

        lines = np.array([
            [pose.pos, bpoints[0]],
            [pose.pos, bpoints[1]],
        ])
        self.lines(lines)

    def draw_binary_grid(self, pose, width, height, data):
        # data = data[::-1, :]
        nw = data.shape[0]
        nh = data.shape[1]
        dw = width / nw
        dh = height / nh
        e = 1e-4
        for i in range(nw):
            for j in range(nh):
                if data[i, j]:
                    self.style2(1., 1., 1., 1., 1.)
                else:
                    self.style2(0., 0., 0., 1., 1.)
                points = np.array([
                    [0, j * dh - e, i * dw - e],
                    [0, (j + 1) * dh + e, i * dw - e],
                    [0, (j + 1) * dh + e, (i + 1) * dw + e],
                    [0, j * dh - e, (i + 1) * dw + e],
                ])
                points -= np.array([0, height / 2, width])
                points = pose.from_frame(points)
                self.polygon(points)
