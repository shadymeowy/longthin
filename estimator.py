import numpy as np
import cv2
from dataclasses import dataclass

from ltparams import LTParams
from geometry import *
from pose import Pose
from marker import *
from camera import CameraParams


@dataclass
class Estimator:
    params: LTParams

    camera_pose: Pose = None
    vehicle_pose: Pose = None

    # markers = None
    marker_dist: np.ndarray = None

    hmat: np.ndarray = None

    def __post_init__(self):
        params = self.params
        self.camera_pose = Pose(params.camera_pos_rel,
                                params.camera_att_rel)
        self.camera_params = CameraParams(params.camera_hfov, params.camera_vfov,
                                          params.camera_width, params.camera_height)

        w = self.params.area_w + 2 * self.params.strip_w
        h = self.params.area_h + 2 * self.params.strip_w
        n = self.params.marker_n
        alt = self.params.marker_alt
        # self.markers = marker_gen(n)
        self.marker_dist = marker_distribute(n, w, h, alt)

        w = self.params.camera_width
        h = self.params.camera_height
        # calculate homography matrix
        # a portion from the bottom of the image
        # temporary solution
        p_image = np.array([
            [w/2 - 100, h - 100],
            [w/2 + 100, h - 100],
            [w/2 + 100, h],
            [w/2 - 100, h]
        ], dtype=np.float32)
        rays = self.camera_params.rays(self.camera_pose.att, p_image)
        pg, ng = np.array([0., 0., 0.]), np.array([0., 0., 1.])
        p_world = np.array([intersection_plane_line(
            (pg, ng), (self.camera_pose.pos, r)) for r in rays])
        p_world = p_world[:, :2]
        p_world = p_world.astype(np.float32)
        self.hmat = cv2.findHomography(p_image, p_world)[0]

    def estimate(self, img):
        if self.params.distort_active:
            img_ud = self.params.distort_params.undistort(img)
        else:
            img_ud = img
        corners, ids = marker_detect(img_ud)
        img_markers = marker_draw(img_ud, corners, ids)
        cv2.imshow('markers', img_markers)
        if corners.size == 0:
            return None
        corners = corners[:, :, 2:].reshape((-1, 2))
        ids = ids.reshape((-1))
        actual_corners = self.corner_position(ids)[:, 2:]
        actual_corners = actual_corners.reshape((-1, 3))

        calculated_corners = np.zeros_like(corners)
        for i in range(len(corners)):
            p = np.array([corners[i, 0], corners[i, 1], 1.])
            p = self.hmat @ p
            p /= p[2]
            calculated_corners[i] = p[:2]
        return corners, actual_corners, ids, calculated_corners

    def corner_position(self, id_):
        if isinstance(id_, (list, tuple, np.ndarray)):
            return np.array([self.corner_position(i) for i in id_])
        pos = self.marker_dist[id_][:3]
        yaw = self.marker_dist[id_][3]
        marker_pose = Pose(pos, [0, self.params.marker_pitch, yaw])
        w = self.params.marker_w
        h = self.params.marker_h
        corners = np.array([
            [0, -w/2, -h],
            [0, w/2, -h],
            [0, w/2, 0],
            [0, -w/2, 0]
        ])
        corners = marker_pose.from_frame(corners)
        return corners
