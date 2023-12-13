import numpy as np
import cv2
from dataclasses import dataclass

from ltparams import LTParams
from geometry import *
from pose import Pose
from marker import *


@dataclass
class Estimator:
    params: LTParams

    camera_pose: Pose = None
    vehicle_pose: Pose = None

    # markers = None
    marker_dist: np.ndarray = None

    def __post_init__(self):
        self.camera_pose = Pose(self.params.camera_pos_rel,
                                self.params.camera_att_rel)
        self.vehicle_pose = Pose(self.params.vehicle_pos,
                                 self.params.vehicle_att)

        w = self.params.area_w + 2 * self.params.strip_w
        h = self.params.area_h + 2 * self.params.strip_w
        n = self.params.marker_n
        alt = self.params.marker_alt
        # self.markers = marker_gen(n)
        self.marker_dist = marker_distribute(n, w, h, alt)

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
        actual_corners = self.corner_position(ids)[:, :2]
        actual_corners = actual_corners.reshape((-1, 2))
        print(actual_corners)
        return corners, actual_corners, ids

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
