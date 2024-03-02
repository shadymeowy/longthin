import numpy as np
import cv2
from dataclasses import dataclass
from typing import Dict

from .graphics import LTRendererParams
from .geometry import *
from .marker import MarkerHelper


@dataclass
class Estimator:
    marker_ids: np.ndarray
    marker_corners: np.ndarray
    camera_params: CameraParams
    dist_params: Distortion = None
    vehicle_pose: Pose = None
    marker_dict = cv2.aruco.DICT_4X4_50
    marker_helper = MarkerHelper.default()

    def __post_init__(self, median_num=3):
        self.history_pos = []
        self.history_att = []
        self.median_num = median_num

    def estimate(self, img, draw=False):
        if self.dist_params is not None:
            img_ud = self.dist_params.undistort(img)
        else:
            img_ud = img
        img_gray = cv2.cvtColor(img_ud, cv2.COLOR_BGR2GRAY)

        corners2, ids = self.marker_helper.detect(img_gray)
        if draw:
            img_markers = self.marker_helper.draw(img_ud, corners2, ids)
        else:
            img_markers = None
        if ids is None:
            return None
        mask = np.isin(ids, self.marker_ids)
        ids = ids[mask]
        corners2 = corners2[mask]
        corners3 = self.marker_corners[ids]
        if corners2.size == 0:
            return None
        ids = ids.reshape((-1))

        if self.dist_params is not None:
            m_intrinsics = self.dist_params.m_intrinsics_2
            m_distortion = self.dist_params.m_distortion
        else:
            m_intrinsics = self.camera_params.to_matrix()
            m_distortion = None
        obj_points = corners3.reshape((-1, 3))[..., [1, 2, 0]]
        obj_points = obj_points.astype(np.float64)
        img_points = corners2.reshape((-1, 2))
        img_points = img_points.astype(np.float64)
        ret, rvec, tvec, _ = cv2.solvePnPRansac(obj_points, img_points, m_intrinsics, m_distortion)
        if not ret:
            return None

        rvec = np.array([rvec[2], rvec[0], rvec[1]])
        tvec = np.array([tvec[2], tvec[0], tvec[1]])
        rmat, _ = cv2.Rodrigues(rvec)
        rmat = rmat.T
        tvec = -rmat @ tvec
        est_pose = Pose(tvec.flatten(), rmat)

        self.history_pos.append(est_pose.pos)
        self.history_att.append(est_pose.att)
        if len(self.history_pos) > self.median_num:
            self.history_pos.pop(0)
            self.history_att.pop(0)
            est_pose.pos = np.median(self.history_pos, axis=0)
            est_pose.att = np.median(self.history_att, axis=0)

        return est_pose, corners3, img_markers
