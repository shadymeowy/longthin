import numpy as np
import cv2
from dataclasses import dataclass
from typing import Dict

from .geometry import *
from .marker import MarkerHelper


@dataclass
class Estimator:
    marker_ids: np.ndarray = None
    marker_corners: np.ndarray = None
    camera_params: CameraParams = None
    dist_params: Distortion = None
    vehicle_pose: Pose = None
    marker_dict = cv2.aruco.DICT_4X4_50
    marker_helper = MarkerHelper.default()
    median_num: int = 3

    def __post_init__(self):
        self.history_pos = []
        self.history_att = []

    @staticmethod
    def from_config(config):
        dist_params = Distortion.from_params(**config.camera.model._asdict())
        camera_params = dist_params.camera_params_2

        marker_corners = []
        marker_ids = []
        for marker in config.markers:
            pose = Pose(marker.pose.position, marker.pose.attitude)
            s = marker.size
            corners = np.array([
                [0, -s/2, -s],
                [0, s/2, -s],
                [0, s/2, 0],
                [0, -s/2, 0]
            ])
            corners = pose.from_frame(corners)
            marker_corners.append(corners)
            marker_ids.append(marker.id)
        marker_corners = np.array(marker_corners)
        marker_ids = np.array(marker_ids)

        return Estimator(marker_ids, marker_corners, camera_params, dist_params)

    def estimate(self, img, draw=False):
        img_ud = self.dist_params.undistort(img)
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
        sorter = np.argsort(self.marker_ids)
        idx = sorter[np.searchsorted(self.marker_ids, ids, sorter=sorter)]
        if not (ids == self.marker_ids[idx]).all():
            raise ValueError('ids are not matched')
        corners3 = self.marker_corners[idx]
        if corners2.size == 0:
            return None
        ids = ids.reshape((-1))

        # TODO check this
        m_intrinsics_2 = self.dist_params.m_intrinsics
        # m_distortion = self.dist_params.m_distortion
        obj_points = corners3.reshape((-1, 3))[..., [1, 2, 0]]
        obj_points = obj_points.astype(np.float64)
        img_points = corners2.reshape((-1, 2))
        img_points = img_points.astype(np.float64)
        ret, rvec, tvec = cv2.solvePnP(obj_points, img_points, m_intrinsics_2, None)
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
