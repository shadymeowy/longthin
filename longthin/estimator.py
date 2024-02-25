import numpy as np
import cv2
from dataclasses import dataclass

from .graphics import LTParams
from .geometry import *
from .marker import MarkerHelper


@dataclass
class Estimator:
    params: LTParams
    vehicle_pose: Pose = None
    markerhelper: MarkerHelper = None

    def __post_init__(self, median_num=3):
        params = self.params
        self.camera_params = CameraParams(params.camera_hfov, params.camera_vfov,
                                          params.camera_width, params.camera_height)
        self.markerhelper = MarkerHelper.from_type()
        self.history_pos = []
        self.history_att = []
        self.median_num = median_num

    def estimate(self, img, draw=False):
        params = self.params
        if params.distort_enable:
            img_ud = params.distort_params.undistort(img)
        else:
            img_ud = img
        img_gray = cv2.cvtColor(img_ud, cv2.COLOR_BGR2GRAY)

        corners, ids = self.markerhelper.detect(img_gray)
        if draw:
            img_markers = self.markerhelper.draw(img_ud, corners, ids)
        else:
            img_markers = None
        if ids is None:
            return None
        mask = ids < len(params.markers)
        corners = corners[mask]
        ids = ids[mask]
        if corners.size == 0:
            return None
        ids = ids.reshape((-1))
        corners_pos = self.corner_position(ids)

        if params.distort_enable:
            m_intrinsics = params.distort_params.m_intrinsics_2
            m_distortion = params.distort_params.m_distortion
        else:
            m_intrinsics = self.camera_params.to_matrix()
            m_distortion = None
        obj_points = corners_pos.reshape((-1, 3))[..., [1, 2, 0]]
        obj_points = obj_points.astype(np.float64)
        img_points = corners.reshape((-1, 2))
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

        return est_pose, corners_pos, img_markers

    def corner_position(self, id_):
        if isinstance(id_, (list, tuple, np.ndarray)):
            return np.array([self.corner_position(i) for i in id_])
        markers = self.params.markers
        pos = markers[id_][:3]
        yaw = markers[id_][3]
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
