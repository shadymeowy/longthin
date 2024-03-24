import numpy as np
import cv2
from dataclasses import dataclass
import time

from .geometry import *
from .marker import MarkerHelper
from .abg import alpha_beta_filter


@dataclass
class Estimator:
    marker_ids: np.ndarray = None
    marker_corners: np.ndarray = None
    camera_params: CameraParams = None
    dist_params: Distortion = None
    camera_rel_pose: Pose = None
    marker_dict = cv2.aruco.DICT_4X4_50
    marker_helper = MarkerHelper.default()

    @staticmethod
    def from_config(config):
        dist_params = Distortion.from_params(**config.camera.model._asdict())
        camera_params = dist_params.camera_params_2
        camera_rel_pose = Pose(config.camera.pose.position, config.camera.pose.attitude)

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

        return Estimator(
            marker_ids,
            marker_corners,
            camera_params,
            dist_params,
            camera_rel_pose,
        )

    def estimate(self, img, draw=False):
        img_ud = self.dist_params.undistort(img)
        img_gray = cv2.cvtColor(img_ud, cv2.COLOR_BGR2GRAY)

        corners2, ids = self.marker_helper.detect(img_gray)
        if ids is None:
            return None
        mask = np.isin(ids, self.marker_ids)
        # remove markers that have corners too close to the edges
        mask &= np.all(corners2[:, :, 0] > 32, axis=1)
        mask &= np.all(corners2[:, :, 0] < img_gray.shape[1] - 32, axis=1)
        mask &= np.all(corners2[:, :, 1] > 32, axis=1)
        mask &= np.all(corners2[:, :, 1] < img_gray.shape[0] - 32, axis=1)
        ids = ids[mask]
        corners2 = corners2[mask]
        if draw:
            img_markers = self.marker_helper.draw(img_ud, corners2, ids)
        else:
            img_markers = None
        sorter = np.argsort(self.marker_ids)
        idx = sorter[np.searchsorted(self.marker_ids, ids, sorter=sorter)]
        if not (ids == self.marker_ids[idx]).all():
            raise ValueError('ids are not matched')
        corners3 = self.marker_corners[idx]
        if corners2.size == 0:
            return None
        ids = ids.reshape((-1))

        # TODO check this
        m_int = self.dist_params.m_intrinsics_2
        obj_points = corners3.reshape((-1, 3))[..., [1, 2, 0]]
        obj_points = obj_points.astype(np.float64)
        img_points = corners2.reshape((-1, 2))
        img_points = img_points.astype(np.float64)
        ret, rvec, tvec, _ = cv2.solvePnPRansac(obj_points, img_points, m_int, None, flags=cv2.SOLVEPNP_SQPNP)
        if not ret:
            return None

        rvec = np.array([rvec[2], rvec[0], rvec[1]])
        tvec = np.array([tvec[2], tvec[0], tvec[1]])
        rmat, _ = cv2.Rodrigues(rvec)
        rmat = rmat.T
        tvec = -rmat @ tvec
        est_pose_cam = Pose(tvec.flatten(), rmat)
        camera_rel_pose = self.camera_rel_pose
        est_pose = est_pose_cam.from_frame(camera_rel_pose.inv())
        return est_pose, corners3, img_markers
