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
    marker_landmarks: np.ndarray = None
    marker_goals: np.ndarray = None
    marker_corners: np.ndarray = None
    dist_params: Distortion = None
    camera_rel_pose: Pose = None
    marker_dict = cv2.aruco.DICT_4X4_50
    marker_helper = MarkerHelper.default()

    @staticmethod
    def from_config(config):
        dist_params = Distortion.from_params(**config.camera.model._asdict())
        camera_rel_pose = Pose(config.camera.pose.position, config.camera.pose.attitude)

        marker_corners = []
        marker_ids = []
        marker_landmarks = []
        marker_goals = []
        for marker in config.markers:
            if marker.role == 'landmark':
                marker_landmarks.append(marker.id)
                marker_ids.append(marker.id)
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
            elif marker.role == 'goal':
                marker_goals.append(marker.id)

        marker_corners = np.array(marker_corners)
        marker_ids = np.array(marker_ids)
        marker_landmarks = np.array(marker_landmarks)
        marker_goals = np.array(marker_goals)

        return Estimator(
            marker_ids,
            marker_landmarks,
            marker_goals,
            marker_corners,
            dist_params,
            camera_rel_pose,
        )

    def estimate(self, img, draw=False):
        img_ud = self.dist_params.undistort(img)
        img_gray = cv2.cvtColor(img_ud, cv2.COLOR_BGR2GRAY)

        corners2, ids = self.marker_helper.detect(img_gray)
        if ids is None:
            return None, None, img_ud, None, img_ud

        img_markers = img_ud.copy()
        mask_goal = np.isin(ids, self.marker_goals)
        mask_landmark = np.isin(ids, self.marker_landmarks)

        if np.any(mask_goal):
            ids_goal = ids[mask_goal]
            corners2_goal = corners2[mask_goal]
            goal_points = corners2_goal.reshape((-1, 2))
            goal_points = goal_points.astype(np.float32)
        else:
            corners2_goal = np.array([])
            goal_points = None

        if np.any(mask_landmark):
            # remove markers that have corners too close to the edges
            mask_landmark &= np.all(corners2[:, :, 0] > 32, axis=1)
            mask_landmark &= np.all(corners2[:, :, 0] < img_gray.shape[1] - 32, axis=1)
            mask_landmark &= np.all(corners2[:, :, 1] > 32, axis=1)
            mask_landmark &= np.all(corners2[:, :, 1] < img_gray.shape[0] - 32, axis=1)

        if np.any(mask_landmark):
            ids_landmark = ids[mask_landmark]
            corners2_landmark = corners2[mask_landmark]

            sorter = np.argsort(self.marker_landmarks)
            idx = sorter[np.searchsorted(self.marker_landmarks, ids_landmark, sorter=sorter)]
            if not (ids_landmark == self.marker_landmarks[idx]).all():
                raise ValueError('ids_landmark are not matched')
            corners3 = self.marker_corners[idx]

            # TODO check this
            m_int = self.dist_params.m_intrinsics_2
            obj_points = corners3.reshape((-1, 3))[..., [1, 2, 0]]
            obj_points = obj_points.astype(np.float64)
            img_points = corners2_landmark.reshape((-1, 2))
            img_points = img_points.astype(np.float64)
            ret, rvec, tvec, _ = cv2.solvePnPRansac(obj_points, img_points, m_int, None, flags=cv2.SOLVEPNP_SQPNP)
            if ret:
                rvec = np.array([rvec[2], rvec[0], rvec[1]])
                tvec = np.array([tvec[2], tvec[0], tvec[1]])
                rmat, _ = cv2.Rodrigues(rvec)
                rmat = rmat.T
                tvec = -rmat @ tvec
                est_pose_cam = Pose(tvec.flatten(), rmat)
            else:
                est_pose_cam = None
        else:
            ids_landmark = None
            corners3 = None
            est_pose_cam = None

        if draw and ids_landmark is not None:
            img_markers = self.marker_helper.draw(img_markers, corners2_landmark, ids_landmark)

        if draw and goal_points is not None:
            for goal in goal_points:
                img_markers = cv2.circle(img_markers, tuple(map(int, goal)), 3, (0, 0, 255), -1)
        return est_pose_cam, corners3, img_markers, goal_points, img_ud
