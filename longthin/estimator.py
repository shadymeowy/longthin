import numpy as np
import cv2
from dataclasses import dataclass

from .ltparams import LTParams
from .geometry import *
from .pose import Pose
from .marker import *
from .camera import CameraParams


@dataclass
class Estimator:
    params: LTParams
    camera_pose: Pose = None
    vehicle_pose: Pose = None
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
        if params.homography_calib_enable:
            self.hmat = params.homography_calib_data
        else:
            self.hmat = self.calculate_homography()

        self.corners = None
        self.act_corners = None

    def estimate(self, img):
        params = self.params
        if params.distort_enable:
            img_ud = params.distort_params.undistort(img)
        else:
            img_ud = img
        img_gray = cv2.cvtColor(img_ud, cv2.COLOR_BGR2GRAY)
        if params.homography_calibration:
            self.calibrate_homography(img_gray)

        corners, ids = marker_detect(img_gray)
        img_markers = marker_draw(img_ud, corners, ids)
        cv2.imshow('markers', img_markers)
        if ids is None:
            return None
        mask = ids < len(params.markers)
        corners = corners[mask]
        ids = ids[mask]
        if corners.size == 0:
            return None
        corners = corners.reshape((-1, 4, 2))
        corners = corners[:, 2:, :].reshape((-1, 2))
        ids = ids.reshape((-1))
        actual_corners = self.corner_position(ids)[:, 2:, :2]
        actual_corners = actual_corners.reshape((-1, 2))
        actual_corners = actual_corners.astype(np.float64)

        calculated_corners = np.zeros_like(corners)
        for i in range(len(corners)):
            p = np.array([corners[i, 0], corners[i, 1], 1.])
            p = self.hmat @ p
            p /= p[2]
            calculated_corners[i] = p[:2]

        # m, inliers = cv2.estimateAffinePartial2D(
        #    calculated_corners, actual_corners)
        # tx = m[0, 2]
        # ty = m[1, 2]
        # yaw = np.arctan2(m[1, 0], m[0, 0])
        # yaw = np.rad2deg(yaw)
        # est = np.array([tx, ty, yaw])

        R, T = estimate_rigid_transform(
            calculated_corners, actual_corners[..., :2].astype(np.float64))
        yaw = np.arctan2(R[1, 0], R[0, 0])
        yaw = np.rad2deg(yaw)
        est = np.array([T[0], T[1], yaw])
        return est, corners, ids, actual_corners, calculated_corners

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

    def calculate_homography(self):
        # calculate homography matrix
        # a portion from the bottom of the image
        # temporary solution
        w = self.params.camera_width
        h = self.params.camera_height
        p_image = np.array([
            [w/2 - 256, h - 256],
            [w/2 + 256, h - 256],
            [w/2 + 256, h],
            [w/2 - 256, h]
        ], dtype=np.float32)
        rays = self.camera_params.rays(self.camera_pose.att, p_image)
        pg, ng = np.array([0., 0.,
                           -self.params.camera_alt
                           + self.params.marker_alt]
                          ), np.array([0., 0., 1.])
        o = np.array([0., 0., 0.])
        p_world = np.array(
            [intersection_plane_line((pg, ng), (o, r)) for r in rays])
        p_world = p_world[:, :2]
        p_world = p_world.astype(np.float32)
        hmat = cv2.findHomography(p_image, p_world)[0]
        return hmat

    def calibrate_homography(self, img):
        # use checkerboard to calibrate homography
        ret, corners = cv2.findChessboardCorners(
            img, (self.params.checker_nw - 1, self.params.checker_nh - 1),
            cv2.CALIB_CB_ADAPTIVE_THRESH)
        if not ret:
            return
        print('found checkerboard')
        corners = corners.reshape((-1, 2))
        img_draw = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        cv2.drawChessboardCorners(
            img_draw, (self.params.checker_nh - 1, self.params.checker_nw - 1), corners, ret)
        # index of corners in the image
        corners = cv2.cornerSubPix(
            img, corners, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        # sort corners, first agains y, then against x
        value = corners[:, 0] + corners[:, 1]*100
        corners = corners[np.argsort(-value)]
        for i in range(self.params.checker_nw-1):
            cv2.putText(img_draw, str(i), tuple(
                corners[i].astype(np.int32)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
        cv2.imshow('checkerboard', img_draw)
        # use calculated homography matrix to calculate world coordinates
        # to compare with checkerboard corners
        hmat_calc = self.calculate_homography()
        p_world = np.array([corners[:, 0], corners[:, 1],
                           np.ones_like(corners[:, 0])])
        p_world = hmat_calc @ p_world
        p_world /= p_world[2]
        p_world = p_world[:2]
        p_world = p_world.T

        # checkerboard pose
        pose = Pose(np.array([
            self.params.checker_offset,
            self.params.camera_pos_rel[1],
            self.params.checker_alt]),
            np.array([0., 0, 0.]))
        # checkerboard corners
        w = self.params.checker_size * self.params.checker_nw
        # h = self.params.checker_size * self.params.checker_nh
        act_corners = np.meshgrid(np.arange(1, self.params.checker_nw)[::-1],
                                  np.arange(1, self.params.checker_nh))
        act_corners = np.stack(act_corners[::-1], axis=-1)
        act_corners = act_corners.astype(np.float32)
        act_corners = act_corners.reshape((-1, 2))
        act_corners[:, 0] *= self.params.checker_size
        act_corners[:, 1] *= self.params.checker_size
        act_corners -= np.array([0, w/2])
        act_corners = np.hstack(
            (act_corners, np.zeros((act_corners.shape[0], 1))))
        act_corners = pose.from_frame(act_corners)[:, :2]

        if self.corners is None:
            self.corners = corners
            self.act_corners = act_corners
        else:
            self.corners = np.vstack((self.corners, corners))
            self.act_corners = np.vstack((self.act_corners, act_corners))

        # calculate homography matrix
        hmat_calib = cv2.findHomography(self.corners, self.act_corners, cv2.RANSAC)[0]
        print('hmat_calib', hmat_calib)
        print('hmat_calc', hmat_calc)
        error = (hmat_calib - hmat_calc) / np.max(np.abs(hmat_calc)) * 100
        print('error', error)
        return hmat_calib
