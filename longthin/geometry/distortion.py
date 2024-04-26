import cv2
import numpy as np
from dataclasses import dataclass

from .camera import CameraParams


@dataclass
class Distortion:
    # calibration matrix
    m_intrinsics: np.ndarray
    m_distortion: np.ndarray
    # original image size
    width: int
    height: int
    # alpha for cv2.getOptimalNewCameraMatrix
    alpha: float = 0.

    # optimal calibration matrix
    m_intrinsics_2: np.ndarray = None
    roi: tuple = None
    # undistorted image size
    width2: int = None
    height2: int = None
    # undistortion map
    mapx: np.ndarray = None
    mapy: np.ndarray = None
    # redistortion map
    mapx_re: np.ndarray = None
    mapy_re: np.ndarray = None
    # camera parameters
    camera_params: CameraParams = None
    camera_params_2: CameraParams = None

    def __post_init__(self):
        size = (self.width, self.height)
        self.m_intrinsics_2, self.roi = cv2.getOptimalNewCameraMatrix(
            self.m_intrinsics, self.m_distortion, size, self.alpha)
        self.width2 = self.roi[2]
        self.height2 = self.roi[3]
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            self.m_intrinsics, self.m_distortion, None, self.m_intrinsics_2, size, 5)

        xs, ys = np.meshgrid(range(self.width), range(self.height))
        points_distorted = np.array(
            [xs.ravel(), ys.ravel()]).T.astype(np.float32)
        points_ud = cv2.undistortPoints(
            points_distorted, self.m_intrinsics, self.m_distortion, None, self.m_intrinsics_2)
        points_ud = points_ud.reshape(
            self.height, self.width, 2)
        self.mapx_re = points_ud[..., 0]
        self.mapy_re = points_ud[..., 1]

        self.camera_params = CameraParams.from_matrix(
            self.m_intrinsics, self.width, self.height)
        self.camera_params_2 = CameraParams.from_matrix(
            self.m_intrinsics_2, self.width2, self.height2)

    @staticmethod
    def from_params(fx, fy, cx, cy, width, height, k1, k2, p1, p2, k3):
        m_intrinsics = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        m_distortion = np.array([k1, k2, p1, p2, k3])
        return Distortion(m_intrinsics, m_distortion, width, height)

    def undistort(self, image: np.ndarray) -> np.ndarray:
        image_ud = cv2.remap(
            image, self.mapx, self.mapy, cv2.INTER_LINEAR)
        image_ud = image_ud[self.roi[1]:self.roi[1] + self.roi[3],
                            self.roi[0]:self.roi[0] + self.roi[2]]
        return image_ud

    def distort(self, image: np.ndarray, pad=(0, 0)) -> np.ndarray:
        h_pad, w_pad = pad
        image_rd = np.ones(
            (self.height + 2 * h_pad, self.width + 2 * w_pad, 3), np.uint8) * 255
        # color to verify the padding
        # image_rd[..., 0] = 255
        # image_rd[..., 2] = 255
        x, y, w, h = self.roi
        image_rd[h_pad + y:h_pad + y + h,
                 w_pad + x:w_pad + x + w] = image
        mapx_re = self.mapx_re + w_pad
        mapy_re = self.mapy_re + h_pad
        image_rd = cv2.remap(image_rd, mapx_re, mapy_re, cv2.INTER_LINEAR)
        return image_rd

    @staticmethod
    def from_file(file: str):
        # layout
        # m00 m01 m02 m10 m11 m12 m20 m21 m22
        # k1 k2 p1 p2 k3
        # width height
        with open(file, 'r') as f:
            lines = f.readlines()
        m_intrinsics = np.array(lines[0].split(), np.float32).reshape(3, 3)
        m_distortion = np.array(lines[1].split(), np.float32)
        width, height = map(int, lines[2].split())
        return Distortion(m_intrinsics, m_distortion, width, height)


if __name__ == '__main__':
    image_distorted = cv2.imread('other/distorted.jpg')
    distortion = Distortion.from_file('other/calibration.txt')
    image_undistorted = distortion.undistort(image_distorted)
    image_redistorted = distortion.distort(image_undistorted)
    image_reundistorted = distortion.undistort(image_redistorted)

    cv2.imshow('image_reundistorted', image_reundistorted)
    cv2.imshow('image_distorted', image_distorted)
    cv2.imshow('image_undistorted', image_undistorted)
    cv2.imshow('image_redistorted', image_redistorted)
    cv2.waitKey(0)
