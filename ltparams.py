from dataclasses import dataclass
import numpy as np
from distortion import Distortion


@dataclass
class LTParams:
    area_w: float = 3  # width of area
    area_h: float = 3  # height of area
    spot_w: float = None  # width of spot (1.5x vehicle)
    spot_l: float = 75e-2  # length of spot
    spot_percent: float = 0.3  # percent of to side length
    strip_w: float = 10e-2  # width of strip

    vehicle_pos: np.ndarray = np.array([0., 0., 0.])  # initial position
    vehicle_att: np.ndarray = np.array([0., 0., 0.])  # initial attitude

    chassis_l: float = 50e-2  # length
    chassis_w: float = 10e-2  # width
    chassis_h: float = 10e-2  # height
    chassis_offset: float = 10e-2  # from center
    chassis_pos_rel: np.ndarray = None  # determined by others
    wheel_r: float = 5e-2  # radius
    wheel_w: float = 2e-2  # width
    wheel_offset: float = 5e-2  # from the center

    camera_alt: float = -25e-2  # from ground
    camera_pos_rel: np.ndarray = None  # determined by others
    camera_att_rel: np.ndarray = np.array(
        [0, -30, 0])  # attitude relative to vehicle

    # rewritten if distortion is active
    camera_hfov: float = 102  # horizontal field of view
    camera_vfov: float = 85.6  # vertical field of view
    camera_width: int = 960  # width of image
    camera_height: int = 720  # height of image

    distort_enable: bool = True  # whether to apply distortion to camera
    distort_path: str = 'other/calibration.txt'  # distortion parameters
    distort_params: Distortion = None

    marker_w: float = 20e-2  # width of marker
    marker_h: float = 20e-2  # height of marker
    marker_alt: float = 0.e-2  # from ground
    marker_n: int = 2  # per side
    marker_pitch: float = 1.  # degrees

    checker_enable: bool = False
    checker_size: float = 22.5e-3  # size of checker
    checker_alt: float = 0.e-2
    checker_nw: int = 7  # number of checkers in width
    checker_nh: int = 10  # number of checkers in height
    checker_pitch: float = -90.  # degrees
    checker_offset: float = 15.e-2  # offset from center of vehicle

    homography_calibration: bool = False
    homography_calib_enable: bool = True
    homography_calib_path: str = 'other/hcalib.txt'
    homography_calib_data: np.ndarray = None

    def __post_init__(self):
        if self.spot_w is None:
            self.spot_w = self.chassis_w * 1.5
        if self.chassis_pos_rel is None:
            self.chassis_pos_rel = np.array([
                self.chassis_l/2 - self.chassis_offset,
                0,
                -self.wheel_r
            ])
        if self.camera_pos_rel is None:
            self.camera_pos_rel = np.array([
                self.chassis_l-self.chassis_offset,
                0,
                self.camera_alt
            ])
        if self.distort_enable:
            self.distort_params = Distortion.from_file(self.distort_path)
            self.camera_hfov = np.rad2deg(self.distort_params.hfov)
            self.camera_vfov = np.rad2deg(self.distort_params.vfov)
            self.camera_width = self.distort_params.width2
            self.camera_height = self.distort_params.height2
        else:
            self.distort_params = None
        if self.homography_calib_enable:
            self.homography_calib_data = np.loadtxt(self.homography_calib_path)
