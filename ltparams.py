from dataclasses import dataclass
import numpy as np


@dataclass
class LTParams:
    area_w: float = 3  # width of area
    area_h: float = 3  # height of area
    spot_w: float = None  # width of spot (1.5x vehicle)
    spot_l: float = 75e-2  # length of spot
    spot_percent: float = 0.3  # percent of to side length
    strip_w: float = 10e-2  # width of strip

    vehicle_pos: np.ndarray = np.array([0., 0., 0.]) # initial position
    vehicle_att: np.ndarray = np.array([0., 0., 0.]) # initial attitude

    chassis_l: float = 50e-2  # length
    chassis_w: float = 10e-2  # width
    chassis_h: float = 10e-2  # height
    chassis_offset: float = 10e-2  # from center
    chassis_pos_rel: np.ndarray = None  # determined by others
    wheel_r: float = 5e-2  # radius
    wheel_w: float = 2e-2  # width
    wheel_offset: float = 5e-2  # from the center

    camera_alt: float = -20e-2  # from ground
    camera_pos_rel: np.ndarray = None  # determined by others
    camera_att_rel: np.ndarray = np.array(
        [0, -35, 0])  # attitude relative to vehicle
    camera_hfov: float = 102  # horizontal field of view
    camera_vfov: float = 85.6  # vertical field of view
    camera_width: int = 960  # width of image
    camera_height: int = 720  # height of image

    marker_w: float = 20e-2  # width of marker
    marker_h: float = 20e-2  # height of marker
    marker_alt: float = 0.e-2  # from ground
    marker_n: int = 2  # per side
    marker_pitch: float = 1.  # degrees

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
