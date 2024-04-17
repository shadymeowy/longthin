from enum import Enum
from dataclasses import dataclass, asdict


LTPARAMS_COUNT = 0x35


class LTParamValueType(Enum):
    LTPARAMS_TYPE_FLOAT = 0
    LTPARAMS_TYPE_UINT32 = 1
    LTPARAMS_TYPE_INT32 = 2


class LTParamType(Enum):
    RESERVED = 0x0
    QCOMP_ALPHA = 0x1
    QCOMP_BETA = 0x2
    THETA_KP = 0x3
    THETA_KI = 0x4
    THETA_KD = 0x5
    THETA_KI_LIMIT = 0x6
    ED_KP = 0x7
    PARKING_KP = 0x8
    PARKING_KI = 0x9
    PARKING_KD = 0xa
    PARKING_KI_LIMIT = 0xb
    FILLER4 = 0xc
    BLINK_PERIOD = 0xd
    MADGWICK_BETA = 0xe
    IMU_FILTER_TYPE = 0xf
    MAHONY_KP = 0x10
    IMU_CALIBRATION_SAMPLES = 0x11
    IMU_PUBLISH_PERIOD = 0x12
    MOTOR_PWM_REFRESH_PERIOD = 0x13
    CONTROL_DEBUG_ENABLE = 0x14
    CONTROL_DEBUG_PUBLISH_PERIOD = 0x15
    IMU_RAW_ENABLE = 0x16
    MOTOR_OUTPUT_ENABLE = 0x17
    MOTOR_OUTPUT_PUBLISH_PERIOD = 0x18
    MAG_CALIB_B0 = 0x19
    MAG_CALIB_B1 = 0x1a
    MAG_CALIB_B2 = 0x1b
    MAG_CALIB_M00 = 0x1c
    MAG_CALIB_M01 = 0x1d
    MAG_CALIB_M02 = 0x1e
    MAG_CALIB_M10 = 0x1f
    MAG_CALIB_M11 = 0x20
    MAG_CALIB_M12 = 0x21
    MAG_CALIB_M20 = 0x22
    MAG_CALIB_M21 = 0x23
    MAG_CALIB_M22 = 0x24
    ACCEL_CALIB_B0 = 0x25
    ACCEL_CALIB_B1 = 0x26
    ACCEL_CALIB_B2 = 0x27
    ACCEL_CALIB_M00 = 0x28
    ACCEL_CALIB_M01 = 0x29
    ACCEL_CALIB_M02 = 0x2a
    ACCEL_CALIB_M10 = 0x2b
    ACCEL_CALIB_M11 = 0x2c
    ACCEL_CALIB_M12 = 0x2d
    ACCEL_CALIB_M20 = 0x2e
    ACCEL_CALIB_M21 = 0x2f
    ACCEL_CALIB_M22 = 0x30
    CONTROLLER_TIMEOUT = 0x31
    LANE_LOOKAHEAD = 0x32
    LANE_INLIER_THRESHOLD = 0x33
    LANE_SLOPE_LIMIT = 0x34


param_type_dict = {
    LTParamType.RESERVED: LTParamValueType.LTPARAMS_TYPE_INT32,
    LTParamType.QCOMP_ALPHA: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.QCOMP_BETA: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.THETA_KP: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.THETA_KI: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.THETA_KD: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.THETA_KI_LIMIT: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ED_KP: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.PARKING_KP: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.PARKING_KI: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.PARKING_KD: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.PARKING_KI_LIMIT: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.FILLER4: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.BLINK_PERIOD: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.MADGWICK_BETA: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.IMU_FILTER_TYPE: LTParamValueType.LTPARAMS_TYPE_UINT32,
    LTParamType.MAHONY_KP: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.IMU_CALIBRATION_SAMPLES: LTParamValueType.LTPARAMS_TYPE_UINT32,
    LTParamType.IMU_PUBLISH_PERIOD: LTParamValueType.LTPARAMS_TYPE_UINT32,
    LTParamType.MOTOR_PWM_REFRESH_PERIOD: LTParamValueType.LTPARAMS_TYPE_UINT32,
    LTParamType.CONTROL_DEBUG_ENABLE: LTParamValueType.LTPARAMS_TYPE_UINT32,
    LTParamType.CONTROL_DEBUG_PUBLISH_PERIOD: LTParamValueType.LTPARAMS_TYPE_UINT32,
    LTParamType.IMU_RAW_ENABLE: LTParamValueType.LTPARAMS_TYPE_UINT32,
    LTParamType.MOTOR_OUTPUT_ENABLE: LTParamValueType.LTPARAMS_TYPE_UINT32,
    LTParamType.MOTOR_OUTPUT_PUBLISH_PERIOD: LTParamValueType.LTPARAMS_TYPE_UINT32,
    LTParamType.MAG_CALIB_B0: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.MAG_CALIB_B1: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.MAG_CALIB_B2: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.MAG_CALIB_M00: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.MAG_CALIB_M01: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.MAG_CALIB_M02: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.MAG_CALIB_M10: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.MAG_CALIB_M11: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.MAG_CALIB_M12: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.MAG_CALIB_M20: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.MAG_CALIB_M21: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.MAG_CALIB_M22: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ACCEL_CALIB_B0: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ACCEL_CALIB_B1: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ACCEL_CALIB_B2: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ACCEL_CALIB_M00: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ACCEL_CALIB_M01: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ACCEL_CALIB_M02: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ACCEL_CALIB_M10: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ACCEL_CALIB_M11: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ACCEL_CALIB_M12: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ACCEL_CALIB_M20: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ACCEL_CALIB_M21: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.ACCEL_CALIB_M22: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.CONTROLLER_TIMEOUT: LTParamValueType.LTPARAMS_TYPE_UINT32,
    LTParamType.LANE_LOOKAHEAD: LTParamValueType.LTPARAMS_TYPE_FLOAT,
    LTParamType.LANE_INLIER_THRESHOLD: LTParamValueType.LTPARAMS_TYPE_UINT32,
    LTParamType.LANE_SLOPE_LIMIT: LTParamValueType.LTPARAMS_TYPE_FLOAT,
}

param_default_dict = {
    LTParamType.RESERVED: 0,
    LTParamType.QCOMP_ALPHA: 0.05,
    LTParamType.QCOMP_BETA: 0.97,
    LTParamType.THETA_KP: 0.1,
    LTParamType.THETA_KI: 0.005,
    LTParamType.THETA_KD: 0.1,
    LTParamType.THETA_KI_LIMIT: 30.0,
    LTParamType.ED_KP: 1.0,
    LTParamType.PARKING_KP: 2.5,
    LTParamType.PARKING_KI: 1e-05,
    LTParamType.PARKING_KD: 5.0,
    LTParamType.PARKING_KI_LIMIT: 100.0,
    LTParamType.FILLER4: 0.0,
    LTParamType.BLINK_PERIOD: 0.0,
    LTParamType.MADGWICK_BETA: 5,
    LTParamType.IMU_FILTER_TYPE: 3,
    LTParamType.MAHONY_KP: 0.37,
    LTParamType.IMU_CALIBRATION_SAMPLES: 500,
    LTParamType.IMU_PUBLISH_PERIOD: 10000,
    LTParamType.MOTOR_PWM_REFRESH_PERIOD: 10000,
    LTParamType.CONTROL_DEBUG_ENABLE: 0,
    LTParamType.CONTROL_DEBUG_PUBLISH_PERIOD: 33333,
    LTParamType.IMU_RAW_ENABLE: 0,
    LTParamType.MOTOR_OUTPUT_ENABLE: 1,
    LTParamType.MOTOR_OUTPUT_PUBLISH_PERIOD: 33333,
    LTParamType.MAG_CALIB_B0: -157.98802123,
    LTParamType.MAG_CALIB_B1: 29.90416336,
    LTParamType.MAG_CALIB_B2: -170.7587217,
    LTParamType.MAG_CALIB_M00: 1.91202375,
    LTParamType.MAG_CALIB_M01: 0.00451775,
    LTParamType.MAG_CALIB_M02: -0.02390958,
    LTParamType.MAG_CALIB_M10: 0.00451775,
    LTParamType.MAG_CALIB_M11: 1.92767187,
    LTParamType.MAG_CALIB_M12: -0.03320508,
    LTParamType.MAG_CALIB_M20: -0.02390958,
    LTParamType.MAG_CALIB_M21: -0.03320508,
    LTParamType.MAG_CALIB_M22: 2.06441668,
    LTParamType.ACCEL_CALIB_B0: 0.00770027,
    LTParamType.ACCEL_CALIB_B1: -0.00205022,
    LTParamType.ACCEL_CALIB_B2: -0.10933483,
    LTParamType.ACCEL_CALIB_M00: 9.80959641,
    LTParamType.ACCEL_CALIB_M01: 0.0,
    LTParamType.ACCEL_CALIB_M02: 0.0,
    LTParamType.ACCEL_CALIB_M10: 0.0,
    LTParamType.ACCEL_CALIB_M11: 9.77402776,
    LTParamType.ACCEL_CALIB_M12: 0.0,
    LTParamType.ACCEL_CALIB_M20: 0.0,
    LTParamType.ACCEL_CALIB_M21: 0.0,
    LTParamType.ACCEL_CALIB_M22: 9.72531012,
    LTParamType.CONTROLLER_TIMEOUT: 1000000,
    LTParamType.LANE_LOOKAHEAD: -175.0,
    LTParamType.LANE_INLIER_THRESHOLD: 100,
    LTParamType.LANE_SLOPE_LIMIT: 0.5,
}


@dataclass
class LTParameters:
    reserved: int
    qcomp_alpha: float
    qcomp_beta: float
    theta_kp: float
    theta_ki: float
    theta_kd: float
    theta_ki_limit: float
    ed_kp: float
    parking_kp: float
    parking_ki: float
    parking_kd: float
    parking_ki_limit: float
    filler4: float
    blink_period: float
    madgwick_beta: float
    imu_filter_type: int
    mahony_kp: float
    imu_calibration_samples: int
    imu_publish_period: int
    motor_pwm_refresh_period: int
    control_debug_enable: int
    control_debug_publish_period: int
    imu_raw_enable: int
    motor_output_enable: int
    motor_output_publish_period: int
    mag_calib_b0: float
    mag_calib_b1: float
    mag_calib_b2: float
    mag_calib_m00: float
    mag_calib_m01: float
    mag_calib_m02: float
    mag_calib_m10: float
    mag_calib_m11: float
    mag_calib_m12: float
    mag_calib_m20: float
    mag_calib_m21: float
    mag_calib_m22: float
    accel_calib_b0: float
    accel_calib_b1: float
    accel_calib_b2: float
    accel_calib_m00: float
    accel_calib_m01: float
    accel_calib_m02: float
    accel_calib_m10: float
    accel_calib_m11: float
    accel_calib_m12: float
    accel_calib_m20: float
    accel_calib_m21: float
    accel_calib_m22: float
    controller_timeout: int
    lane_lookahead: float
    lane_inlier_threshold: int
    lane_slope_limit: float

    @classmethod
    def from_dict(cls, d):
        new_dict = {}
        for key, item in d.items():
            if key not in param_default_dict:
                raise ValueError(f"Unknown parameter {key}")
            key = key.name.lower()
            new_dict[key] = item
        return cls(**new_dict)

    @classmethod
    def from_default(cls):
        return cls.from_dict(param_default_dict)

    def to_dict(self):
        dct = asdict(self)
        new_dict = {}
        for key, item in dct.items():
            key = key.upper()
            new_dict[LTParamType[key]] = item
        return new_dict

    def __getitem__(self, key):
        key = key.name.lower()
        return getattr(self, key)

    def __setitem__(self, key, value):
        key = key.name.lower()
        setattr(self, key, value)