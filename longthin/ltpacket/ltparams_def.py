from enum import Enum


LTPARAMS_COUNT = 0x31


class LTParamType(Enum):
    LTPARAMS_TYPE_FLOAT = 0
    LTPARAMS_TYPE_UINT32 = 1
    LTPARAMS_TYPE_INT32 = 2


class LTParams(Enum):
    RESERVED = 0x0
    QCOMP_ALPHA = 0x1
    QCOMP_BETA = 0x2
    THETA_KP = 0x3
    THETA_KI = 0x4
    ED_KP = 0x5
    ED_KI = 0x6
    VDESIRED_KP = 0x7
    VDESIRED_KI = 0x8
    WDESIRED_KP = 0x9
    WDESIRED_KI = 0xa
    WHEEL_RADIUS = 0xb
    WHEEL_DISTANCE = 0xc
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


param_type_dict = {
    LTParams.RESERVED: LTParamType.LTPARAMS_TYPE_INT32,
    LTParams.QCOMP_ALPHA: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.QCOMP_BETA: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.THETA_KP: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.THETA_KI: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ED_KP: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ED_KI: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.VDESIRED_KP: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.VDESIRED_KI: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.WDESIRED_KP: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.WDESIRED_KI: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.WHEEL_RADIUS: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.WHEEL_DISTANCE: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.BLINK_PERIOD: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.MADGWICK_BETA: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.IMU_FILTER_TYPE: LTParamType.LTPARAMS_TYPE_UINT32,
    LTParams.MAHONY_KP: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.IMU_CALIBRATION_SAMPLES: LTParamType.LTPARAMS_TYPE_UINT32,
    LTParams.IMU_PUBLISH_PERIOD: LTParamType.LTPARAMS_TYPE_UINT32,
    LTParams.MOTOR_PWM_REFRESH_PERIOD: LTParamType.LTPARAMS_TYPE_UINT32,
    LTParams.CONTROL_DEBUG_ENABLE: LTParamType.LTPARAMS_TYPE_UINT32,
    LTParams.CONTROL_DEBUG_PUBLISH_PERIOD: LTParamType.LTPARAMS_TYPE_UINT32,
    LTParams.IMU_RAW_ENABLE: LTParamType.LTPARAMS_TYPE_UINT32,
    LTParams.MOTOR_OUTPUT_ENABLE: LTParamType.LTPARAMS_TYPE_UINT32,
    LTParams.MOTOR_OUTPUT_PUBLISH_PERIOD: LTParamType.LTPARAMS_TYPE_UINT32,
    LTParams.MAG_CALIB_B0: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.MAG_CALIB_B1: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.MAG_CALIB_B2: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.MAG_CALIB_M00: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.MAG_CALIB_M01: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.MAG_CALIB_M02: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.MAG_CALIB_M10: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.MAG_CALIB_M11: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.MAG_CALIB_M12: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.MAG_CALIB_M20: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.MAG_CALIB_M21: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.MAG_CALIB_M22: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ACCEL_CALIB_B0: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ACCEL_CALIB_B1: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ACCEL_CALIB_B2: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ACCEL_CALIB_M00: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ACCEL_CALIB_M01: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ACCEL_CALIB_M02: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ACCEL_CALIB_M10: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ACCEL_CALIB_M11: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ACCEL_CALIB_M12: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ACCEL_CALIB_M20: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ACCEL_CALIB_M21: LTParamType.LTPARAMS_TYPE_FLOAT,
    LTParams.ACCEL_CALIB_M22: LTParamType.LTPARAMS_TYPE_FLOAT,
}

param_default_dict = {
    LTParams.RESERVED: 0,
    LTParams.QCOMP_ALPHA: 0.05,
    LTParams.QCOMP_BETA: 0.97,
    LTParams.THETA_KP: 0.05,
    LTParams.THETA_KI: 0.0,
    LTParams.ED_KP: 0.8,
    LTParams.ED_KI: 0.0,
    LTParams.VDESIRED_KP: 1.0,
    LTParams.VDESIRED_KI: 0.0,
    LTParams.WDESIRED_KP: 1.0,
    LTParams.WDESIRED_KI: 0.0,
    LTParams.WHEEL_RADIUS: 1.0,
    LTParams.WHEEL_DISTANCE: 1.0,
    LTParams.BLINK_PERIOD: 0.0,
    LTParams.MADGWICK_BETA: 5,
    LTParams.IMU_FILTER_TYPE: 2,
    LTParams.MAHONY_KP: 1.0,
    LTParams.IMU_CALIBRATION_SAMPLES: 500,
    LTParams.IMU_PUBLISH_PERIOD: 33333,
    LTParams.MOTOR_PWM_REFRESH_PERIOD: 10000,
    LTParams.CONTROL_DEBUG_ENABLE: 0,
    LTParams.CONTROL_DEBUG_PUBLISH_PERIOD: 33333,
    LTParams.IMU_RAW_ENABLE: 0,
    LTParams.MOTOR_OUTPUT_ENABLE: 1,
    LTParams.MOTOR_OUTPUT_PUBLISH_PERIOD: 33333,
    LTParams.MAG_CALIB_B0: -157.98802123,
    LTParams.MAG_CALIB_B1: 29.90416336,
    LTParams.MAG_CALIB_B2: -170.7587217,
    LTParams.MAG_CALIB_M00: 1.91202375,
    LTParams.MAG_CALIB_M01: 0.00451775,
    LTParams.MAG_CALIB_M02: -0.02390958,
    LTParams.MAG_CALIB_M10: 0.00451775,
    LTParams.MAG_CALIB_M11: 1.92767187,
    LTParams.MAG_CALIB_M12: -0.03320508,
    LTParams.MAG_CALIB_M20: -0.02390958,
    LTParams.MAG_CALIB_M21: -0.03320508,
    LTParams.MAG_CALIB_M22: 2.06441668,
    LTParams.ACCEL_CALIB_B0: 0.00770027,
    LTParams.ACCEL_CALIB_B1: -0.00205022,
    LTParams.ACCEL_CALIB_B2: -0.10933483,
    LTParams.ACCEL_CALIB_M00: 9.80959641,
    LTParams.ACCEL_CALIB_M01: 0.0,
    LTParams.ACCEL_CALIB_M02: 0.0,
    LTParams.ACCEL_CALIB_M10: 0.0,
    LTParams.ACCEL_CALIB_M11: 9.77402776,
    LTParams.ACCEL_CALIB_M12: 0.0,
    LTParams.ACCEL_CALIB_M20: 0.0,
    LTParams.ACCEL_CALIB_M21: 0.0,
    LTParams.ACCEL_CALIB_M22: 9.72531012,
}