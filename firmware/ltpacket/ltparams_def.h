#define LTPARAMS_COUNT 0x3f
LTDEF(0x0, LTPARAMS_RESERVED, LTPARAMS_TYPE_INT32, i, 0)
LTDEF(0x1, LTPARAMS_QCOMP_ALPHA, LTPARAMS_TYPE_FLOAT, f, 0.05)
LTDEF(0x2, LTPARAMS_QCOMP_BETA, LTPARAMS_TYPE_FLOAT, f, 0.97)
LTDEF(0x3, LTPARAMS_THETA_KP, LTPARAMS_TYPE_FLOAT, f, 0.1)
LTDEF(0x4, LTPARAMS_THETA_KI, LTPARAMS_TYPE_FLOAT, f, 0.005)
LTDEF(0x5, LTPARAMS_THETA_KD, LTPARAMS_TYPE_FLOAT, f, 0.1)
LTDEF(0x6, LTPARAMS_THETA_KI_LIMIT, LTPARAMS_TYPE_FLOAT, f, 30.0)
LTDEF(0x7, LTPARAMS_ED_KP, LTPARAMS_TYPE_FLOAT, f, 1.0)
LTDEF(0x8, LTPARAMS_VISION_KP, LTPARAMS_TYPE_FLOAT, f, 1.0)
LTDEF(0x9, LTPARAMS_VISION_KI, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0xa, LTPARAMS_VISION_KD, LTPARAMS_TYPE_FLOAT, f, 3.0)
LTDEF(0xb, LTPARAMS_VISION_KI_LIMIT, LTPARAMS_TYPE_FLOAT, f, 100.0)
LTDEF(0xc, LTPARAMS_FILLER4, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0xd, LTPARAMS_BLINK_PERIOD, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0xe, LTPARAMS_MADGWICK_BETA, LTPARAMS_TYPE_FLOAT, f, 5)
LTDEF(0xf, LTPARAMS_IMU_FILTER_TYPE, LTPARAMS_TYPE_UINT32, ui, 3)
LTDEF(0x10, LTPARAMS_MAHONY_KP, LTPARAMS_TYPE_FLOAT, f, 0.37)
LTDEF(0x11, LTPARAMS_IMU_CALIBRATION_SAMPLES, LTPARAMS_TYPE_UINT32, ui, 500)
LTDEF(0x12, LTPARAMS_IMU_PUBLISH_PERIOD, LTPARAMS_TYPE_UINT32, ui, 10000)
LTDEF(0x13, LTPARAMS_MOTOR_PWM_REFRESH_PERIOD, LTPARAMS_TYPE_UINT32, ui, 10000)
LTDEF(0x14, LTPARAMS_CONTROL_DEBUG_ENABLE, LTPARAMS_TYPE_UINT32, ui, 0)
LTDEF(0x15, LTPARAMS_CONTROL_DEBUG_PUBLISH_PERIOD, LTPARAMS_TYPE_UINT32, ui, 33333)
LTDEF(0x16, LTPARAMS_IMU_RAW_ENABLE, LTPARAMS_TYPE_UINT32, ui, 0)
LTDEF(0x17, LTPARAMS_MOTOR_OUTPUT_ENABLE, LTPARAMS_TYPE_UINT32, ui, 1)
LTDEF(0x18, LTPARAMS_MOTOR_OUTPUT_PUBLISH_PERIOD, LTPARAMS_TYPE_UINT32, ui, 33333)
LTDEF(0x19, LTPARAMS_MAG_CALIB_B0, LTPARAMS_TYPE_FLOAT, f, -157.98802123)
LTDEF(0x1a, LTPARAMS_MAG_CALIB_B1, LTPARAMS_TYPE_FLOAT, f, 29.90416336)
LTDEF(0x1b, LTPARAMS_MAG_CALIB_B2, LTPARAMS_TYPE_FLOAT, f, -170.7587217)
LTDEF(0x1c, LTPARAMS_MAG_CALIB_M00, LTPARAMS_TYPE_FLOAT, f, 1.91202375)
LTDEF(0x1d, LTPARAMS_MAG_CALIB_M01, LTPARAMS_TYPE_FLOAT, f, 0.00451775)
LTDEF(0x1e, LTPARAMS_MAG_CALIB_M02, LTPARAMS_TYPE_FLOAT, f, -0.02390958)
LTDEF(0x1f, LTPARAMS_MAG_CALIB_M10, LTPARAMS_TYPE_FLOAT, f, 0.00451775)
LTDEF(0x20, LTPARAMS_MAG_CALIB_M11, LTPARAMS_TYPE_FLOAT, f, 1.92767187)
LTDEF(0x21, LTPARAMS_MAG_CALIB_M12, LTPARAMS_TYPE_FLOAT, f, -0.03320508)
LTDEF(0x22, LTPARAMS_MAG_CALIB_M20, LTPARAMS_TYPE_FLOAT, f, -0.02390958)
LTDEF(0x23, LTPARAMS_MAG_CALIB_M21, LTPARAMS_TYPE_FLOAT, f, -0.03320508)
LTDEF(0x24, LTPARAMS_MAG_CALIB_M22, LTPARAMS_TYPE_FLOAT, f, 2.06441668)
LTDEF(0x25, LTPARAMS_ACCEL_CALIB_B0, LTPARAMS_TYPE_FLOAT, f, 0.00770027)
LTDEF(0x26, LTPARAMS_ACCEL_CALIB_B1, LTPARAMS_TYPE_FLOAT, f, -0.00205022)
LTDEF(0x27, LTPARAMS_ACCEL_CALIB_B2, LTPARAMS_TYPE_FLOAT, f, -0.10933483)
LTDEF(0x28, LTPARAMS_ACCEL_CALIB_M00, LTPARAMS_TYPE_FLOAT, f, 9.80959641)
LTDEF(0x29, LTPARAMS_ACCEL_CALIB_M01, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0x2a, LTPARAMS_ACCEL_CALIB_M02, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0x2b, LTPARAMS_ACCEL_CALIB_M10, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0x2c, LTPARAMS_ACCEL_CALIB_M11, LTPARAMS_TYPE_FLOAT, f, 9.77402776)
LTDEF(0x2d, LTPARAMS_ACCEL_CALIB_M12, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0x2e, LTPARAMS_ACCEL_CALIB_M20, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0x2f, LTPARAMS_ACCEL_CALIB_M21, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0x30, LTPARAMS_ACCEL_CALIB_M22, LTPARAMS_TYPE_FLOAT, f, 9.72531012)
LTDEF(0x31, LTPARAMS_CONTROLLER_TIMEOUT, LTPARAMS_TYPE_UINT32, ui, 1000000)
LTDEF(0x32, LTPARAMS_LANE_LOOKAHEAD, LTPARAMS_TYPE_FLOAT, f, -100.0)
LTDEF(0x33, LTPARAMS_LANE_INLIER_THRESHOLD, LTPARAMS_TYPE_UINT32, ui, 100)
LTDEF(0x34, LTPARAMS_LANE_SLOPE_LIMIT, LTPARAMS_TYPE_FLOAT, f, 0.5)
LTDEF(0x35, LTPARAMS_PLANNER_GOAL_AREA_THRESHOLD, LTPARAMS_TYPE_FLOAT, f, 0.005)
LTDEF(0x36, LTPARAMS_PLANNER_MEASUREMENT_COUNT, LTPARAMS_TYPE_UINT32, ui, 50)
LTDEF(0x37, LTPARAMS_PARKING_ESTIMATOR_XY_LIMIT, LTPARAMS_TYPE_FLOAT, f, 1.0)
LTDEF(0x38, LTPARAMS_PARKING_ESTIMATOR_APPROACH_D, LTPARAMS_TYPE_FLOAT, f, 0.8)
LTDEF(0x39, LTPARAMS_PARKING_ESTIMATOR_ALIGNMENT_D, LTPARAMS_TYPE_FLOAT, f, 2.0)
LTDEF(0x3a, LTPARAMS_DUBINS_LIMIT_ANGLE, LTPARAMS_TYPE_FLOAT, f, 60)
LTDEF(0x3b, LTPARAMS_DUBINS_RADIUS, LTPARAMS_TYPE_FLOAT, f, 0.2)
LTDEF(0x3c, LTPARAMS_DUBINS_SAFETY_MARGIN, LTPARAMS_TYPE_FLOAT, f, 0.6)
LTDEF(0x3d, LTPARAMS_VISION_CONTROL_MAX_CONTROL, LTPARAMS_TYPE_FLOAT, f, 0.7)
LTDEF(0x3e, LTPARAMS_PARKING_VISIBILITY_LIMIT, LTPARAMS_TYPE_FLOAT, f, 0.83)