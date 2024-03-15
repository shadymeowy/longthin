#define LTPARAMS_COUNT 0x19
LTDEF(0x0, LTPARAMS_RESERVED, LTPARAMS_TYPE_INT32, i, 0)
LTDEF(0x1, LTPARAMS_QCOMP_ALPHA, LTPARAMS_TYPE_FLOAT, f, 0.05)
LTDEF(0x2, LTPARAMS_QCOMP_BETA, LTPARAMS_TYPE_FLOAT, f, 0.97)
LTDEF(0x3, LTPARAMS_THETA_KP, LTPARAMS_TYPE_FLOAT, f, 0.05)
LTDEF(0x4, LTPARAMS_THETA_KI, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0x5, LTPARAMS_ED_KP, LTPARAMS_TYPE_FLOAT, f, 0.8)
LTDEF(0x6, LTPARAMS_ED_KI, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0x7, LTPARAMS_VDESIRED_KP, LTPARAMS_TYPE_FLOAT, f, 1.0)
LTDEF(0x8, LTPARAMS_VDESIRED_KI, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0x9, LTPARAMS_WDESIRED_KP, LTPARAMS_TYPE_FLOAT, f, 1.0)
LTDEF(0xa, LTPARAMS_WDESIRED_KI, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0xb, LTPARAMS_WHEEL_RADIUS, LTPARAMS_TYPE_FLOAT, f, 1.0)
LTDEF(0xc, LTPARAMS_WHEEL_DISTANCE, LTPARAMS_TYPE_FLOAT, f, 1.0)
LTDEF(0xd, LTPARAMS_BLINK_PERIOD, LTPARAMS_TYPE_FLOAT, f, 0.0)
LTDEF(0xe, LTPARAMS_MADGWICK_BETA, LTPARAMS_TYPE_FLOAT, f, 5)
LTDEF(0xf, LTPARAMS_IMU_FILTER_TYPE, LTPARAMS_TYPE_UINT32, ui, 2)
LTDEF(0x10, LTPARAMS_MAHONY_KP, LTPARAMS_TYPE_FLOAT, f, 1.0)
LTDEF(0x11, LTPARAMS_IMU_CALIBRATION_SAMPLES, LTPARAMS_TYPE_UINT32, ui, 500)
LTDEF(0x12, LTPARAMS_IMU_PUBLISH_PERIOD, LTPARAMS_TYPE_UINT32, ui, 33333)
LTDEF(0x13, LTPARAMS_MOTOR_PWM_REFRESH_PERIOD, LTPARAMS_TYPE_UINT32, ui, 10000)
LTDEF(0x14, LTPARAMS_CONTROL_DEBUG_ENABLE, LTPARAMS_TYPE_UINT32, ui, 0)
LTDEF(0x15, LTPARAMS_CONTROL_DEBUG_PUBLISH_PERIOD, LTPARAMS_TYPE_UINT32, ui, 33333)
LTDEF(0x16, LTPARAMS_IMU_RAW_ENABLE, LTPARAMS_TYPE_UINT32, ui, 0)
LTDEF(0x17, LTPARAMS_MOTOR_OUTPUT_ENABLE, LTPARAMS_TYPE_UINT32, ui, 1)
LTDEF(0x18, LTPARAMS_MOTOR_OUTPUT_PUBLISH_PERIOD, LTPARAMS_TYPE_UINT32, ui, 33333)