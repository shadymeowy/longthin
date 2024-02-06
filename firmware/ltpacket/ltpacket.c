#include "ltpacket.h"
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define _READCASE(typ, strt_t, prop)                               \
	case typ:                                                  \
		if (buffer_length < sizeof(struct strt_t)) {              \
			return -1;                                 \
		}                                                  \
		memcpy(&packet->prop, buffer + 1, sizeof(struct strt_t)); \
		return 1 + sizeof(struct strt_t);

int ltpacket_read_buffer(struct ltpacket_t *packet, uint8_t *buffer, uint16_t buffer_length)
{
	if (buffer_length < 1) {
		return -1;
	}
	packet->type = (enum ltpacket_type_t)buffer[0];
	switch (packet->type) {
	case LTPACKET_TYPE_RESERVED:
		return -1;
		_READCASE(LTPACKET_TYPE_IMU, ltpacket_imu_t, imu);
		_READCASE(LTPACKET_TYPE_IMU_RAW, ltpacket_imu_raw_t, imu_raw);
		_READCASE(LTPACKET_TYPE_ADC, ltpacket_adc_t, adc);
		_READCASE(LTPACKET_TYPE_ADC_RAW, ltpacket_adc_raw_t, adc_raw);
		_READCASE(LTPACKET_TYPE_VISO, ltpacket_viso_t, viso);
		_READCASE(LTPACKET_TYPE_MOTOR, ltpacket_motor_t, motor);
		_READCASE(LTPACKET_TYPE_MOTOR_RAW, ltpacket_motor_raw_t, motor_raw);
		_READCASE(LTPACKET_TYPE_SETPOINT, ltpacket_setpoint_t, setpoint);
		_READCASE(LTPACKET_TYPE_EVISO, ltpacket_eviso_t, eviso);
		_READCASE(LTPACKET_TYPE_SETPARAM, ltpacket_setparam_t, setparam);
		_READCASE(LTPACKET_TYPE_LED, ltpacket_led_t, led);
		_READCASE(LTPACKET_TYPE_CONTROL_DEBUG, ltpacket_control_debug_t, control_debug);
	default:
		return -1;
	}
}

#define _WRITECASE(typ, strt_t, prop)                              \
	case typ:                                                  \
		if (buffer_length < sizeof(struct strt_t)) {              \
			return -1;                                 \
		}                                                  \
		memcpy(buffer + 1, &packet->prop, sizeof(struct strt_t)); \
		return 1 + sizeof(struct strt_t);

int ltpacket_write_buffer(struct ltpacket_t *packet, uint8_t *buffer, uint16_t buffer_length)
{
	if (buffer_length < 1) {
		return -1;
	}
	buffer[0] = packet->type;
	switch (packet->type) {
	case LTPACKET_TYPE_RESERVED:
		return -1;
		_WRITECASE(LTPACKET_TYPE_IMU, ltpacket_imu_t, imu);
		_WRITECASE(LTPACKET_TYPE_IMU_RAW, ltpacket_imu_raw_t, imu_raw);
		_WRITECASE(LTPACKET_TYPE_ADC, ltpacket_adc_t, adc);
		_WRITECASE(LTPACKET_TYPE_ADC_RAW, ltpacket_adc_raw_t, adc_raw);
		_WRITECASE(LTPACKET_TYPE_VISO, ltpacket_viso_t, viso);
		_WRITECASE(LTPACKET_TYPE_MOTOR, ltpacket_motor_t, motor);
		_WRITECASE(LTPACKET_TYPE_MOTOR_RAW, ltpacket_motor_raw_t, motor_raw);
		_WRITECASE(LTPACKET_TYPE_SETPOINT, ltpacket_setpoint_t, setpoint);
		_WRITECASE(LTPACKET_TYPE_EVISO, ltpacket_eviso_t, eviso);
		_WRITECASE(LTPACKET_TYPE_SETPARAM, ltpacket_setparam_t, setparam);
		_WRITECASE(LTPACKET_TYPE_LED, ltpacket_led_t, led);
		_WRITECASE(LTPACKET_TYPE_CONTROL_DEBUG, ltpacket_control_debug_t, control_debug);
	default:
		return -1;
	}
}

#ifdef __cplusplus
}
#endif