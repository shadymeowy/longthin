#ifndef LTPACKET_H
#define LTPACKET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

enum ltpacket_type_t {
	LTPACKET_TYPE_RESERVED = 0x00, // reserved
	LTPACKET_TYPE_IMU = 0x01, // filtered imu data
	LTPACKET_TYPE_IMU_RAW = 0x02, // raw imu data
	LTPACKET_TYPE_ADC = 0x03, // scaled adc data as voltage
	LTPACKET_TYPE_ADC_RAW = 0x04, // raw adc data
	LTPACKET_TYPE_VISO = 0x05, // visual odometry data
	LTPACKET_TYPE_MOTOR = 0x06, // scaled motor data to -1..1
	LTPACKET_TYPE_MOTOR_RAW = 0x07, // raw motor data as pwm
	LTPACKET_TYPE_SETPOINT = 0x08, // setpoint data
	LTPACKET_TYPE_EVISO = 0x09, // external visual odometry data
	LTPACKET_TYPE_SETPARAM = 0x0a, // set parameter
	LTPACKET_TYPE_LED = 0x0b, // led control
	LTPACKET_TYPE_CONTROL_DEBUG = 0x0c, // control debug data
};

struct ltpacket_reserved_t {};

struct ltpacket_imu_t {
	float roll;
	float pitch;
	float yaw;
	float vel;
};

struct ltpacket_imu_raw_t {
	float accel[3];
	float gyro[3];
	float mag[3];
};

struct ltpacket_adc_t {
	float adc[4];
};

struct ltpacket_adc_raw_t {
	uint16_t adc[4];
};

struct ltpacket_viso_t {
	float x;
	float y;
	float z;
	float yaw;
};

struct ltpacket_motor_t {
	float motor_left;
	float motor_right;
};

struct ltpacket_motor_raw_t {
	int16_t motor_left;
	int16_t motor_right;
};

struct ltpacket_setpoint_t {
	float vel;
	float yaw;
};

struct ltpacket_eviso_t {
	float x;
	float y;
	float z;
	float yaw;
};

struct ltpacket_setparam_t {
	int32_t param;
	float value;
};

struct ltpacket_led_t {
	uint8_t led;
	uint8_t state;
};

struct ltpacket_reader_t {
	uint8_t *buffer;
	uint16_t buffer_length;
	uint16_t buffer_index;
	uint16_t packet_length;
};

struct ltpacket_control_debug_t {
	float current_d;
	float current_yaw;
	float desired_d;
	float desired_yaw;
	float current_vel;
	float current_w;
	float desired_v;
	float desired_w;
	float u_v;
	float u_w;
	float u_r;
	float u_l;
};

struct ltpacket_t {
	enum ltpacket_type_t type;
	union {
		struct ltpacket_reserved_t reserved;
		struct ltpacket_imu_t imu;
		struct ltpacket_imu_raw_t imu_raw;
		struct ltpacket_adc_t adc;
		struct ltpacket_adc_raw_t adc_raw;
		struct ltpacket_viso_t viso;
		struct ltpacket_motor_t motor;
		struct ltpacket_motor_raw_t motor_raw;
		struct ltpacket_setpoint_t setpoint;
		struct ltpacket_eviso_t eviso;
		struct ltpacket_setparam_t setparam;
		struct ltpacket_control_debug_t control_debug;
		struct ltpacket_led_t led;
		
	};
};

int ltpacket_read_buffer(struct ltpacket_t *packet, uint8_t *buffer, uint16_t buffer_length);
int ltpacket_write_buffer(struct ltpacket_t *packet, uint8_t *buffer, uint16_t buffer_length);
int ltpacket_size(struct ltpacket_t *packet);

#ifdef __cplusplus
}
#endif

#endif