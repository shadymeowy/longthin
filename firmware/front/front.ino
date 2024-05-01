#include <Arduino.h>
#include <Wire.h>

#include "mpu6050.h"
#include "hmc5883l.h"
#include "packet.h"
#include "ltpacket.h"
#include "ltparams.h"

#include "qcomp.h"
#include "madgwick.h"
#include "madgwick_ahrs.h"
#include "mahony_ahrs.h"

bool blink = false;

uint8_t buffer[100];
struct packet_reader_t reader;
uint8_t buffer1[100];
struct packet_reader_t reader1;
uint8_t buffer2[100];
struct packet_reader_t reader2;

int serial_write(const uint8_t *buffer, uint16_t size)
{
	Serial.write(buffer, size);
	Serial1.write(buffer, size);
	Serial2.write(buffer, size);
	return 0;
}

void listen_init()
{
	packet_reader_init(&reader, buffer, sizeof(buffer));
	packet_reader_init(&reader1, buffer1, sizeof(buffer1));
	packet_reader_init(&reader2, buffer2, sizeof(buffer2));
}

void listen_process()
{
	int ret, c;
	struct ltpacket_t packet;
	uint8_t *data;

	while (Serial.available()) {
		c = Serial.read();
		if ((ret = packet_reader_push(&reader, c)) >= 0) {
			data = packet_reader_head(&reader);
			ltpacket_read_buffer(&packet, data, ret);
			listen_handle(&packet);
			Serial1.write(data - 3, ret + 5);
			Serial2.write(data - 3, ret + 5);
		}
	}
	while (Serial1.available()) {
		c = Serial1.read();
		if ((ret = packet_reader_push(&reader1, c)) >= 0) {
			data = packet_reader_head(&reader1);
			ltpacket_read_buffer(&packet, data, ret);
			listen_handle(&packet);
			Serial.write(data - 3, ret + 5);
			Serial2.write(data - 3, ret + 5);
		}
	}
	while (Serial2.available()) {
		c = Serial2.read();
		if ((ret = packet_reader_push(&reader2, c)) >= 0) {
			data = packet_reader_head(&reader2);
			ltpacket_read_buffer(&packet, data, ret);
			listen_handle(&packet);
			Serial.write(data - 3, ret + 5);
			Serial1.write(data - 3, ret + 5);
		}
	}
}

void listen_handle(struct ltpacket_t *packet)
{
	enum ltparams_index_t index;
	switch (packet->type) {
	case LTPACKET_TYPE_LED:
	case LTPACKET_TYPE_LED_CONTROL:
		led_handle(packet);
		break;
	case LTPACKET_TYPE_SETPARAM:
		index = (enum ltparams_index_t)packet->setparam.param;
		ltparams_set(index, packet->setparam.value);
		break;
	case LTPACKET_TYPE_SETPARAMU:
		index = (enum ltparams_index_t)packet->setparamu.param;
		ltparams_setu(index, packet->setparamu.value);
		break;
	case LTPACKET_TYPE_SETPARAMI:
		index = (enum ltparams_index_t)packet->setparami.param;
		ltparams_seti(index, packet->setparami.value);
		break;
	case LTPACKET_TYPE_REBOOT:
		rp2040.reboot();
		break;
	default:
		break;
	}
}

float imu_meas[9] = { 0, 0, 1, 0, 0, 0, 1, 1, 0 };
struct mpu6050 imu = { 0 };
struct hmc5883l mag = { 0 };
float dt = 0;
quat_t q_est = { 1, 0, 0, 0 };
vec3_t delta_vel = { 0, 0, 0 };
float delta_vel_dt = 0;
bool filter_init_done = false;

void imu_init()
{
	mpu6050_init(&imu, 0x68);
	uint32_t samples = ltparams_getu(LTPARAMS_IMU_CALIBRATION_SAMPLES);
	mpu6050_calibrate(&imu, samples);
	hmc5883l_init(&mag, 0x1E);
	imu_write_calibration();
}

void imu_write_calibration()
{
	mag.bias[0] = ltparams_get(LTPARAMS_MAG_CALIB_B0);
	mag.bias[1] = ltparams_get(LTPARAMS_MAG_CALIB_B1);
	mag.bias[2] = ltparams_get(LTPARAMS_MAG_CALIB_B2);
	mag.mtx[0] = ltparams_get(LTPARAMS_MAG_CALIB_M00);
	mag.mtx[1] = ltparams_get(LTPARAMS_MAG_CALIB_M01);
	mag.mtx[2] = ltparams_get(LTPARAMS_MAG_CALIB_M02);
	mag.mtx[3] = ltparams_get(LTPARAMS_MAG_CALIB_M10);
	mag.mtx[4] = ltparams_get(LTPARAMS_MAG_CALIB_M11);
	mag.mtx[5] = ltparams_get(LTPARAMS_MAG_CALIB_M12);
	mag.mtx[6] = ltparams_get(LTPARAMS_MAG_CALIB_M20);
	mag.mtx[7] = ltparams_get(LTPARAMS_MAG_CALIB_M21);
	mag.mtx[8] = ltparams_get(LTPARAMS_MAG_CALIB_M22);
	imu.bias_accel[0] = ltparams_get(LTPARAMS_ACCEL_CALIB_B0);
	imu.bias_accel[1] = ltparams_get(LTPARAMS_ACCEL_CALIB_B1);
	imu.bias_accel[2] = ltparams_get(LTPARAMS_ACCEL_CALIB_B2);
	imu.mtx_accel[0] = ltparams_get(LTPARAMS_ACCEL_CALIB_M00);
	imu.mtx_accel[1] = ltparams_get(LTPARAMS_ACCEL_CALIB_M01);
	imu.mtx_accel[2] = ltparams_get(LTPARAMS_ACCEL_CALIB_M02);
	imu.mtx_accel[3] = ltparams_get(LTPARAMS_ACCEL_CALIB_M10);
	imu.mtx_accel[4] = ltparams_get(LTPARAMS_ACCEL_CALIB_M11);
	imu.mtx_accel[5] = ltparams_get(LTPARAMS_ACCEL_CALIB_M12);
	imu.mtx_accel[6] = ltparams_get(LTPARAMS_ACCEL_CALIB_M20);
	imu.mtx_accel[7] = ltparams_get(LTPARAMS_ACCEL_CALIB_M21);
	imu.mtx_accel[8] = ltparams_get(LTPARAMS_ACCEL_CALIB_M22);
}

void imu_read()
{
	mpu6050_read(&imu);
	for (int i = 0; i < 3; i++) {
		imu_meas[i] = imu.accel[i];
		imu_meas[i + 3] = imu.gyro[i];
	}
	rp2040.fifo.push_nb(0);
	hmc5883l_read(&mag);
	for (int i = 0; i < 3; i++) {
		imu_meas[i + 6] = mag.mag[i];
	}
	rp2040.fifo.push_nb(0);
}

void imu_filter()
{
	static uint32_t last_time = 0;
	uint32_t dummy;
	if (!rp2040.fifo.pop_nb(&dummy)) {
		return;
	}
	bool need_reset = false;
	for (int i = 0; i < 4; i++) {
		if (isnan(q_est[i]) || isinf(q_est[i])) {
			need_reset = true;
			break;
		}
	}
	if (!filter_init_done || need_reset) {
		filter_init_done = true;
		qcomp_init(&imu_meas[0], &imu_meas[6], q_est);
		last_time = micros();
		return;
	}
	uint32_t now = micros();
	dt = (now - last_time) / 1e6;
	last_time = now;

	float qcomp_alpha = ltparams_get(LTPARAMS_QCOMP_ALPHA);
	float qcomp_beta = ltparams_get(LTPARAMS_QCOMP_BETA);
	float madgwick_beta = ltparams_get(LTPARAMS_MADGWICK_BETA);
	float mahony_kp = ltparams_get(LTPARAMS_MAHONY_KP);

	struct quaternion q_;
	switch (ltparams_getu(LTPARAMS_IMU_FILTER_TYPE)) {
	case 0: // QCOMP
		qcomp_update(q_est, &imu_meas[0], &imu_meas[3], &imu_meas[6], dt, qcomp_alpha, qcomp_beta);
		break;
	case 1: // Madgwick
		q_.w = q_est[0];
		q_.x = q_est[1];
		q_.y = q_est[2];
		q_.z = q_est[3];
		madgwick_filter(&q_, &imu_meas[0], &imu_meas[3], &imu_meas[6], dt);
		q_est[0] = q_.w;
		q_est[1] = q_.x;
		q_est[2] = q_.y;
		q_est[3] = q_.z;
		break;
	case 2: // Madgwick AHRS
		madgwick_ahrs_update(q_est, &imu_meas[3], &imu_meas[0], &imu_meas[6], dt, madgwick_beta);
		break;
	case 3: // Mahony AHRS
		mahony_ahrs_update(q_est, &imu_meas[3], &imu_meas[0], &imu_meas[6], dt, mahony_kp);
		break;
	default:
		break;
	}

	vec3_t accel = { imu_meas[0], imu_meas[1], imu_meas[2] };
	vec3_t accel_earth;
	rotate_with_quat(q_est, accel, accel_earth);
	for (int i = 0; i < 3; i++) {
		delta_vel[i] += accel_earth[i] * dt;
	}
	delta_vel_dt += dt;
}

void imu_publish()
{
	static uint32_t last_time = 0;
	uint32_t now = micros();
	uint32_t period = ltparams_getu(LTPARAMS_IMU_PUBLISH_PERIOD);
	if (now - last_time < period) {
		return;
	}
	last_time = now;

	float euler[3] = { 0 };
	euler_from_quat(q_est, euler);

	// TODO a better way to check if the filter is working
	for (int i = 0; i < 3; i++) {
		if (isnan(euler[i]) || isinf(euler[i])) {
			filter_init_done = false;
			return;
		}
	}

	struct ltpacket_t packet;
	packet.type = LTPACKET_TYPE_IMU;
	packet.imu.qw = q_est[0];
	packet.imu.qx = q_est[1];
	packet.imu.qy = q_est[2];
	packet.imu.qz = q_est[3];
	packet.imu.dvx = delta_vel[0];
	packet.imu.dvy = delta_vel[1];
	packet.imu.dvz = delta_vel[2];
	packet.imu.dt = delta_vel_dt;
	delta_vel[0] = 0;
	delta_vel[1] = 0;
	delta_vel[2] = 0;
	delta_vel_dt = 0;
	ltpacket_send(&packet, serial_write);
}

void imu_publish_raw()
{
	if (!ltparams_getu(LTPARAMS_IMU_RAW_ENABLE)) {
		return;
	}

	static uint32_t last_time = 0;
	uint32_t now = micros();
	uint32_t period = ltparams_getu(LTPARAMS_IMU_PUBLISH_PERIOD);
	if (now - last_time < period) {
		return;
	}
	last_time = now;

	struct ltpacket_t packet;
	packet.type = LTPACKET_TYPE_IMU_RAW;
	packet.imu_raw.accel_x = imu.accel_raw[0];
	packet.imu_raw.accel_y = imu.accel_raw[1];
	packet.imu_raw.accel_z = imu.accel_raw[2];
	packet.imu_raw.gyro_x = imu.gyro_raw[0];
	packet.imu_raw.gyro_y = imu.gyro_raw[1];
	packet.imu_raw.gyro_z = imu.gyro_raw[2];
	packet.imu_raw.mag_x = mag.mag_raw[0];
	packet.imu_raw.mag_y = mag.mag_raw[1];
	packet.imu_raw.mag_z = mag.mag_raw[2];
	ltpacket_send(&packet, serial_write);
}

struct led_t {
	// Internal part
	uint8_t pin;
	uint8_t inversion;
	uint8_t current_state;
	uint32_t last_change_time;
	uint32_t tone;
	// Configurable part
	uint8_t default_state;
	uint32_t high_time;
	uint32_t low_time;
	int32_t remaining_cycles;
};

struct led_t leds[] = {
	{
		.pin = LED_BUILTIN,
		.inversion = 0,
		.current_state = 0,
		.last_change_time = 0,
		.tone = 0,
		.default_state = 0,
		.high_time = 0,
		.low_time = 0,
		.remaining_cycles = 0,
	},
	{
		.pin = 10,
		.inversion = 1,
		.current_state = 0,
		.last_change_time = 0,
		.tone = 0,
		.default_state = 0,
		.high_time = 0,
		.low_time = 0,
		.remaining_cycles = 0,
	},
	{
		.pin = 11,
		.inversion = 1,
		.current_state = 0,
		.last_change_time = 0,
		.tone = 0,
		.default_state = 0,
		.high_time = 0,
		.low_time = 0,
		.remaining_cycles = 0,
	},
	{
		.pin = 13,
		.inversion = 1,
		.current_state = 0,
		.last_change_time = 0,
		.tone = 0,
		.default_state = 0,
		.high_time = 0,
		.low_time = 0,
		.remaining_cycles = 0,
	},
	{
		.pin = 14,
		.inversion = 0,
		.current_state = 0,
		.last_change_time = 0,
		.tone = 0,
		.default_state = 0,
		.high_time = 0,
		.low_time = 0,
		.remaining_cycles = 0,
	},
};

void led_init()
{
	for (int i = 0; i < sizeof(leds) / sizeof(leds[0]); i++) {
		pinMode(leds[i].pin, OUTPUT);
		leds[i].current_state = leds[i].default_state;
		led_write(i, leds[i].default_state);
		leds[i].last_change_time = millis();
	}
}

void led_handle(struct ltpacket_t *packet)
{
	if (packet->type == LTPACKET_TYPE_LED_CONTROL) {
		uint8_t id = packet->led_control.id;
		if (id >= sizeof(leds) / sizeof(leds[0])) {
			return;
		}
		led_t *led = &leds[id];
		bool resetting = led->remaining_cycles == 0;
		led->default_state = packet->led_control.default_state;
		led->high_time = packet->led_control.high_time;
		led->low_time = packet->led_control.low_time;
		led->remaining_cycles = packet->led_control.remaining_cycles;
		if (resetting && led->remaining_cycles != 0) {
			led->current_state = !led->default_state;
			led->last_change_time = millis();
			led_write(id, led->current_state);
		}
	} else if (packet->type == LTPACKET_TYPE_LED) {
		uint8_t id = packet->led.index;
		if (id >= sizeof(leds) / sizeof(leds[0])) {
			return;
		}
		led_t *led = &leds[id];
		led->default_state = packet->led.state;
		led->high_time = 0;
		led->low_time = 0;
		led->remaining_cycles = 0;
	}
}

void led_write(uint8_t id, uint8_t state)
{
	if (id >= sizeof(leds) / sizeof(leds[0])) {
		return;
	}
	if (leds[id].tone) {
		tone(leds[id].pin, (state ^ leds[id].inversion) ? leds[id].tone : 0);
	} else {
		digitalWrite(leds[id].pin, state ^ leds[id].inversion);
	}
}

void led_process()
{
	uint32_t now = millis();
	for (int i = 0; i < sizeof(leds) / sizeof(leds[0]); i++) {
		uint32_t last = leds[i].last_change_time;
		uint32_t delta = now - last;
		if (leds[i].remaining_cycles == 0) {
			led_write(i, leds[i].default_state);
			leds[i].current_state = leds[i].default_state;
			leds[i].last_change_time = now;
		} else if (leds[i].remaining_cycles != 0) {
			bool change = false;
			if (leds[i].current_state) {
				if (delta > leds[i].high_time) {
					leds[i].current_state = 0;
					change = true;
				}
			} else {
				if (delta > leds[i].low_time) {
					leds[i].current_state = 1;
					change = true;
				}
			}
			if (change) {
				led_write(i, leds[i].current_state);
				leds[i].last_change_time = now;
				if (leds[i].current_state == leds[i].default_state && leds[i].remaining_cycles > 0) {
					leds[i].remaining_cycles--;
				}
			}
		}
	}
}

void setup()
{
	Wire1.setSDA(18);
	Wire1.setSCL(19);
	Serial1.setTX(16);
	Serial1.setRX(17);
	Serial2.setTX(4);
	Serial2.setRX(5);
	Serial.begin(115200);
	Serial1.begin(115200);
	Serial2.begin(115200);

	led_init();
	listen_init();
}

void loop()
{
	led_process();
	imu_filter();
	imu_publish();
	imu_publish_raw();
	listen_process();
}

void setup1()
{
	imu_init();
}

void loop1()
{
	imu_read();
}