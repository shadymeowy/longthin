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
		blink = packet->led.state;
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

float imu_raw[9] = { 0, 0, 1, 0, 0, 0, 1, 1, 0 };
struct mpu6050 imu = { 0 };
struct hmc5883l mag = { 0 };
float dt = 0;
quat_t q_est = { 1, 0, 0, 0 };
bool filter_init_done = false;

void imu_init()
{
	mpu6050_init(&imu, 0x68);
	uint32_t samples = ltparams_getu(LTPARAMS_IMU_CALIBRATION_SAMPLES);
	mpu6050_calibrate(&imu, samples);
	hmc5883l_init(&mag, 0x1E);
}

void imu_read()
{
	mpu6050_read(&imu);
	for (int i = 0; i < 3; i++) {
		imu_raw[i] = imu.accel[i];
		imu_raw[i + 3] = imu.gyro[i];
	}
	rp2040.fifo.push_nb(0);
	hmc5883l_read(&mag);
	for (int i = 0; i < 3; i++) {
		imu_raw[i + 6] = mag.mag[i];
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
		qcomp_init(&imu_raw[0], &imu_raw[6], q_est);
		last_time = micros();
		return;
	}
	uint32_t now = micros();
	dt = (now - last_time) / 1e6;
	last_time = now;

	float qcomp_alpha = ltparams_get(LTPARAMS_QCOMP_ALPHA);
	float qcomp_beta = ltparams_get(LTPARAMS_QCOMP_BETA);
	float madgwick_beta = ltparams_get(LTPARAMS_MADGWICK_BETA);

	struct quaternion q_;
	switch (ltparams_getu(LTPARAMS_IMU_FILTER_TYPE)) {
	case 0: // QCOMP
		qcomp_update(q_est, &imu_raw[0], &imu_raw[3], &imu_raw[6], dt, qcomp_alpha, qcomp_beta);
		break;
	case 1: // Madgwick
		q_.w = q_est[0];
		q_.x = q_est[1];
		q_.y = q_est[2];
		q_.z = q_est[3];
		madgwick_filter(&q_, &imu_raw[0], &imu_raw[3], &imu_raw[6], dt);
		q_est[0] = q_.w;
		q_est[1] = q_.x;
		q_est[2] = q_.y;
		q_est[3] = q_.z;
		break;
	case 2: // Madgwick AHRS
		madgwick_ahrs_update(q_est, &imu_raw[3], &imu_raw[0], &imu_raw[6], dt, madgwick_beta);
		break;
	case 3: // Mahony AHRS
		mahony_ahrs_update(q_est, &imu_raw[3], &imu_raw[0], &imu_raw[6], dt, madgwick_beta);
		break;
	default:
		break;
	}
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

	for (int i = 0; i < 3; i++) {
		if (isnan(euler[i]) || isinf(euler[i])) {
			filter_init_done = false;
			return;
		}
	}

	struct ltpacket_t packet;
	packet.type = LTPACKET_TYPE_IMU;
	packet.imu.roll = euler[0];
	packet.imu.pitch = euler[1];
	packet.imu.yaw = euler[2];
	packet.imu.vel = dt;
	ltpacket_send(&packet, serial_write);

	if (!ltparams_getu(LTPARAMS_IMU_RAW_ENABLE)) {
		return;
	}
	packet.type = LTPACKET_TYPE_IMU_RAW;
	packet.imu_raw.accel_x = imu_raw[0];
	packet.imu_raw.accel_y = imu_raw[1];
	packet.imu_raw.accel_z = imu_raw[2];
	packet.imu_raw.gyro_x = imu_raw[3];
	packet.imu_raw.gyro_y = imu_raw[4];
	packet.imu_raw.gyro_z = imu_raw[5];
	packet.imu_raw.mag_x = imu_raw[6];
	packet.imu_raw.mag_y = imu_raw[7];
	packet.imu_raw.mag_z = imu_raw[8];
	ltpacket_send(&packet, serial_write);
}

void led_init()
{
	pinMode(LED_BUILTIN, OUTPUT);
}

void led_process()
{
	static unsigned long last_blink = 0;
	float period = ltparams_get(LTPARAMS_BLINK_PERIOD);
	if (blink) {
		unsigned long now = micros();
		if (period == 0) {
			digitalWrite(LED_BUILTIN, HIGH);
			return;
		}
		if (now - last_blink > period * 1e6) {
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
			last_blink = now;
		}
	} else {
		digitalWrite(LED_BUILTIN, LOW);
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