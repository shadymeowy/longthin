#include <Arduino.h>
#include <Wire.h>

#include "mpu6050.h"
#include "hmc5883l.h"
#include "qcomp.h"
#include "packet.h"
#include "ltpacket.h"
#include "ltparams.h"

uint8_t buffer[100];
struct packet_reader_t reader;

void listen_init()
{
	packet_reader_init(&reader, buffer, sizeof(buffer));
}

void listen_helper()
{
	int c = Serial2.read();
	if (c == -1) {
		return;
	}
	int ret = packet_reader_push(&reader, c);
	if (ret < 0) {
		return;
	}
	struct ltpacket_t packet;
	ltpacket_read_buffer(&packet, packet_reader_head(&reader), ret);
	switch (packet.type) {
	case LTPACKET_TYPE_LED:
		digitalWrite(LED_BUILTIN, packet.led.state);
		break;
	case LTPACKET_TYPE_SETPARAM:
		ltparams_set(packet.setparam.param, packet.setparam.value);
		break;
	default:
		break;
	}
}

void listen_process()
{
	while (Serial2.available()) {
		listen_helper();
	}
}

void send_packet(struct ltpacket_t *packet)
{
	uint8_t magic = PACKET_MAGIC;
	Serial2.write((uint8_t *)&magic, 1);

	uint16_t packet_length = ltpacket_size(packet);
	uint16_t length = 5 + packet_length;
	Serial2.write((uint8_t *)&length, 2);

	uint8_t type = (uint8_t)packet->type;
	uint16_t checksum = crc16(&type, 1);
	Serial2.write((uint8_t *)&type, 1);

	crc16_continue(&checksum, (uint8_t *)&packet->reserved, packet_length - 1);
	Serial2.write((uint8_t *)&packet->reserved, packet_length - 1);

	Serial2.write((uint8_t *)&checksum, 2);
}

float imu_raw[9] = { 0, 0, 1, 0, 0, 0, 1, 1, 0 };
struct mpu6050 imu = { 0 };
struct hmc5883l mag = { 0 };
float dt = 0;
quat_t q_est = { 1, 0, 0, 0 };

void imu_init()
{
	mpu6050_init(&imu, 0x68);
	mpu6050_calibrate(&imu, 2000);
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
	if (last_time == 0) {
		qcomp_init(&imu_raw[0], &imu_raw[6], q_est);
		last_time = micros();
		return;
	}
	for (int i = 0; i < 4; i++) {
		if (isnan(q_est[i]) || isinf(q_est[i])) {
			qcomp_init(&imu_raw[0], &imu_raw[6], q_est);
			last_time = micros();
			return;
		}
	}
	uint32_t now = micros();
	dt = (now - last_time) / 1e6;
	last_time = now;
	float alpha = ltparams_get(LTPARAMS_QCOMP_ALPHA);
	float beta = ltparams_get(LTPARAMS_QCOMP_BETA);
	qcomp_update(q_est, &imu_raw[0], &imu_raw[3], &imu_raw[6], dt, alpha, beta);
}

void imu_publish()
{
	static uint32_t last_time = 0;
	uint32_t now = micros();
	if (now - last_time < 33333) { // 30 Hz
		return;
	}
	last_time = now;
	float euler[3] = { 0 };
	euler_from_quat(q_est, euler);
	struct ltpacket_t packet;
	packet.type = LTPACKET_TYPE_IMU;
	packet.imu.roll = euler[0];
	packet.imu.pitch = euler[1];
	packet.imu.yaw = euler[2];
	packet.imu.vel = dt;
	send_packet(&packet);
}

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	Wire1.setSDA(18);
	Wire1.setSCL(19);
	Serial2.setTX(4);
	Serial2.setRX(5);
	Serial.begin(115200);
	Serial2.begin(115200);

	ltparams_load_defaults();
	listen_init();
}

void loop()
{
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