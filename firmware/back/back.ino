#include <Arduino.h>
#include "packet.h"
#include "ltpacket.h"
#include "ltparams.h"

#define MOTOR_LEFT_FORWARD 11
#define MOTOR_LEFT_BACKWARD 10
#define MOTOR_RIGHT_FORWARD 13
#define MOTOR_RIGHT_BACKWARD 12

int16_t motor_left = 0;
int16_t motor_right = 0;
int manual_mode = 0;

float current_d = 0;
float current_yaw = 0;
float desired_d = 0;
float desired_yaw = 0;

float current_vel = 0;
float current_w = 0;
float desired_v = 0;
float desired_w = 0;

float u_v = 0;
float u_w = 0;
float u_r = 0;
float u_l = 0;

void motor_init()
{
	pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
	pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
	pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
	pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
	analogWriteRange(2048);
}

void motor_set_raw(int16_t left, int16_t right)
{
	motor_left = left;
	motor_right = right;
}

void motor_set(float left, float right)
{
	motor_set_raw(left * 2048, right * 2048);
}

static inline float modby2pi(float x)
{
	while (x < 0) {
		x += 360;
	}
	while (x > 360) {
		x -= 360;
	}
	return x;
}

static inline float clamp(float x, float min, float max)
{
	if (x < min) {
		return min;
	}
	if (x > max) {
		return max;
	}
	return x;
}

void motor_update()
{
	static float last_time = 0;
	float current_time = micros();
	float dt = (current_time - last_time) / 1e6;

	if (!manual_mode) {
		float e_d = desired_d - current_d;
		float e_theta = modby2pi(desired_yaw) - modby2pi(current_yaw);
		e_theta = modby2pi(e_theta);
		if (e_theta > 180) {
			e_theta -= 360;
		}
		static float e_d_sum = 0;
		e_d_sum += e_d * dt;
		desired_v = ltparams_get(LTPARAMS_ED_KP) * e_d + ltparams_get(LTPARAMS_ED_KI) * e_d_sum;
		static float e_theta_sum = 0;
		e_theta_sum += e_theta * dt;
		desired_w = ltparams_get(LTPARAMS_THETA_KP) * e_theta + ltparams_get(LTPARAMS_THETA_KI) * e_theta_sum;
		static float e_v_sum = 0;
		float e_v = desired_v - current_vel;
		e_v_sum += e_v * dt;
		static float e_w_sum = 0;
		float e_w = desired_w - current_w;
		e_w_sum += e_w * dt;
		u_v = ltparams_get(LTPARAMS_VDESIRED_KP) * e_v + ltparams_get(LTPARAMS_VDESIRED_KI) * e_v_sum;
		u_w = ltparams_get(LTPARAMS_WDESIRED_KP) * e_w + ltparams_get(LTPARAMS_WDESIRED_KI) * e_w_sum;

		u_l = (u_v - ltparams_get(LTPARAMS_WHEEL_DISTANCE) * u_w / 2) / ltparams_get(LTPARAMS_WHEEL_RADIUS);
		u_r = (u_v + ltparams_get(LTPARAMS_WHEEL_DISTANCE) * u_w / 2) / ltparams_get(LTPARAMS_WHEEL_RADIUS);
		if (desired_d == 0) {
			u_l = 0;
			u_r = 0;
		}
		if (u_l < 0) {
			u_r -= u_l;
			u_l = 0;
		} else if (u_r < 0) {
			u_l -= u_r;
			u_r = 0;
		}
		u_l = clamp(u_l, 0, 1);
		u_r = clamp(u_r, 0, 1);

		motor_set(u_l, u_r);
	}

	static uint32_t last_refresh_time = 0;
	uint32_t now = micros();
	if (now - last_refresh_time < 10000) { // 100 Hz
		return;
	}
	analogWrite(MOTOR_LEFT_FORWARD, motor_left > 0 ? motor_left : 0);
	analogWrite(MOTOR_LEFT_BACKWARD, motor_left < 0 ? -motor_left : 0);
	analogWrite(MOTOR_RIGHT_FORWARD, motor_right > 0 ? motor_right : 0);
	analogWrite(MOTOR_RIGHT_BACKWARD, motor_right < 0 ? -motor_right : 0);
}

void motor_publish()
{
	static uint32_t last_time = 0;
	uint32_t now = millis();
	// 2 Hz
	if (now - last_time < 500) {
		return;
	}
	last_time = now;
	struct ltpacket_control_debug_t packet;
	packet.current_d = current_d;
	packet.current_yaw = current_yaw;
	packet.desired_d = desired_d;
	packet.desired_yaw = desired_yaw;
	packet.current_vel = current_vel;
	packet.current_w = current_w;
	packet.desired_v = desired_v;
	packet.desired_w = desired_w;
	packet.u_v = u_v;
	packet.u_w = u_w;
	packet.u_r = u_r;
	packet.u_l = u_l;

	uint8_t buffer[100];
	struct packet_writer_t writer;
	packet_writer_init(&writer, buffer, sizeof(buffer));
	packet_writer_start(&writer);
	packet_writer_put(&writer, (uint8_t)LTPACKET_TYPE_CONTROL_DEBUG);
	packet_writer_write(&writer, (uint8_t *)&packet, sizeof(packet));
	packet_writer_end(&writer);
	size_t packet_size = packet_writer_packet_length(&writer);
	uint8_t *packet_buffer = packet_writer_packet(&writer);
	if (Serial1.availableForWrite()) {
		Serial1.write(packet_buffer, packet_size);
	}
}

uint8_t buffer[100];
struct packet_reader_t reader;

void listen_init()
{
	packet_reader_init(&reader, buffer, sizeof(buffer));
}

void listen_helper()
{
	int c = Serial1.read();
	if (c == -1) {
		return;
	}
	if (packet_reader_put(&reader, c) < 0) {
		packet_reader_start(&reader);
		return;
	}
	if (packet_reader_available(&reader) < 0) {
		return;
	}
	int packet_len = packet_reader_end(&reader);
	if (packet_len < 0) {
		packet_reader_start(&reader);
		return;
	}
	size_t data_size = packet_reader_data_length(&reader);
	uint8_t *data = packet_reader_data(&reader);
	struct ltpacket_t packet;
	ltpacket_read_buffer(&packet, data, data_size);
	switch (packet.type) {
	case LTPACKET_TYPE_LED:
		digitalWrite(LED_BUILTIN, packet.led.state);
		break;
	case LTPACKET_TYPE_SETPARAM:
		ltparams_set(packet.setparam.param, packet.setparam.value);
		break;
	case LTPACKET_TYPE_MOTOR_RAW:
		motor_set_raw(packet.motor_raw.motor_left, packet.motor_raw.motor_right);
		manual_mode = 1;
		break;
	case LTPACKET_TYPE_MOTOR:
		motor_set(packet.motor.motor_left, packet.motor.motor_right);
		manual_mode = 1;
		break;
	case LTPACKET_TYPE_SETPOINT:
		desired_d = packet.setpoint.vel;
		desired_yaw = packet.setpoint.yaw;
		manual_mode = 0;
		break;
	case LTPACKET_TYPE_IMU:
		current_yaw = packet.imu.yaw;
		current_d = packet.imu.vel; // TODO: Fix later
		break;
	default:
		break;
	}
	packet_reader_start(&reader);
}

void listen_process()
{
	while (Serial1.available()) {
		listen_helper();
	}
}

void setup()
{
	pinMode(LED_BUILTIN, OUTPUT);

	Serial.begin(115200);
	Serial1.begin(115200);

	ltparams_load_defaults();
	listen_init();
	motor_init();
}

void loop()
{
	listen_process();
	motor_update();
	motor_publish();
}

void setup1()
{
}

void loop1()
{
}