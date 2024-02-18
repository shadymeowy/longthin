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

int serial_write(const uint8_t *buffer, uint16_t size)
{
	Serial.write(buffer, size);
	Serial1.write(buffer, size);
	return 0;
}

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

	float ed_kp = ltparams_get(LTPARAMS_ED_KP);
	float ed_ki = ltparams_get(LTPARAMS_ED_KI);
	float theta_kp = ltparams_get(LTPARAMS_THETA_KP);
	float theta_ki = ltparams_get(LTPARAMS_THETA_KI);
	float vdesired_kp = ltparams_get(LTPARAMS_VDESIRED_KP);
	float vdesired_ki = ltparams_get(LTPARAMS_VDESIRED_KI);
	float wdesired_kp = ltparams_get(LTPARAMS_WDESIRED_KP);
	float wdesired_ki = ltparams_get(LTPARAMS_WDESIRED_KI);
	float wheel_distance = ltparams_get(LTPARAMS_WHEEL_DISTANCE);
	float wheel_radius = ltparams_get(LTPARAMS_WHEEL_RADIUS);

	if (!manual_mode) {
		float e_d = desired_d - current_d;
		float e_theta = modby2pi(desired_yaw) - modby2pi(current_yaw);
		e_theta = modby2pi(e_theta);
		if (e_theta > 180) {
			e_theta -= 360;
		}

		static float e_d_sum = 0;
		e_d_sum += e_d * dt;
		desired_v = ed_kp * e_d + ed_ki * e_d_sum;

		static float e_theta_sum = 0;
		e_theta_sum += e_theta * dt;
		desired_w = theta_kp * e_theta + theta_ki * e_theta_sum;

		static float e_v_sum = 0;
		float e_v = desired_v - current_vel;
		e_v_sum += e_v * dt;

		static float e_w_sum = 0;
		float e_w = desired_w - current_w;
		e_w_sum += e_w * dt;

		u_v = vdesired_kp * e_v + vdesired_ki * e_v_sum;
		u_w = wdesired_kp * e_w + wdesired_ki * e_w_sum;

		u_l = (u_v - wheel_distance * u_w / 2) / wheel_radius;
		u_r = (u_v + wheel_distance * u_w / 2) / wheel_radius;
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
	struct ltpacket_t packet;
	packet.type = LTPACKET_TYPE_CONTROL_DEBUG;
	struct ltpacket_control_debug_t *p = &packet.control_debug;
	p->current_d = current_d;
	p->current_yaw = current_yaw;
	p->desired_d = desired_d;
	p->desired_yaw = desired_yaw;
	p->current_vel = current_vel;
	p->current_w = current_w;
	p->desired_vel = desired_v;
	p->desired_w = desired_w;
	p->u_v = u_v;
	p->u_w = u_w;
	p->u_r = u_r;
	p->u_l = u_l;
	ltpacket_send(&packet, serial_write);
}

bool blink = false;

uint8_t buffer[100];
struct packet_reader_t reader;
uint8_t buffer1[100];
struct packet_reader_t reader1;

void listen_init()
{
	packet_reader_init(&reader, buffer, sizeof(buffer));
	packet_reader_init(&reader1, buffer1, sizeof(buffer1));
}

void listen_process()
{
	int ret;
	while (Serial.available()) {
		int c = Serial.read();
		if ((ret = packet_reader_push(&reader, c)) < 0) {
			continue;
		}
		listen_handle(ret);
	}
	while (Serial1.available()) {
		int c = Serial1.read();
		if ((ret = packet_reader_push(&reader1, c)) < 0) {
			continue;
		}
		listen_handle(ret);
	}
}

void listen_handle(int data_length)
{
	struct ltpacket_t packet;
	ltpacket_read_buffer(&packet, packet_reader_head(&reader), data_length);
	enum ltparams_index_t index;
	switch (packet.type) {
	case LTPACKET_TYPE_LED:
		blink = packet.led.state;
		break;
	case LTPACKET_TYPE_SETPARAM:
		index = (enum ltparams_index_t)packet.setparam.param;
		ltparams_set(index, packet.setparam.value);
		break;
	case LTPACKET_TYPE_SETPARAMU:
		index = (enum ltparams_index_t)packet.setparamu.param;
		ltparams_setu(index, packet.setparamu.value);
		break;
	case LTPACKET_TYPE_SETPARAMI:
		index = (enum ltparams_index_t)packet.setparami.param;
		ltparams_seti(index, packet.setparami.value);
		break;
	case LTPACKET_TYPE_MOTOR_RAW:
		motor_set_raw(packet.motor_raw.left, packet.motor_raw.right);
		manual_mode = 1;
		break;
	case LTPACKET_TYPE_MOTOR:
		motor_set(packet.motor.left, packet.motor.right);
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
		unsigned long now = millis();
		if (period == 0) {
			digitalWrite(LED_BUILTIN, HIGH);
			return;
		}
		if (now - last_blink > period * 1000) {
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
			last_blink = now;
		}
	} else {
		digitalWrite(LED_BUILTIN, LOW);
	}
}

void setup()
{
	Serial.begin(115200);
	Serial1.begin(115200);

	led_init();
	listen_init();
	motor_init();
}

void loop()
{
	listen_process();
	led_process();
	motor_update();
	motor_publish();
}

void setup1()
{
}

void loop1()
{
}