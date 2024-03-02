#include <Arduino.h>

#include "packet.h"
#include "ltpacket.h"
#include "ltparams.h"

bool blink = false;

uint8_t buffer[100];
struct packet_reader_t reader;
uint8_t buffer1[100];
struct packet_reader_t reader1;

int serial_write(const uint8_t *buffer, uint16_t size)
{
	Serial.write(buffer, size);
	Serial1.write(buffer, size);
	return 0;
}

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
	case LTPACKET_TYPE_REBOOT:
		rp2040.reboot();
		break;
	default:
		break;
	}
}

void publish_imu()
{
	static int counter = 0;
	static uint32_t last_time = 0;
	uint32_t now = millis();
	// 30 Hz
	if (now - last_time < 1000 / 30) {
		return;
	}
	float dt = (now - last_time) / 1000.0f;
	last_time = now;

	struct ltpacket_t packet;
	packet.type = LTPACKET_TYPE_IMU;
	packet.imu.roll = cos(counter * 5 * M_PI / 180) * 45;
	packet.imu.pitch = sin(counter * 5 * M_PI / 180) * 45;
	packet.imu.yaw = (counter * 5) % 360;
	counter++;
	packet.imu.vel = dt;
	ltpacket_send(&packet, serial_write);
}

void publish_motor()
{
	static int counter = 0;
	static uint32_t last_time = 0;
	uint32_t now = millis();
	// 10 Hz
	if (now - last_time < 1000 / 10) {
		return;
	}
	float dt = (now - last_time) / 1000.0f;
	last_time = now;

	struct ltpacket_t packet;
	packet.type = LTPACKET_TYPE_MOTOR;
	packet.motor.left = -cos(counter * 5 * M_PI / 180);
	packet.motor.right = -sin(counter * 5 * M_PI / 180);
	counter++;
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

	listen_init();
	led_init();
}

void loop()
{
	listen_process();
	led_process();
	publish_imu();
	publish_motor();
}