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
	switch (packet.type) {
	case LTPACKET_TYPE_LED:
		blink = packet.led.state;
		break;
	case LTPACKET_TYPE_SETPARAM:
		ltparams_set(packet.setparam.param, packet.setparam.value);
		break;
	default:
		break;
	}
}

void publish_dt()
{
	static uint32_t last_time = 0;
	uint32_t now = millis();
	// 10 Hz
	if (now - last_time < 1000 / 10) {
		return;
	}
	float dt = (now - last_time) / 1000.0f;
	last_time = now;

	struct ltpacket_t packet;
	packet.type = LTPACKET_TYPE_IMU;
	packet.imu.roll = 0;
	packet.imu.pitch = 0;
	packet.imu.yaw = micros() / 1e6 * 90;
	packet.imu.vel = dt;
	ltpacket_send(&packet, serial_write);
}

void led_init()
{
	pinMode(LED_BUILTIN, OUTPUT);
}

void led_process()
{
	static unsigned long last_blink = 0;
	float period = 0;
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
	publish_dt();
}