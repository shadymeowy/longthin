#include <Arduino.h>

#include "packet.h"
#include "ltpacket.h"
#include "ltparams.h"

uint8_t buffer[100];
struct packet_reader_t reader;

bool blink = false;

void listen_init()
{
	packet_reader_init(&reader, buffer, sizeof(buffer));
}

void listen_helper()
{
	int c = Serial.read();
	int ret = packet_reader_push(&reader, c);
	if (ret < 0) {
		return;
	}
	struct ltpacket_t packet;
	ltpacket_read_buffer(&packet, packet_reader_head(&reader), ret);
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

void listen_process()
{
	while (Serial.available()) {
		listen_helper();
	}
}

void send_packet(struct ltpacket_t *packet)
{
	uint8_t magic = PACKET_MAGIC;
	Serial.write((uint8_t *)&magic, 1);

	uint16_t packet_length = ltpacket_size(packet);
	uint16_t length = 5 + packet_length;
	Serial.write((uint8_t *)&length, 2);

	uint8_t type = (uint8_t)packet->type;
	uint16_t checksum = crc16(&type, 1);
	Serial.write((uint8_t *)&type, 1);

	crc16_continue(&checksum, (uint8_t *)&packet->reserved, packet_length - 1);
	Serial.write((uint8_t *)&packet->reserved, packet_length - 1);

	Serial.write((uint8_t *)&checksum, 2);
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
	send_packet(&packet);
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

	listen_init();
	led_init();
}

void loop()
{
	listen_process();
	led_process();
	publish_dt();
}