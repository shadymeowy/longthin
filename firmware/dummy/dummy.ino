#include <Arduino.h>

#include "packet.h"
#include "ltpacket.h"
#include "ltparams.h"

bool blink = false;

uint8_t buffer[100];
struct packet_reader_t reader;
uint8_t buffer1[100];
struct packet_reader_t reader1;
// uint8_t buffer2[100];
// struct packet_reader_t reader2;

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
	// packet_reader_init(&reader2, buffer2, sizeof(buffer2));
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
			// Serial2.write(data - 3, ret + 5);
		}
	}
	while (Serial1.available()) {
		c = Serial1.read();
		if ((ret = packet_reader_push(&reader1, c)) >= 0) {
			data = packet_reader_head(&reader1);
			ltpacket_read_buffer(&packet, data, ret);
			listen_handle(&packet);
			Serial.write(data - 3, ret + 5);
			// Serial2.write(data - 3, ret + 5);
		}
	}
	/* while (Serial2.available()) {
		c = Serial2.read();
		if ((ret = packet_reader_push(&reader2, c)) >= 0) {
			data = packet_reader_head(&reader2);
			ltpacket_read_buffer(&packet, data, ret);
			listen_handle(&packet);
			Serial.write(data - 3, ret + 5);
			Serial1.write(data - 3, ret + 5);
		}
	} */
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

void publish_imu()
{
	static int counter = 0;
	static uint32_t last_time = 0;
	uint32_t now = micros();
	uint32_t period = ltparams_getu(LTPARAMS_IMU_PUBLISH_PERIOD);
	if (now - last_time < period) {
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
	uint32_t now = micros();
	uint32_t period = ltparams_getu(LTPARAMS_MOTOR_OUTPUT_PUBLISH_PERIOD);
	if (now - last_time < period) {
		return;
	}
	float dt = (now - last_time) / 1e6;
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