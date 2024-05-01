#include <Arduino.h>

#include "packet.h"
#include "ltpacket.h"
#include "ltparams.h"

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

void publish_imu()
{
	static uint32_t last_time = 0;
	uint32_t now = micros();
	uint32_t period = ltparams_getu(LTPARAMS_IMU_PUBLISH_PERIOD);
	if (now - last_time < period) {
		return;
	}
	float dt = (now - last_time) / 1000.0f;
	last_time = now;

	struct ltpacket_t packet = {};
	packet.type = LTPACKET_TYPE_IMU;
	packet.imu = { 0 };
	packet.imu.dt = dt;
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
		.pin = 12,
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
		.pin = 11,
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
		.pin = 14,
		.inversion = 0,
		.current_state = 0,
		.last_change_time = 0,
		.tone = 1000,
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