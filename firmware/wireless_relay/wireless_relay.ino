#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <user_interface.h>
#include <string.h>
#include <stdint.h>

#include "packet.h"
#include "chacha.h"

uint8_t mac_self[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t mac_receiver[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#define BUFFER_SIZE 256
#define SERIAL_BAUD 115200
#define SERIAL_INST Serial

#define LCG_A 1664525
#define LCG_C 1013904223
static const uint8_t key[32] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
				 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F };
uint32_t nonce[2] = { 0, 0 };
struct chacha_t chacha;

void on_data_send(uint8_t *mac_addr, uint8_t sendStatus)
{
	// No check for send status for now because we are broadcasting
	//if (sendStatus == 0) {
	//} else {
	//}
}

void on_data_recv(uint8_t *mac_addr, uint8_t *buffer, uint8_t buffer_length)
{
	// If packet is too short, ignore
	if (buffer_length < 5 + 8) {
		return;
	}
	// Set the nonce
	chacha_nonce_set8(&chacha, buffer);
	// Decrypt the packet
	chacha_crypt(&chacha, buffer + 8, buffer_length - 8);
	// Check the packet
	if (packet_check(buffer + 8, buffer_length - 8) < 0) {
		return;
	}
	// Relay the packet
	SERIAL_INST.write(buffer + 8, buffer_length - 8);
}

uint8_t buffer[BUFFER_SIZE];
struct packet_reader_t reader;

void setup()
{
	SERIAL_INST.begin(SERIAL_BAUD);
	WiFi.mode(WIFI_STA);
	WiFi.disconnect();
	if (esp_now_init() != 0) {
		pinMode(LED_BUILTIN, OUTPUT);
		digitalWrite(LED_BUILTIN, HIGH);
		return;
	}
	nonce[0] = RANDOM_REG32;
	delay(10);
	nonce[1] = RANDOM_REG32;
	chacha_init(&chacha, (uint8_t *)key);

	packet_reader_init(&reader, buffer, BUFFER_SIZE);
	esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
	esp_now_register_send_cb(on_data_send);
	esp_now_add_peer(mac_receiver, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
	esp_now_register_recv_cb(on_data_recv);
}

void loop()
{
	int c = SERIAL_INST.read();
	int ret;
	if ((ret = packet_reader_push(&reader, c)) < 0) {
		return;
	}
	size_t packet_len = ret + 5;
	// Set the nonce
	nonce[0] *= LCG_A;
	nonce[0] += LCG_C;
	nonce[1] *= LCG_A;
	nonce[1] += LCG_C;
	chacha_nonce_set32(&chacha, nonce);
	// Encrypt the packet
	chacha_crypt(&chacha, reader.buffer, packet_len);
	// Create a buffer for nonce + packet
	// TODO: Use a non-variable length buffer
	uint8_t packet[packet_len + 8];
	memcpy(packet, nonce, 8);
	memcpy(packet + 8, reader.buffer, packet_len);
	// Send the packet
	esp_now_send(mac_receiver, packet, packet_len + 8);
}
