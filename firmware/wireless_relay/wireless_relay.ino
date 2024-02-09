#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <string.h>
#include <stdint.h>

#include "packet.h"

uint8_t mac_self[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t mac_receiver[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#define BUFFER_SIZE 256
#define SERIAL_BAUD 115200
#define SERIAL Serial

void on_data_send(uint8_t *mac_addr, uint8_t sendStatus)
{
	// No check for send status for now because we are broadcasting
	//if (sendStatus == 0) {
	//} else {
	//}
}

void on_data_recv(uint8_t *mac_addr, uint8_t *buffer, uint8_t buffer_length)
{
	if (packet_check(buffer, buffer_length) < 0) {
		return;
	}
	SERIAL.write(buffer, buffer_length);
}

uint8_t buffer[BUFFER_SIZE];
struct packet_reader_t reader;

void setup()
{
	SERIAL.begin(SERIAL_BAUD);
	WiFi.mode(WIFI_STA);
	WiFi.disconnect();
	if (esp_now_init() != 0) {
		pinMode(LED_BUILTIN, OUTPUT);
		digitalWrite(LED_BUILTIN, HIGH);
		return;
	}
	packet_reader_init(&reader, buffer, BUFFER_SIZE);
	esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
	esp_now_register_send_cb(on_data_send);
	esp_now_add_peer(mac_receiver, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
	esp_now_register_recv_cb(on_data_recv);
}

void loop()
{
	int c = SERIAL.read();
	int ret;
	if ((ret = packet_reader_push(&reader, c)) < 0) {
		return;
	}
	// A better way to handle this?
	esp_now_send(mac_receiver, reader.buffer, ret + 5);
}
