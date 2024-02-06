#include "packet.h"

#include <string.h>

int packet_reader_init(struct packet_reader_t *reader, uint8_t *buffer,
		       uint16_t buffer_length)
{
	reader->buffer = buffer;
	reader->buffer_length = buffer_length;
	reader->buffer_index = 0;
	return 0;
}

int packet_reader_push(struct packet_reader_t *reader, int c)
{
	int ret;
	if (c < 0) {
		return c;
	}
	if (reader->buffer_index >= reader->buffer_length) {
		reader->buffer_index = 0;
		return PACKET_BUFFER_OVERFLOW;
	}
	reader->buffer[reader->buffer_index++] = c;
	ret = packet_check(reader->buffer, reader->buffer_index);
	if (ret == PACKET_INCOMPLETE) {
		return ret;
	}
	if (ret < 0) {
		reader->buffer_index = 0;
		return ret;
	}
	reader->buffer_index = 0;
	return ret - 5;
}

int packet_write(uint8_t *buffer, uint16_t buffer_length, uint8_t *data,
		 uint16_t data_length)
{
	int packet_len = data_length + 5;
	if (buffer_length < packet_len) {
		return PACKET_BUFFER_OVERFLOW;
	}
	buffer[0] = PACKET_MAGIC;
	buffer[1] = packet_len & 0xFF;
	buffer[2] = (packet_len >> 8) & 0xFF;
	memcpy(buffer + 3, data, data_length);
	uint16_t crc = crc16(buffer + 3, data_length);
	buffer[data_length + 3] = crc & 0xFF;
	buffer[data_length + 4] = (crc >> 8) & 0xFF;
	return packet_len;
}

int packet_check(uint8_t *packet, uint16_t packet_len)
{
	if (packet_len < 5) {
		return PACKET_INCOMPLETE;
	}
	if (packet[0] != PACKET_MAGIC) {
		return PACKET_INVALID_MAGIC;
	}
	if (packet_len > PACKET_MAX_LENGTH) {
		return PACKET_INVALID_LENGTH;
	}
	int packet_len2 = packet[1] | (packet[2] << 8);
	if (packet_len2 > PACKET_MAX_LENGTH) {
		return PACKET_INVALID_LENGTH;
	}
	if (packet_len2 > packet_len) {
		return PACKET_INCOMPLETE;
	}
	if (packet_len2 < 5) {
		return PACKET_INVALID_LENGTH;
	}
	if (packet_len2 < packet_len) {
		return PACKET_INVALID_LENGTH;
	}
	uint16_t crc = crc16(packet + 3, packet_len2 - 5);
	uint16_t packet_crc = packet[packet_len2 - 2] |
			      (packet[packet_len2 - 1] << 8);
	if (crc != packet_crc) {
		return PACKET_INVALID_CRC;
	}
	return packet_len2;
}

uint16_t crc16(uint8_t *data, uint16_t length)
{
	uint16_t crc = 0xFFFF;
	crc16_continue(&crc, data, length);
	return crc;
}

void crc16_continue(uint16_t *crc_, uint8_t *data, uint16_t length)
{
	uint16_t crc = *crc_;
	uint8_t i;
	while (length--) {
		crc ^= *data++;
		for (i = 0; i < 8; i++) {
			if (crc & 0x0001) {
				crc >>= 1;
				crc ^= 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}
	*crc_ = crc;
}