#include "packet.h"

#include <string.h>

int packet_reader_init(struct packet_reader_t *reader, uint8_t *buffer,
		       uint16_t buffer_length)
{
	reader->buffer = buffer;
	reader->buffer_length = buffer_length;
	packet_reader_start(reader);
	return 0;
}

int packet_reader_start(struct packet_reader_t *reader)
{
	reader->buffer_index = 0;
	return 0;
}

int packet_reader_put(struct packet_reader_t *reader, uint8_t byte)
{
	if (reader->buffer_index >= reader->buffer_length) {
		return PACKET_BUFFER_OVERFLOW;
	}
	if (reader->buffer_index == 0 && byte != PACKET_MAGIC) {
		return PACKET_INVALID_MAGIC;
	}
	reader->buffer[reader->buffer_index++] = byte;
	return 0;
}

int packet_reader_write(struct packet_reader_t *reader, uint8_t *data,
			uint16_t data_length)
{
	if (reader->buffer_index + data_length >= reader->buffer_length) {
		return PACKET_BUFFER_OVERFLOW;
	}
	if (reader->buffer_index == 0 && data[0] != PACKET_MAGIC) {
		return PACKET_INVALID_MAGIC;
	}
	memcpy(reader->buffer + reader->buffer_index, data, data_length);
	reader->buffer_index += data_length;
	return 0;
}

int packet_reader_available(struct packet_reader_t *reader)
{
	if (reader->buffer_index < 5) {
		return PACKET_INCOMPLETE;
	}
	int packet_len = reader->buffer[1] | (reader->buffer[2] << 8);
	if (packet_len > reader->buffer_length) {
		return PACKET_INCOMPLETE;
	}
	if (reader->buffer_index != packet_len) {
		return PACKET_INVALID_LENGTH;
	}
	return packet_len;
}

int packet_reader_end(struct packet_reader_t *reader)
{
	int packet_len2 = reader->buffer[1] | (reader->buffer[2] << 8);
	if (packet_reader_available(reader) < 0) {
		return PACKET_INCOMPLETE;
	}
	int packet_len = reader->buffer[1] | (reader->buffer[2] << 8);
	if (packet_len == 0) {
		return PACKET_INCOMPLETE;
	}
	uint16_t crc = crc16(reader->buffer, packet_len - 2);
	uint16_t packet_crc = reader->buffer[packet_len - 2] |
			      (reader->buffer[packet_len - 1] << 8);
	if (crc != packet_crc) {
		return PACKET_INVALID_CRC;
	}
	return packet_len;
}

int packet_reader_data_length(struct packet_reader_t *reader)
{
	return packet_reader_end(reader) - 5;
}

uint8_t *packet_reader_data(struct packet_reader_t *reader)
{
	return reader->buffer + 3;
}

int packet_writer_init(struct packet_writer_t *writer, uint8_t *buffer,
		       uint16_t buffer_length)
{
	writer->buffer = buffer;
	writer->buffer_length = buffer_length;
	packet_writer_start(writer);
	return 0;
}

int packet_writer_start(struct packet_writer_t *writer)
{
	writer->buffer_index = 0;
	writer->buffer[writer->buffer_index++] = PACKET_MAGIC;
	writer->buffer[writer->buffer_index++] = 0;
	writer->buffer[writer->buffer_index++] = 0;
	return 0;
}

int packet_writer_put(struct packet_writer_t *writer, uint8_t byte)
{
	if (writer->buffer_index >= writer->buffer_length) {
		return PACKET_BUFFER_OVERFLOW;
	}
	writer->buffer[writer->buffer_index++] = byte;
	return 0;
}

int packet_writer_write(struct packet_writer_t *writer, uint8_t *data,
			uint16_t data_length)
{
	if (writer->buffer_index + data_length >= writer->buffer_length) {
		return PACKET_BUFFER_OVERFLOW;
	}
	memcpy(writer->buffer + writer->buffer_index, data, data_length);
	writer->buffer_index += data_length;
	return 0;
}

int packet_writer_end(struct packet_writer_t *writer)
{
	int packet_len = writer->buffer_index + 2;
	writer->buffer[1] = packet_len & 0xFF;
	writer->buffer[2] = (packet_len >> 8) & 0xFF;
	uint16_t crc = crc16(writer->buffer, writer->buffer_index);
	writer->buffer[writer->buffer_index++] = crc & 0xFF;
	writer->buffer[writer->buffer_index++] = (crc >> 8) & 0xFF;
	return writer->buffer_index;
}

uint8_t *packet_writer_packet(struct packet_writer_t *reader)
{
	return reader->buffer;
}

int packet_writer_packet_length(struct packet_writer_t *reader)
{
	return reader->buffer_index;
}

int packet_write(uint8_t *buffer, uint16_t buffer_length, uint8_t *data,
		 uint16_t data_length)
{
	if (buffer_length < data_length + 5) {
		return PACKET_BUFFER_OVERFLOW;
	}
	buffer[0] = PACKET_MAGIC;
	buffer[1] = (data_length + 5) & 0xFF;
	buffer[2] = ((data_length + 5) >> 8) & 0xFF;
	memcpy(buffer + 3, data, data_length);
	uint16_t crc = crc16(buffer, data_length + 3);
	buffer[data_length + 3] = crc & 0xFF;
	buffer[data_length + 4] = (crc >> 8) & 0xFF;
	return data_length + 5;
}

int packet_check(uint8_t *buffer, uint16_t buffer_length)
{
	if (buffer_length < 5) {
		return PACKET_INCOMPLETE;
	}
	if (buffer[0] != PACKET_MAGIC) {
		return PACKET_INVALID_MAGIC;
	}
	int packet_len = buffer[1] | (buffer[2] << 8);
	if (packet_len > buffer_length) {
		return PACKET_INCOMPLETE;
	}
	if (buffer_length != packet_len) {
		return PACKET_INVALID_LENGTH;
	}
	uint16_t crc = crc16(buffer, packet_len - 2);
	uint16_t packet_crc = buffer[packet_len - 2] |
			      (buffer[packet_len - 1] << 8);
	if (crc != packet_crc) {
		return PACKET_INVALID_CRC;
	}
	return packet_len;
}

int packet_read(uint8_t *buffer, uint16_t buffer_length, uint8_t *data,
		uint16_t data_length)
{
	int packet_len = packet_check(buffer, buffer_length);
	if (packet_len < 0) {
		return packet_len;
	}
	if (data_length < packet_len - 5) {
		return PACKET_BUFFER_OVERFLOW;
	}
	memcpy(data, buffer + 3, packet_len - 5);
	return packet_len - 5;
}

uint16_t crc16(uint8_t *data, uint16_t length)
{
	uint16_t crc = 0xFFFF;
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
	return crc;
}