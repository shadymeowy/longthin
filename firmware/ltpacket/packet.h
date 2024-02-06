#ifndef PACKET_H
#define PACKET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define PACKET_MAGIC 0x78
#define PACKET_MAX_LENGTH 1024

#define PACKET_INVALID_MAGIC -1
#define PACKET_BUFFER_OVERFLOW -2
#define PACKET_INCOMPLETE -3
#define PACKET_INVALID_CRC -4
#define PACKET_INVALID_LENGTH -5

struct packet_reader_t {
	uint8_t *buffer;
	uint16_t buffer_length;
	uint16_t buffer_index;
};

int packet_reader_init(struct packet_reader_t *reader, uint8_t *buffer,
		       uint16_t buffer_length);
int packet_reader_push(struct packet_reader_t *reader, int c);
static inline uint8_t *packet_reader_head(struct packet_reader_t *reader)
{
	return reader->buffer + 3;
}

int packet_write(uint8_t *buffer, uint16_t buffer_length, uint8_t *data,
		 uint16_t data_length);
int packet_check(uint8_t *buffer, uint16_t buffer_length);

uint16_t crc16(uint8_t *data, uint16_t length);
void crc16_continue(uint16_t *crc_, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif