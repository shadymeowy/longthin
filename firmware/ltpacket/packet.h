#ifndef PACKET_H
#define PACKET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define PACKET_MAGIC 0x78
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

int packet_reader_init(struct packet_reader_t *reader, uint8_t *buffer, uint16_t buffer_length);
int packet_reader_start(struct packet_reader_t *reader);
int packet_reader_put(struct packet_reader_t *reader, uint8_t byte);
int packet_reader_write(struct packet_reader_t *reader, uint8_t *data, uint16_t data_length);
int packet_reader_available(struct packet_reader_t *reader);
int packet_reader_end(struct packet_reader_t *reader);
int packet_reader_data_length(struct packet_reader_t *reader);
uint8_t *packet_reader_data(struct packet_reader_t *reader);

uint8_t *packet_data(struct packet_reader_t *reader);

struct packet_writer_t {
	uint8_t *buffer;
	uint16_t buffer_length;
	uint16_t buffer_index;
};

int packet_writer_init(struct packet_writer_t *writer, uint8_t *buffer, uint16_t buffer_length);
int packet_writer_start(struct packet_writer_t *writer);
int packet_writer_put(struct packet_writer_t *writer, uint8_t byte);
int packet_writer_write(struct packet_writer_t *writer, uint8_t *data, uint16_t data_length);
int packet_writer_end(struct packet_writer_t *writer);
int packet_writer_packet_length(struct packet_writer_t *writer);
uint8_t *packet_writer_packet(struct packet_writer_t *writer);

int packet_write(uint8_t *buffer, uint16_t buffer_length, uint8_t *data, uint16_t data_length);
int packet_check(uint8_t *buffer, uint16_t buffer_length);
int packet_read(uint8_t *buffer, uint16_t buffer_length, uint8_t *data, uint16_t data_length);

uint16_t crc16(uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif