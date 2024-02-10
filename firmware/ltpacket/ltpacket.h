#ifndef LTPACKET_H
#define LTPACKET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "packet.h"

#include "ltpacket_type.h"

#define LTMESSAGE(id, idn, typ, prop) idn = id,
enum ltpacket_type_t {
#include "ltpacket_def.h"
};
#undef LTMESSAGE

#define LTMESSAGE(id, idn, typ, prop) struct typ prop;
struct ltpacket_t {
	enum ltpacket_type_t type;
	union {
#include "ltpacket_def.h"
	};
};
#undef LTMESSAGE

int ltpacket_read_buffer(struct ltpacket_t *packet, uint8_t *buffer,
			 uint16_t buffer_length);
int ltpacket_write_buffer(struct ltpacket_t *packet, uint8_t *buffer,
			  uint16_t buffer_length);
int ltpacket_size(struct ltpacket_t *packet);

typedef int (*ltpacket_send_fn)(const uint8_t *buffer, uint16_t buffer_length);

static inline int ltpacket_send(struct ltpacket_t *packet,
				ltpacket_send_fn send_fn)
{
	uint8_t magic = PACKET_MAGIC;
	send_fn((uint8_t *)&magic, 1);

	uint16_t packet_length = ltpacket_size(packet);
	uint16_t length = 5 + packet_length;
	send_fn((uint8_t *)&length, 2);

	uint8_t type = (uint8_t)packet->type;
	uint16_t checksum = crc16(&type, 1);
	send_fn((uint8_t *)&type, 1);

	crc16_continue(&checksum, (uint8_t *)&packet->reserved,
		       packet_length - 1);
	send_fn((uint8_t *)&packet->reserved, packet_length - 1);

	send_fn((uint8_t *)&checksum, 2);
	return 0;
}

#ifdef __cplusplus
}
#endif

#endif