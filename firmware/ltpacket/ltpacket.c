#include "ltpacket.h"
#include <string.h>

#define LTMESSAGE(id, idn, typ, prop)                                  \
	case idn:                                                      \
		if (buffer_length < sizeof(struct typ)) {              \
			return -1;                                     \
		}                                                      \
		memcpy(&packet->prop, buffer + 1, sizeof(struct typ)); \
		return 1 + sizeof(struct typ);

int ltpacket_read_buffer(struct ltpacket_t *packet, uint8_t *buffer,
			 uint16_t buffer_length)
{
	if (buffer_length < 1) {
		return -1;
	}
	packet->type = (enum ltpacket_type_t)buffer[0];
	switch (packet->type) {
#include "ltpacket_def.h"
	default:
		return -1;
	}
}
#undef LTMESSAGE

#define LTMESSAGE(id, idn, typ, prop)                                  \
	case idn:                                                      \
		if (buffer_length < sizeof(struct typ)) {              \
			return -1;                                     \
		}                                                      \
		memcpy(buffer + 1, &packet->prop, sizeof(struct typ)); \
		return 1 + sizeof(struct typ);

int ltpacket_write_buffer(struct ltpacket_t *packet, uint8_t *buffer,
			  uint16_t buffer_length)
{
	if (buffer_length < 1) {
		return -1;
	}
	buffer[0] = packet->type;
	switch (packet->type) {
#include "ltpacket_def.h"
	default:
		return -1;
	}
}
#undef LTMESSAGE

#define LTMESSAGE(id, idn, typ, prop) \
	case idn:                     \
		return 1 + sizeof(struct typ);

int ltpacket_size(struct ltpacket_t *packet)
{
	switch (packet->type) {
#include "ltpacket_def.h"
	default:
		return -1;
	}
}
#undef LTMESSAGE