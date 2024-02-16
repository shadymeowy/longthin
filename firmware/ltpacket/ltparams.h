#ifndef LTPARAMS_H
#define LTPARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

enum ltparams_type_t {
	LTPARAMS_TYPE_FLOAT,
	LTPARAMS_TYPE_UINT32,
	LTPARAMS_TYPE_INT32
};

// Some bold assumptions about the target platform
union ltparam_t {
	float f;
	uint32_t ui;
	int32_t i;
	uint8_t b[4];
};
// Raise an error if the assumption is wrong
static_assert(sizeof(float) == 4, "float is not 4 bytes long");
static_assert(sizeof(uint32_t) == 4, "uint32_t is not 4 bytes long");
static_assert(sizeof(int32_t) == 4, "int32_t is not 4 bytes long");
static_assert(sizeof(uint8_t) == 1, "uint8_t is not 1 byte long");
static_assert(sizeof(union ltparam_t) == 4,
	      "union ltparam_t is not 4 bytes long");

#include "ltparams_def.h"

static inline float ltparams_get(enum ltparams_index_t index)
{
	if (index >= LTPARAMS_COUNT)
		return 0;
	return ltparams[index].f;
}

static inline void ltparams_set(enum ltparams_index_t index, float value)
{
	if (index >= LTPARAMS_COUNT)
		return;
	ltparams[index].f = value;
}

static inline void ltparams_setu(enum ltparams_index_t index, uint32_t value)
{
	if (index >= LTPARAMS_COUNT)
		return;
	ltparams[index].ui = value;
}

static inline uint32_t ltparams_getu(enum ltparams_index_t index)
{
	if (index >= LTPARAMS_COUNT)
		return 0;
	return ltparams[index].ui;
}

static inline void ltparams_seti(enum ltparams_index_t index, int32_t value)
{
	if (index >= LTPARAMS_COUNT)
		return;
	ltparams[index].i = value;
}

static inline int32_t ltparams_geti(enum ltparams_index_t index)
{
	if (index >= LTPARAMS_COUNT)
		return 0;
	return ltparams[index].i;
}

#ifdef __cplusplus
}
#endif

#endif // LTPARAMS_H