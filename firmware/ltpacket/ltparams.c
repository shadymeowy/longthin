#include "ltparams.h"

#define LTDEF(id, name, type, field, value) [name] = { .field = value },
union ltparam_t ltparams[LTPARAMS_COUNT] = {
#include "ltparams_def.h"
};
#undef LTDEF