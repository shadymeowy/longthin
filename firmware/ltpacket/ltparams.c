#include "ltparams.h"

union ltparam_t ltparams[LTPARAMS_COUNT] = {
	[LTPARAMS_RESERVED] = { .ui = 0 },
	[LTPARAMS_QCOMP_ALPHA] = { .f = 0.05 },
	[LTPARAMS_QCOMP_BETA] = { .f = 0.97 },
	[LTPARAMS_THETA_KP] = { .f = 0.05 },
	[LTPARAMS_THETA_KI] = { .f = 0. },
	[LTPARAMS_ED_KP] = { .f = 0.8 },
	[LTPARAMS_ED_KI] = { .f = 0. },
	[LTPARAMS_VDESIRED_KP] = { .f = 1. },
	[LTPARAMS_VDESIRED_KI] = { .f = 0. },
	[LTPARAMS_WDESIRED_KP] = { .f = 1. },
	[LTPARAMS_WDESIRED_KI] = { .f = 0. },
	[LTPARAMS_WHEEL_RADIUS] = { .f = 1. },
	[LTPARAMS_WHEEL_DISTANCE] = { .f = 1. },
	[LTPARAMS_BLINK_PERIOD] = { .f = 0. }
};