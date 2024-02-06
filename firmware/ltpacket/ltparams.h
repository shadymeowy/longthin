#ifndef LTPARAMS_H
#define LTPARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define LTPARAMS_COUNT 0x16
#define LTPARAMS_RESERVED 0x00
#define LTPARAMS_QCOMP_ALPHA 0x01
#define LTPARAMS_QCOMP_BETA 0x02
#define LTPARAMS_THETA_KP 0x03
#define LTPARAMS_THETA_KI 0x04
#define LTPARAMS_ED_KP 0x05
#define LTPARAMS_ED_KI 0x06
#define LTPARAMS_VDESIRED_KP 0x07
#define LTPARAMS_VDESIRED_KI 0x08
#define LTPARAMS_WDESIRED_KP 0x09
#define LTPARAMS_WDESIRED_KI 0x0a
#define LTPARAMS_WHEEL_RADIUS 0x0b
#define LTPARAMS_WHEEL_DISTANCE 0x0c
#define LTPARAMS_BLINK_PERIOD 0x0d

extern float ltparams[LTPARAMS_COUNT];

static inline void ltparams_load_defaults()
{
    ltparams[LTPARAMS_RESERVED] = 0;
	ltparams[LTPARAMS_QCOMP_ALPHA] = 0.05;
	ltparams[LTPARAMS_QCOMP_BETA] = 0.97;
	
	ltparams[LTPARAMS_THETA_KP] = 0.05;
	ltparams[LTPARAMS_THETA_KI] = 0.;
	ltparams[LTPARAMS_ED_KP] = 0.8;
	ltparams[LTPARAMS_ED_KI] = 0.;
	ltparams[LTPARAMS_VDESIRED_KP] = 1.;
	ltparams[LTPARAMS_VDESIRED_KI] = 0.;
	ltparams[LTPARAMS_WDESIRED_KP] = 1.;
	ltparams[LTPARAMS_WDESIRED_KI] = 0.;
	ltparams[LTPARAMS_WHEEL_RADIUS] = 1.; //1.75e-2;
	ltparams[LTPARAMS_WHEEL_DISTANCE] = 1.; //10e-2;
	ltparams[LTPARAMS_BLINK_PERIOD] = 0.;
}

static inline float ltparams_get(uint8_t index)
{
	return ltparams[index];
}

static inline void ltparams_set(uint8_t index, float value)
{
	ltparams[index] = value;
}

#ifdef __cplusplus
}
#endif

#endif // LTPARAMS_H