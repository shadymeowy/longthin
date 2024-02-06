#ifndef QCOMPLEMENTARY_H
#define QCOMPLEMENTARY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "qmath.h"
#include <math.h>
#include <stdint.h>

void qcomp_init(vec3_t a, vec3_t m, quat_t q);
void qcomp_update(quat_t q_, vec3_t a_, vec3_t w_, vec3_t m_, real_t dt,
                  real_t alpha_, real_t beta);

#ifdef __cplusplus
}
#endif
#endif