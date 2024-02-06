#ifndef QMATH_H
#define QMATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

typedef float real_t;
typedef real_t vec3_t[3];
typedef real_t mat33_t[3][3];
typedef real_t quat_t[4];

static inline void vec3_cross(vec3_t a, vec3_t b, vec3_t c) {
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
}

static inline real_t vec3_dot(vec3_t a, vec3_t b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static inline real_t vec3_norm(vec3_t a) { return sqrtf(vec3_dot(a, a)); }

static inline void vec3_normalize(vec3_t a) {
  real_t norm = vec3_norm(a);
  a[0] /= norm;
  a[1] /= norm;
  a[2] /= norm;
}

static inline void quat_conjugate(quat_t q) {
  q[1] = -q[1];
  q[2] = -q[2];
  q[3] = -q[3];
}

static inline real_t quat_norm(quat_t q) {
  return sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

static inline void quat_normalize(quat_t q) {
  real_t norm = quat_norm(q);
  q[0] /= norm;
  q[1] /= norm;
  q[2] /= norm;
  q[3] /= norm;
}

static inline void quat_mul(quat_t a, quat_t b, quat_t c) {
  c[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  c[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  c[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  c[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

static inline void quat_scalar(quat_t q, real_t scalar, quat_t r) {
  r[0] = q[0] * scalar;
  r[1] = q[1] * scalar;
  r[2] = q[2] * scalar;
  r[3] = q[3] * scalar;
}

static inline void quat_add(quat_t a, quat_t b, quat_t c) {
  c[0] = a[0] + b[0];
  c[1] = a[1] + b[1];
  c[2] = a[2] + b[2];
  c[3] = a[3] + b[3];
}

static inline void quat_sub(quat_t a, quat_t b, quat_t c) {
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
  c[2] = a[2] - b[2];
  c[3] = a[3] - b[3];
}

static inline void lerp(quat_t a, quat_t b, real_t t, quat_t c) {
  quat_t tmp = {0};
  quat_scalar(a, 1 - t, tmp);
  quat_scalar(b, t, c);
  quat_add(tmp, c, c);
  quat_normalize(c);
}

static inline void quat_slerp(quat_t a, quat_t b, real_t t, quat_t c) // check
{
  real_t dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (dot < 0) { // not sure if this is necessary
    quat_scalar(b, -1, b);
    dot = -dot;
  }
  if (dot > 0.9995) {
    lerp(a, b, t, c);
    return;
  }
  real_t W = acosf(dot);
  real_t sinfW = sinf(W);
  real_t W1 = sinf((1 - t) * W) / sinfW;
  real_t W2 = sinf(t * W) / sinfW;

  quat_t tmp = {0};
  quat_scalar(a, W1, tmp);
  quat_scalar(b, W2, c);
  quat_add(tmp, c, c);
}

static inline void euler_from_quat(quat_t q_, real_t euler[3]) {
  quat_t q = {q_[0], q_[1], q_[2], q_[3]};
  quat_normalize(q);
  euler[0] = atan2f(2 * (q[0] * q[1] + q[2] * q[3]),
                    1 - 2 * (q[1] * q[1] + q[2] * q[2])) *
             180 / M_PI;
  euler[1] = asinf(2 * (q[0] * q[2] - q[3] * q[1])) * 180 / M_PI;
  euler[2] = atan2f(2 * (q[0] * q[3] + q[1] * q[2]),
                    1 - 2 * (q[2] * q[2] + q[3] * q[3])) *
             180 / M_PI;
  if (euler[2] < 0) {
    euler[2] += 360;
  }
}

static inline void mat33_mul_vec3(mat33_t m, vec3_t v, vec3_t r) {
  r[0] = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2];
  r[1] = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2];
  r[2] = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2];
}

static inline void mat33_from_quat(mat33_t m, quat_t q) {
  real_t q00 = q[0] * q[0];
  real_t q11 = q[1] * q[1];
  real_t q22 = q[2] * q[2];
  real_t q33 = q[3] * q[3];
  real_t q01 = q[0] * q[1];
  real_t q02 = q[0] * q[2];
  real_t q03 = q[0] * q[3];
  real_t q12 = q[1] * q[2];
  real_t q13 = q[1] * q[3];
  real_t q23 = q[2] * q[3];

  m[0][0] = q00 + q11 - q22 - q33;
  m[0][1] = 2.0f * (q12 - q03);
  m[0][2] = 2.0f * (q13 + q02);
  m[1][0] = 2.0f * (q12 + q03);
  m[1][1] = q00 - q11 + q22 - q33;
  m[1][2] = 2.0f * (q23 - q01);
  m[2][0] = 2.0f * (q13 - q02);
  m[2][1] = 2.0f * (q23 + q01);
  m[2][2] = q00 - q11 - q22 + q33;
}

static inline void rotate_with_quat(quat_t q, vec3_t v, vec3_t r) {
  quat_t qv = {0, v[0], v[1], v[2]};
  quat_t qvq = {0};
  quat_t q_conj = {q[0], -q[1], -q[2], -q[3]};
  quat_mul(qv, q_conj, qvq);
  quat_mul(q, qvq, qv);
  r[0] = qv[1];
  r[1] = qv[2];
  r[2] = qv[3];
}

static inline void rotate_with_iquat(quat_t q, vec3_t v, vec3_t r) {
  quat_t qv = {0, v[0], v[1], v[2]};
  quat_t qvq = {0};
  quat_t q_conj = {q[0], -q[1], -q[2], -q[3]};
  quat_mul(q_conj, qv, qvq);
  quat_mul(qvq, q, qv);
  r[0] = qv[1];
  r[1] = qv[2];
  r[2] = qv[3];
}

#ifdef __cplusplus
}
#endif

#endif