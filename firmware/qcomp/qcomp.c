#include "qcomp.h"

#include <stdlib.h>

void qcomp_init(vec3_t a_, vec3_t m_, quat_t q)
{
  vec3_t a = {a_[0], a_[1], a_[2]};
  vec3_t m = {m_[0], m_[1], m_[2]};
  vec3_normalize(a);
  vec3_normalize(m);

  // eqn 25
  quat_t q_acc = {0};
  if (a[2] < 0)
  {
    real_t tmp = sqrtf(2 * (1 - a[2]));
    q_acc[0] = -a[1] / tmp;
    q_acc[1] = tmp / 2;
    q_acc[2] = 0;
    q_acc[3] = a[0] / tmp;
  }
  else
  {
    real_t tmp = sqrtf(2 * (1 + a[2]));
    q_acc[0] = tmp / 2;
    q_acc[1] = -a[1] / tmp;
    q_acc[2] = a[0] / tmp;
    q_acc[3] = 0;
  }

  vec3_t l = {0};
  rotate_with_iquat(q_acc, m, l);
  real_t gamma = l[0] * l[0] + l[1] * l[1];
  real_t gamma_sqrt = sqrtf(gamma);

  // eqn 31
  quat_t q_mag = {0};
  if (l[0] < 0)
  {
    real_t tmp = sqrtf(2 * (gamma - l[0] * gamma_sqrt));
    q_mag[0] = l[1] / tmp;
    q_mag[3] = tmp / (2 * gamma_sqrt);
  }
  else
  {
    real_t tmp = sqrtf(2 * (gamma + l[0] * gamma_sqrt));
    q_mag[0] = tmp / (2 * gamma_sqrt);
    q_mag[3] = l[1] / tmp;
  }

  quat_mul(q_acc, q_mag, q);

  // output is q_gl not q_lg
  quat_conjugate(q);
}

void qcomp_update(quat_t q_, vec3_t a_, vec3_t w_, vec3_t m_, real_t dt,
                  real_t alpha_, real_t beta)
{
  vec3_t a = {a_[0], a_[1], a_[2]};
  vec3_t m = {m_[0], m_[1], m_[2]};
  vec3_t w = {w_[0], w_[1], w_[2]};
  real_t a_norm = vec3_norm(a);
  vec3_normalize(a);
  vec3_normalize(m);

  quat_t q_lg_p = {q_[0], q_[1], q_[2], q_[3]};
  // input is q_gl not q_lg
  quat_conjugate(q_lg_p);

  quat_t q_lg_dot = {0};
  // eqn 39-41
  q_lg_dot[0] =
      0 * q_lg_p[0] + w[0] * q_lg_p[1] + w[1] * q_lg_p[2] + w[2] * q_lg_p[3];
  q_lg_dot[1] =
      -w[0] * q_lg_p[0] + 0 * q_lg_p[1] + w[2] * q_lg_p[2] - w[1] * q_lg_p[3];
  q_lg_dot[2] =
      -w[1] * q_lg_p[0] - w[2] * q_lg_p[1] + 0 * q_lg_p[2] + w[0] * q_lg_p[3];
  q_lg_dot[3] =
      -w[2] * q_lg_p[0] + w[1] * q_lg_p[1] - w[0] * q_lg_p[2] + 0 * q_lg_p[3];

  // eqn 42
  quat_t q_lgw = {q_lg_dot[0], q_lg_dot[1], q_lg_dot[2], q_lg_dot[3]};
  quat_scalar(q_lgw, dt, q_lgw);
  quat_add(q_lgw, q_lg_p, q_lgw);

  // eqn 44
  vec3_t g = {0};
  rotate_with_iquat(q_lgw, a, g);

  // eqn 47
  quat_t q_acc = {0};
  /*if (g[2] < 0) {
          real_t tmp = sqrtf(2 * (1 - g[2]));
          q_acc[0] = -g[1] / tmp;
          q_acc[1] = tmp / 2;
          q_acc[2] = 0;
          q_acc[3] = g[0] / tmp;
  } else {*/
  {
    real_t tmp = sqrtf(2 * (1 + g[2]));
    q_acc[0] = tmp / 2;
    q_acc[1] = -g[1] / tmp;
    q_acc[2] = g[0] / tmp;
    q_acc[3] = 0;
  }
  //}

  // eqn 60-61
  real_t e_m = abs(a_norm - 9.81) / 9.81;
  real_t alpha = alpha_;
  if (e_m < 0.1)
  {
    alpha = alpha_;
  }
  else if (e_m < 0.2)
  {
    alpha = alpha_ * (0.2 - e_m) * 10;
  }
  else
  {
    alpha = 0;
  }

  // eqn 50-52
  quat_t q_acc_hat = {0};
  quat_t q_i = {1, 0, 0, 0};
  quat_slerp(q_i, q_acc, alpha, q_acc_hat);

  // eqn 53
  quat_t q_lg_prime = {0};
  quat_mul(q_lgw, q_acc_hat, q_lg_prime);

  // eqn 54
  vec3_t l = {0};
  rotate_with_iquat(q_lg_prime, m, l);

  // eqn 58
  quat_t q_mag = {0};
  real_t gamma = l[0] * l[0] + l[1] * l[1];
  real_t gamma_sqrt = sqrtf(gamma);
  /*if (l[0] < 0) {
          real_t tmp = sqrtf(2 * (gamma - l[0] * gamma_sqrt));
          q_mag[0] = l[1] / tmp;
          q_mag[3] = tmp / (2 * gamma_sqrt);
  } else {*/
  real_t tmp = sqrtf(2 * (gamma + l[0] * gamma_sqrt));
  q_mag[0] = tmp / (2 * gamma_sqrt);
  q_mag[3] = l[1] / tmp;
  ///}

  quat_t q_mag_hat = {0};
  quat_slerp(q_i, q_mag, beta, q_mag_hat);

  // eqn 59
  quat_t q_lg = {0};
  quat_mul(q_lg_prime, q_mag_hat, q_lg);

  // output is q_gl not q_lg again
  quat_conjugate(q_lg);
  q_[0] = q_lg[0];
  q_[1] = q_lg[1];
  q_[2] = q_lg[2];
  q_[3] = q_lg[3];
}
