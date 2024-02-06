//
//  madgwickFilter.h
//  madgwickFilter
//
//  Created by Blake Johnson on 4/28/20.
//

#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

#ifndef PI
#define PI 3.14159265358979f
#endif

#ifndef GYRO_MEAN_ERROR
#define GYRO_MEAN_ERROR \
	PI * (10.0f / 180.0f) // 5 deg/s gyroscope measurement error (in rad/s)
#endif

#ifndef BETA
#define BETA sqrt(3.0f / 4.0f) * GYRO_MEAN_ERROR
#endif

#include <math.h>
#include <stdio.h>

struct quaternion {
	float w;
	float x;
	float y;
	float z;
};

// Multiply two quaternions and return a copy of the result, prod = L * R
struct quaternion quat_mult(struct quaternion q_L, struct quaternion q_R);

// Multiply a reference of a quaternion by a scalar, q = s*q
static inline void quat_scalar(struct quaternion *q, float scalar)
{
	q->w *= scalar;
	q->x *= scalar;
	q->y *= scalar;
	q->z *= scalar;
}

// Adds two quaternions together and the sum is the pointer to another quaternion, Sum = L + R
static inline void quat_add(struct quaternion *Sum, struct quaternion L,
			    struct quaternion R)
{
	Sum->w = L.w + R.w;
	Sum->x = L.x + R.x;
	Sum->y = L.y + R.y;
	Sum->z = L.z + R.z;
}

// Subtracts two quaternions together and the sum is the pointer to another quaternion, sum = L - R
static inline void quat_sub(struct quaternion *Sum, struct quaternion L,
			    struct quaternion R)
{
	Sum->w = L.w - R.w;
	Sum->x = L.x - R.x;
	Sum->y = L.y - R.y;
	Sum->z = L.z - R.z;
}

// the conjugate of a quaternion is it's imaginary component sign changed  q* = [s, -v] if q = [s, v]
static inline struct quaternion quat_conjugate(struct quaternion q)
{
	q.x = -q.x;
	q.y = -q.y;
	q.z = -q.z;
	return q;
}

// norm of a quaternion is the same as a complex number
// sqrt( q1^2 + q2^2 + q3^2 + q4^2)
// the norm is also the sqrt(q * conjugate(q)), but thats a lot of operations in the quaternion multiplication
static inline float quat_norm(struct quaternion q)
{
	return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// Normalizes pointer q by calling quat_norm(q),
static inline void quat_normalize(struct quaternion *q)
{
	float norm = quat_norm(*q);
	q->w /= norm;
	q->x /= norm;
	q->y /= norm;
	q->z /= norm;
}

void madgwick_filter_nomag(struct quaternion *q_est, float a[3], float g[3], float dt);
void madgwick_filter(struct quaternion *q_est, float a[3], float g[3], float m[3], float dt);
void euler_angles(struct quaternion q, float *roll, float *pitch, float *yaw);

#endif /* MADGWICK_FILTER_H */
