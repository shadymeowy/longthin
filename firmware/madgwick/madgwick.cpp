#include "madgwick.h"

struct quaternion quat_mult(struct quaternion L, struct quaternion R)
{
	struct quaternion product;
	product.w = (L.w * R.w) - (L.x * R.x) - (L.y * R.y) - (L.z * R.z);
	product.x = (L.w * R.x) + (L.x * R.w) + (L.y * R.z) - (L.z * R.y);
	product.y = (L.w * R.y) - (L.x * R.z) + (L.y * R.w) + (L.z * R.x);
	product.z = (L.w * R.z) + (L.x * R.y) - (L.y * R.x) + (L.z * R.w);
	return product;
}

// The resulting quaternion is a global variable (q_est), so it is not returned or passed by reference/pointer
// Gyroscope Angular Velocity components are in Radians per Second
// Accelerometer componets will be normalized
void madgwick_filter_nomag(struct quaternion *q_est, float a[3], float g[3], float dt)
{
	struct quaternion q_prev = *q_est;
	struct quaternion q_est_dot = { 0 };
	struct quaternion q_a = { 0, a[0], a[1], a[2] };
	float f_g[3] = { 0 };
	float J_g[3][4] = { 0 };

	struct quaternion gradient = { 0 };
	struct quaternion q_w;
	// the real component is zero, which the Madgwick uses to simplfy quat. mult.
	q_w.w = 0;
	q_w.x = g[0];
	q_w.y = g[1];
	q_w.z = g[2];

	quat_scalar(&q_w, 0.5);
	q_w = quat_mult(q_prev, q_w);

	/* NOTE
    * Page 10 states equation (40) substitutes equation (13) into it. This seems false, as he actually
    * substitutes equation (12), q_se_dot_w, not equation (13), q_se_w.
    * 
    * // quat_scalar(&q_w, deltaT);               // equation (13) integrates the angles velocity to position
    * // quat_add(&q_w, q_w, q_prev);         // addition part of equation (13)
    */

	quat_normalize(&q_a);
	f_g[0] = 2 * (q_prev.x * q_prev.z - q_prev.w * q_prev.y) - q_a.x;
	f_g[1] = 2 * (q_prev.w * q_prev.x + q_prev.y * q_prev.z) - q_a.y;
	f_g[2] = 2 * (0.5 - q_prev.x * q_prev.x - q_prev.y * q_prev.y) - q_a.z;

	//Compute the Jacobian matrix, equation (26), for gravity
	J_g[0][0] = -2 * q_prev.y;
	J_g[0][1] = 2 * q_prev.z;
	J_g[0][2] = -2 * q_prev.w;
	J_g[0][3] = 2 * q_prev.x;

	J_g[1][0] = 2 * q_prev.x;
	J_g[1][1] = 2 * q_prev.w;
	J_g[1][2] = 2 * q_prev.z;
	J_g[1][3] = 2 * q_prev.y;

	J_g[2][0] = 0;
	J_g[2][1] = -4 * q_prev.x;
	J_g[2][2] = -4 * q_prev.y;
	J_g[2][3] = 0;

	gradient.w = J_g[0][0] * f_g[0] + J_g[1][0] * f_g[1] + J_g[2][0] * f_g[2];
	gradient.x = J_g[0][1] * f_g[0] + J_g[1][1] * f_g[1] + J_g[2][1] * f_g[2];
	gradient.y = J_g[0][2] * f_g[0] + J_g[1][2] * f_g[1] + J_g[2][2] * f_g[2];
	gradient.z = J_g[0][3] * f_g[0] + J_g[1][3] * f_g[1] + J_g[2][3] * f_g[2];

	// Normalize the gradient, equation (44)
	quat_normalize(&gradient);

	// multiply normalized gradient by beta
	quat_scalar(&gradient, BETA);
	// subtract above from q_w, the integrated gyro quaternion
	quat_sub(&q_est_dot, q_w, gradient);
	quat_scalar(&q_est_dot, dt);
	// Integrate orientation rate to find position
	quat_add(q_est, q_prev, q_est_dot);
	// normalize the orientation of the estimate
	quat_normalize(q_est);
}

void madgwick_filter(struct quaternion *q_est, float a[3], float g[3], float m[3], float dt)
{
	struct quaternion q_prev = *q_est;
	struct quaternion q_est_dot = { 0 };
	struct quaternion q_a = { 0, a[0], a[1], a[2] };
	struct quaternion q_w = {0, g[0], g[1], g[2]};
    struct quaternion q_m = {0, m[0], m[1], m[2]};
    struct quaternion gradient = { 0 };
    struct quaternion q_b = {0};

    float f_gb[6] = { 0 };
	float J_gb[6][4] = { 0 };

	quat_scalar(&q_w, 0.5);
	q_w = quat_mult(q_prev, q_w);
    quat_normalize(&q_a);
    quat_normalize(&q_m);

    // compute the reference direction of flux
    q_b = quat_mult(q_prev, quat_mult(q_m, quat_conjugate(q_prev)));
    q_b.w = 0;
    q_b.x = sqrt(q_b.x * q_b.x + q_b.y * q_b.y);
    q_b.y = 0;
	
	f_gb[0] = 2 * (q_prev.x * q_prev.z - q_prev.w * q_prev.y) - q_a.x;
	f_gb[1] = 2 * (q_prev.w * q_prev.x + q_prev.y * q_prev.z) - q_a.y;
	f_gb[2] = 2 * (0.5 - q_prev.x * q_prev.x - q_prev.y * q_prev.y) - q_a.z;
	f_gb[3] = 2 * q_b.x * (0.5 - q_prev.y * q_prev.y - q_prev.z * q_prev.z) + 2 * q_b.z * (q_prev.x * q_prev.z - q_prev.w * q_prev.y) - q_m.x;
	f_gb[4] = 2 * q_b.x * (q_prev.x * q_prev.y - q_prev.w * q_prev.z) + 2 * q_b.z * (q_prev.w * q_prev.x + q_prev.y * q_prev.z) - q_m.y;
	f_gb[5] = 2 * q_b.x * (q_prev.w * q_prev.y + q_prev.x * q_prev.z) + 2 * q_b.z * (0.5 - q_prev.x * q_prev.x - q_prev.y * q_prev.y) - q_m.z;

	//Compute the Jacobian matrix, equation (26), for gravity
	J_gb[0][0] = -2 * q_prev.y;
	J_gb[0][1] = 2 * q_prev.z;
	J_gb[0][2] = -2 * q_prev.w;
	J_gb[0][3] = 2 * q_prev.x;

	J_gb[1][0] = 2 * q_prev.x;
	J_gb[1][1] = 2 * q_prev.w;
	J_gb[1][2] = 2 * q_prev.z;
	J_gb[1][3] = 2 * q_prev.y;

	J_gb[2][0] = 0;
	J_gb[2][1] = -4 * q_prev.x;
	J_gb[2][2] = -4 * q_prev.y;
	J_gb[2][3] = 0;

	J_gb[3][0] = -2 * q_b.z * q_prev.y;
	J_gb[3][1] = 2 * q_b.z * q_prev.z;
	J_gb[3][2] = -4 * q_b.x * q_prev.y - 2 * q_b.z * q_prev.w;
	J_gb[3][3] = -4 * q_b.x * q_prev.z + 2 * q_b.z * q_prev.x;

	J_gb[4][0] = -2 * q_b.x * q_prev.z + 2 * q_b.z * q_prev.x;
	J_gb[4][1] = 2 * q_b.x * q_prev.y + 2 * q_b.z * q_prev.w;
	J_gb[4][2] = 2 * q_b.x * q_prev.x + 2 * q_b.z * q_prev.z;
	J_gb[4][3] = -2 * q_b.x * q_prev.w + 2 * q_b.z * q_prev.y;

	J_gb[5][0] = 2 * q_b.x * q_prev.y;
	J_gb[5][1] = 2 * q_b.x * q_prev.z - 4 * q_b.z * q_prev.x;
	J_gb[5][2] = 2 * q_b.x * q_prev.w - 4 * q_b.z * q_prev.y;
	J_gb[5][3] = 2 * q_b.x * q_prev.x;

	gradient.w = J_gb[0][0] * f_gb[0] + J_gb[1][0] * f_gb[1] + J_gb[2][0] * f_gb[2] + J_gb[3][0] * f_gb[3] + J_gb[4][0] * f_gb[4] + J_gb[5][0] * f_gb[5];
	gradient.x = J_gb[0][1] * f_gb[0] + J_gb[1][1] * f_gb[1] + J_gb[2][1] * f_gb[2] + J_gb[3][1] * f_gb[3] + J_gb[4][1] * f_gb[4] + J_gb[5][1] * f_gb[5];
	gradient.y = J_gb[0][2] * f_gb[0] + J_gb[1][2] * f_gb[1] + J_gb[2][2] * f_gb[2] + J_gb[3][2] * f_gb[3] + J_gb[4][2] * f_gb[4] + J_gb[5][2] * f_gb[5];
	gradient.z = J_gb[0][3] * f_gb[0] + J_gb[1][3] * f_gb[1] + J_gb[2][3] * f_gb[2] + J_gb[3][3] * f_gb[3] + J_gb[4][3] * f_gb[4] + J_gb[5][3] * f_gb[5];

	// Normalize the gradient, equation (44)
	quat_normalize(&gradient);
	// multiply normalized gradient by beta
	quat_scalar(&gradient, BETA);
	// subtract above from q_w, the integrated gyro quaternion
	quat_sub(&q_est_dot, q_w, gradient);
	quat_scalar(&q_est_dot, dt);
	// Integrate orientation rate to find position
	quat_add(q_est, q_prev, q_est_dot);
	// normalize the orientation of the estimate
	quat_normalize(q_est);
}

/*
 returns as pointers, roll pitch and yaw from the quaternion generated in imu_filter
 Assume right hand system
 Roll is about the x axis, represented as phi
 Pitch is about the y axis, represented as theta
 Yaw is about the z axis, represented as psi (trident looking greek symbol)
 */
void euler_angles(struct quaternion q, float *roll, float *pitch, float *yaw)
{
	*yaw = atan2f(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
	*pitch = asinf(2 * (q.w * q.y - q.z * q.x));
	*roll = atan2f(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));

	*yaw *= (180.0f / PI);
	*pitch *= (180.0f / PI);
	*roll *= (180.0f / PI);
}
