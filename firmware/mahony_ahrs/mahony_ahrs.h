//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------------------------------------------------------------------------
// Function declarations

void mahony_ahrs_update(float q[4], float g[3], float a[3], float m[3], float dt, float kp);
void mahony_ahrs_update_imu(float q[4], float g[3], float a[3], float dt, float kp);

#ifdef __cplusplus
}
#endif

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
