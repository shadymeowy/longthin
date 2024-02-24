//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef madgwick_ahrs_h
#define madgwick_ahrs_h

#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------------------------------------------------------------------------
// Function declarations

void madgwick_ahrs_update(float q[4], float g[3], float a[3], float m[3], float dt, float beta);
void madgwick_ahrs_update_imu(float q[4], float g[3], float a[3], float dt, float beta);

#ifdef __cplusplus
}
#endif

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
