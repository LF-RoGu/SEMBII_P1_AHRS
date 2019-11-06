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
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration
typedef struct
{
	float roll;
	float pitch;
	float yaw;
}MahonyAHRSEuler_t;
//---------------------------------------------------------------------------------------------------
// Function declarations

MahonyAHRSEuler_t MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
MahonyAHRSEuler_t MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
