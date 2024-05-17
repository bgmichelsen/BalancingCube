//=============================================================================
//=============================================================================
// File:    AttitudeEstimator.h
// Brief:   Definitions and prototypes for the attitude estimation
//=============================================================================
// Author:  Brandon Michelsen
// Date:    5/16/24
//=============================================================================
// This file is used under the MIT license.
//=============================================================================
//=============================================================================

#ifndef __ATTITUDE_ESTIMATOR_H
#define __ATTITUDE_ESTIMATOR_H

//=============================================================================
// Include Files
//=============================================================================
#include "../quaternion/Quaternion.h"
#include <stdint.h>

//=============================================================================
// Definitions
//=============================================================================

//=============================================================================
// Structs and Types
//=============================================================================

//=============================================================================
// Class Definition
//=============================================================================
class AttitudeEstimator
{
public:
    AttitudeEstimator() {}
    ~AttitudeEstimator() {}

    //=========================================================================
    // Name:        AttitudeEstimator::begin
    // Brief:       Initialize the estimator
    //=========================================================================
    void begin();

    //=========================================================================
    // Name:        AttitudeEstimator::SetQ_Gyro
    // Brief:       Set the gyro quaternion (derivative quaternion)
    // Param[in]:   wx = The X gyro rate (Roll)
    // Param[in]:   wy = The Y gyro rate (Pitch)
    // Param[in]:   wz = The Z gyro rate (Yaw)
    // Param[in]:   dt = Sampling time
    // Retval:      N.A.
    //=========================================================================
    void SetQ_Gyro(float wx, float wy, float wz, float dt);

    //=========================================================================
    // Name:        AttitudeEstimator::SetQ_Accel
    // Brief:       Set the accelerometer quaternion
    // Param[in]:   ax = The X acceleration (Roll)
    // Param[in]:   ay = The Y acceleration (Pitch)
    // Param[in]:   az = The Z acceleration (Yaw)
    // Param[in]:   dt = Sampling time
    // Retval:      N.A.
    //=========================================================================
    void SetQ_Accel(float ax, float ay, float az, float dt);

    //=========================================================================
    // Name:        AttitudeEstimator::Estimate
    // Brief:       Estimate the attitude
    // Retval:      N.A.
    // NOTE:        Must have set the gyro and accel. quaternions
    //=========================================================================
    void Estimate();

    //=========================================================================
    // Name:        AttitudeEstimator::GetAttitude
    // Brief:       Get the attitude
    // Retval:      The attitude of the cube
    //=========================================================================
    Quaternion_t GetAttitude();

    //=========================================================================
    // Name:        AttitudeEstimator::GetQ_Gyro
    // Brief:       Get the gyro quaternion (derivative quaternion)
    // Retval:      The derivative quaternion
    //=========================================================================
    Quaternion_t GetQ_Gyro();

    //=========================================================================
    // Name:        AttitudeEstimator::GetQ_Accel
    // Brief:       Get the accelerometer quaternion
    // Retval:      The accelerometer quaternion
    //=========================================================================
    Quaternion_t GetQ_Accel();
private:
    Quaternion_t _q;
    Quaternion_t _qd;
    Quaternion_t _qa;
};

#endif // __ATTITUDE_ESTIMATOR_H

//=============================================================================
// End of file
//=============================================================================