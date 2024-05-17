//=============================================================================
//=============================================================================
// File:    AttitudeEstimator.cpp
// Brief:   Definitions and prototypes for the attitude estimation
//=============================================================================
// Author:  Brandon Michelsen
// Date:    5/16/24
//=============================================================================
// This file is used under the MIT license.
//=============================================================================
//=============================================================================

//=============================================================================
// Include Files
//=============================================================================
#include "AttitudeEstimator.h"

//=========================================================================
// Name:        AttitudeEstimator::begin
// Brief:       Initialize the estimator
//=========================================================================
void AttitudeEstimator::begin()
{
    // Initialize the quaternions
    _q  = { 1, 0, 0, 0 };
    _qd = { 1, 0, 0, 0 };
    _qa = { 1, 0, 0, 0 };
}

//=========================================================================
// Name:        AttitudeEstimator::SetQ_Gyro
// Brief:       Set the gyro quaternion (derivative quaternion)
// Param[in]:   wx = The X gyro rate (Roll)
// Param[in]:   wy = The Y gyro rate (Pitch)
// Param[in]:   wz = The Z gyro rate (Yaw)
// Param[in]:   dt = Sampling time
// Retval:      N.A.
//=========================================================================
void AttitudeEstimator::SetQ_Gyro(float wx, float wy, float wz, float dt)
{
    // Compute the quaternion derivative
    // See STM's note: DT0060
    _qd.qw = 0.5 * (-_q.qx*wx - _q.qy*wy - _q.qz*wz);
    _qd.qx = 0.5 * ( _q.qw*wx - _q.qz*wy + _q.qy*wz);
    _qd.qy = 0.5 * ( _q.qz*wx + _q.qw*wy - _q.qx*wz);
    _qd.qz = 0.5 * (-_q.qy*wx + _q.qx*wy + _q.qw*wz);
    _qd = Quaternion_ScalarMult(_qd, dt);
}

//=========================================================================
// Name:        AttitudeEstimator::SetQ_Accel
// Brief:       Set the accelerometer quaternion
// Param[in]:   ax = The X acceleration (Roll)
// Param[in]:   ay = The Y acceleration (Pitch)
// Param[in]:   az = The Z acceleration (Yaw)
// Param[in]:   dt = Sampling time
// Retval:      N.A.
//=========================================================================
void AttitudeEstimator::SetQ_Accel(float ax, float ay, float az, float dt)
{
    // TODO: Calculate the acceleromter quaternion
}

//=========================================================================
// Name:        AttitudeEstimator::Estimate
// Brief:       Estimate the attitude
// Retval:      N.A.
// NOTE:        Must have set the gyro and accel. quaternions
//=========================================================================
void AttitudeEstimator::Estimate()
{
    // Calculate the attitude from the derivative quaternion
    _q = Quaternion_Add(_q, _qd);
    _q = Quaternion_Norm(_q);       // Normalize

    // TODO: Linear interpolation with accel quaternion
}

//=========================================================================
// Name:        AttitudeEstimator::GetAttitude
// Brief:       Get the attitude
// Retval:      The attitude of the cube
//=========================================================================
Quaternion_t AttitudeEstimator::GetAttitude()
{
    return _q;
}

//=========================================================================
// Name:        AttitudeEstimator::GetQ_Gyro
// Brief:       Get the gyro quaternion (derivative quaternion)
// Retval:      The derivative quaternion
//=========================================================================
Quaternion_t AttitudeEstimator::GetQ_Gyro()
{
    return _qd;
}

//=========================================================================
// Name:        AttitudeEstimator::GetQ_Accel
// Brief:       Get the accelerometer quaternion
// Retval:      The accelerometer quaternion
//=========================================================================
Quaternion_t AttitudeEstimator::GetQ_Accel()
{
    return _qa;
}

//=============================================================================
// End of file
//=============================================================================