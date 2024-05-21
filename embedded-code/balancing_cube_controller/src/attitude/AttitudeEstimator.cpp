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
// NOTE: Estimation of attitude based on Madgwick filter (https://nitinjsanket.github.io/tutorials/attitudeest/madgwick)
//=============================================================================
//=============================================================================

//=============================================================================
// Include Files
//=============================================================================
#include "AttitudeEstimator.h"

//=========================================================================
// Name:        AttitudeEstimator::begin
// Brief:       Initialize the estimator
// Param[in]    gx_err = The mean zero x error
// Param[in]    gy_err = The mean zero y error
// Param[in]    gz_err = The mean zero z error
//=========================================================================
void AttitudeEstimator::begin()
{
    // Initialize the quaternions
    _q  = { 1, 0, 0, 0 };
    _qd = { 1, 0, 0, 0 };
    _qa = { 1, 0, 0, 0 };
    _qg = { 1, 0, 0, 0 };
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
void AttitudeEstimator::SetQ_Gyro(float wx, float wy, float wz)
{
    // Compute the quaternion derivative using gyro
    // See STM's note: DT0060
    _qg.qw = 0.5 * (-_q.qx*wx - _q.qy*wy - _q.qz*wz);
    _qg.qx = 0.5 * ( _q.qw*wx - _q.qz*wy + _q.qy*wz);
    _qg.qy = 0.5 * ( _q.qz*wx + _q.qw*wy - _q.qx*wz);
    _qg.qz = 0.5 * (-_q.qy*wx + _q.qx*wy + _q.qw*wz);
}

//=========================================================================
// Name:        AttitudeEstimator::SetQ_Accel
// Brief:       Set the accelerometer quaternion
// Param[in]:   ax = The X acceleration (Roll)
// Param[in]:   ay = The Y acceleration (Pitch)
// Param[in]:   az = The Z acceleration (Yaw)
// Param[in]:   dt = Sampling time
// Retval:      N.A.
// NOTE:        See https://nitinjsanket.github.io/tutorials/attitudeest/madgwick
//=========================================================================
void AttitudeEstimator::SetQ_Accel(float ax, float ay, float az)
{
    // Local variables
    float       fg[3];      // Objective function for gradient descent
    float       J[3][4];    // Jacobian for gradient descent

    _qa.qw = 0.0f;
    _qa.qw = ax;
    _qa.qy = ay;
    _qa.qz = az;

    // Normalize the accelerometer measurements
    _qa = Quaternion_Norm(_qa);

    // Update the objective function
    fg[0] = 2*(_q.qx*_q.qz - _q.qw*_q.qy) - _qa.qx;
    fg[1] = 2*(_q.qw*_q.qx + _q.qy*_q.qz) - _qa.qy;
    fg[2] = 2*(0.5 - _q.qx*_q.qx - _q.qy*_q.qy) - _qa.qz;

    // Update the Jacobian
    J[0][0] = -2*_q.qy;
    J[0][1] =  2*_q.qz;
    J[0][2] = -2*_q.qw;
    J[0][3] =  2*_q.qx;

    J[1][0] =  2*_q.qx;
    J[1][1] =  2*_q.qw;
    J[1][2] =  2*_q.qz;
    J[1][3] =  2*_q.qy;

    J[2][0] =  0.0f;
    J[2][1] = -4*_q.qx;
    J[2][2] = -4*_q.qy;
    J[2][3] =  0.0f;

    // Calculate the gradient descent
    _qa.qw = J[0][0]*fg[0] + J[1][0]*fg[1] + J[2][0]*fg[2];
    _qa.qx = J[0][1]*fg[0] + J[1][1]*fg[1] + J[2][1]*fg[2];
    _qa.qy = J[0][2]*fg[0] + J[1][2]*fg[1] + J[2][2]*fg[2];
    _qa.qz = J[0][3]*fg[0] + J[1][3]*fg[1] + J[2][3]*fg[2];

    // Normalize the gradient decent
    _qa = Quaternion_Norm(_qa);

    // Multiply by beta
    _qa = Quaternion_ScalarMult(_qa, _beta);
}

//=========================================================================
// Name:        AttitudeEstimator::Estimate
// Brief:       Estimate the attitude
// Retval:      N.A.
// NOTE:        Must have set the gyro and accel. quaternions
//=========================================================================
void AttitudeEstimator::Estimate(float dt)
{
    // Fuse the measurements from accel and gyro into derivative quaternion
    _qd.qw = _qg.qw - _qa.qw;
    _qd.qx = _qg.qx - _qa.qx;
    _qd.qy = _qg.qy - _qa.qy;
    _qd.qz = _qg.qz - _qa.qz;
    _qd = Quaternion_ScalarMult(_qd, dt);

    // Calculate the attitude from the derivative quaternion
    _q = Quaternion_Add(_q, _qd);
    _q = Quaternion_Norm(_q);       // Normalize
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