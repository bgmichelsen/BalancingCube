//=============================================================================
//=============================================================================
// File:    Quaternion.cpp
// Brief:   File with function definitions and typedefs for working with
//          quaternions
//=============================================================================
// Author:  Brandon Michelsen
// Date:    5/12/24
//=============================================================================
// This file is used under the MIT license.
//=============================================================================
// Note:  See https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
//        for quaternion operations
//=============================================================================
//=============================================================================

//=============================================================================
// Include Files
//=============================================================================
#include "Quaternion.h"
#include "math.h"

//=============================================================================
// Function Definitions
//=============================================================================

//=============================================================================
// Name:        Quaternion_Euler2Quat
// Brief:       Converts Euler angles to quaternion
// param[in]:   angles = The Euler angles to convert
// retval:      Calculated quaternion         
//=============================================================================
Quaternion_t Quaternion_Euler2Quat(Euler_t &angles)
{
    // Local variables
    Quaternion_t  q = { 0, 0, 0, 0 };
    float         cos_roll  = cos(angles.roll / 2);
    float         cos_pitch = cos(angles.pitch / 2);
    float         cos_yaw   = cos(angles.yaw / 2);
    float         sin_roll  = sin(angles.roll / 2);
    float         sin_pitch = sin(angles.pitch / 2);
    float         sin_yaw   = sin(angles.yaw / 2);

    // Get the real part
    q.qw = (cos_roll * cos_pitch * cos_yaw) + (sin_roll * sin_pitch * sin_yaw);

    // Get the X part
    q.qx = (sin_roll * cos_pitch * cos_yaw) + (cos_roll * sin_pitch * sin_yaw);

    // Get the Y part
    q.qy = (cos_roll * sin_pitch * cos_yaw) + (sin_roll * cos_pitch * sin_yaw);

    // Get the Z part
    q.qz = (cos_roll * cos_pitch * sin_yaw) + (sin_roll * sin_pitch * cos_yaw);

    return (q);
}

//=============================================================================
// Name:        Quaternion_Quat2Euler
// Brief:       Converts quaternion to Euler angles
// param[in]:   q = The quaternion to convert
// retval:      Calculated Euler angles
//=============================================================================
Euler_t Quaternion_Quat2Euler(Quaternion_t &q)
{
    // Local variables
    Euler_t   angles = { 0, 0, 0 };
    float     qw2 = q.qw * q.qw;
    float     qx2 = q.qx * q.qx;
    float     qy2 = q.qy * q.qy;
    float     qz2 = q.qz * q.qz;
    float     num = 0;
    float     den = 0;

    // Calculate roll
    num = 2 * (q.qw * q.qx + q.qy * q.qz);
    den = qw2 - qx2 - qy2 + qz2;
    angles.roll = atan2(num, den);

    // Calculate pitch
    num = 2 * (q.qw * q.qy - q.qx * q.qz);
    angles.pitch = asin(num);

    // Calculate yaw
    num = 2 * (q.qw * q.qz + q.qx * q.qy);
    den = qw2 + qx2 - qy2 - qz2;
    angles.yaw = atan2(num, den);

    return (angles);
}

//=============================================================================
// Name:        Quaternion_Norm
// Brief:       Normalizes a quaternion
// param[in]:   q = The quaternion to normalize
// retval:      Normalized quaternion
//=============================================================================
Quaternion_t Quaternion_Norm(Quaternion_t &q)
{
    // Local variables
    Quaternion_t  q_ret = { 0, 0, 0, 0 };
    float         norm;
    float         qw2 = q.qw * q.qw;
    float         qx2 = q.qx * q.qx;
    float         qy2 = q.qy * q.qy;
    float         qz2 = q.qz * q.qz;

    // Calculate the normalization factor
    norm = sqrt(qw2 + qx2 + qy2 + qz2);

    // Normalize the quaternion components
    q_ret.qw = q.qw / norm;
    q_ret.qx = q.qx / norm;
    q_ret.qy = q.qy / norm;
    q_ret.qz = q.qz / norm;

    return (q_ret);
}

//=============================================================================
// Name:        Quaternion_Mult
// Brief:       Multiplies two quaternions
// param[in]:   q1 = The first quaternion
// param[in]:   q2 = The second quaternion
// retval:      Calculated quaternion product
// NOTE:        Quaternion multiplication is not commutative
//              a*b != b*a
//=============================================================================
Quaternion_t Quaternion_Mult(Quaternion_t &q1, Quaternion_t &q2)
{
    // Local variables
    Quaternion_t  q_ret = { 0, 0, 0, 0 };

    // Calculate the real part
    q_ret.qw = q1.qw * q2.qw - q1.qx * q2.qx - q1.qy * q2.qy - q1.qz * q2.qz;

    // Calculate the x part
    q_ret.qx = q1.qw * q2.qx + q1.qx * q2.qw - q1.qy * q2.qz + q1.qz * q2.qy;

    // Calculate the y part
    q_ret.qy = q1.qw * q2.qy + q1.qx * q2.qz + q1.qy * q2.qw - q1.qz * q2.qx;

    // Calculate the z part
    q_ret.qz = q1.qw * q2.qz - q1.qx * q2.qy + q1.qy * q2.qx + q1.qz * q2.qw;

    return (q_ret);
}

//=============================================================================
// Name:        Quaternion_Invert
// Brief:       Gets the inverted quaternion (conjugate)
// param[in]:   q = The quaternion to invert
// retval:      Inverted quaternion
//=============================================================================
Quaternion_t Quaternion_Invert(Quaternion_t &q)
{
    // Local variables
    Quaternion_t  q_ret = { 0, 0, 0, 0 };
 
    q_ret.qw = q.qw;
    q_ret.qx = -1 * q.qx;
    q_ret.qy = -1 * q.qy;
    q_ret.qz = -1 * q.qz;

    return (q_ret);
}

//=============================================================================
// Name:        Quaternion_Add
// Brief:       Adds two quaternions
// param[in]:   q1 = The first quaternion
// param[in]:   q2 = The second quaternion
// retval:      Calculated quaternion sum
//=============================================================================
Quaternion_t Quaternion_Add(Quaternion_t &q1, Quaternion_t &q2)
{
    // Local variables
    Quaternion_t  q_ret = { 0, 0, 0, 0 };

    q_ret.qw = q1.qw + q2.qw;
    q_ret.qx = q1.qx + q2.qx;
    q_ret.qy = q1.qy + q2.qy;
    q_ret.qz = q1.qz + q2.qz;

    return (q_ret);
}

//=============================================================================
// Name:        Quaternion_ScalarMult
// Brief:       Multiplies a quaternion by a scalar
// param[in]:   q = The quaternion
// param[in]:   scalar = The scalar value
// retval:      Calculated quaternion product
//=============================================================================
Quaternion_t Quaternion_ScalarMult(Quaternion_t &q, float scalar)
{
    // Local variables
    Quaternion_t  q_ret = { 0, 0, 0, 0 };

    q_ret.qw = q.qw * scalar;
    q_ret.qx = q.qx * scalar;
    q_ret.qy = q.qy * scalar;
    q_ret.qz = q.qz * scalar;

    return (q_ret);
}

//=============================================================================
// Name:        Quaternion_Degrees2Euler
// Brief:       Converts degree-based angles to Euler radians
// param[in]:   x = Degrees around x
// param[in]:   y = Degrees around y
// param[in]:   z = Degrees around z
// retval:      Converted radians Euler angles
//=============================================================================
Euler_t Quaternion_Degrees2Euler(float x, float y, float z)
{
    // Local variables
    Euler_t   angles = { 0, 0, 0 };

    angles.roll   = x * DEGREE_2_RAD;
    angles.pitch  = y * DEGREE_2_RAD;
    angles.yaw    = z * DEGREE_2_RAD;

    return (angles);
}

//=============================================================================
// End of File
//=============================================================================
