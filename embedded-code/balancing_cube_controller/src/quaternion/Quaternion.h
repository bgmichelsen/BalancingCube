//=============================================================================
//=============================================================================
// File:    Quaternion.h
// Brief:   File with function prototypes and typedefs for working with
//          quaternion objects
//=============================================================================
// Author:  Brandon Michelsen
// Date:    5/12/24
//=============================================================================
// This file is used under the MIT license.
//=============================================================================
// NOTE:    Uses Tait-Bryan variant of Euler angles (rotate Yaw-Pitch-Roll order)
//=============================================================================
//=============================================================================

#ifndef __QUATERNION_H
#define __QUATERNION_H

//=============================================================================
// Include Files
//=============================================================================
#include <stdint.h>
#include "math.h"

//=============================================================================
// Defines
//=============================================================================

#define DEGREE_2_RAD  ((float)(M_PI / 180.0f))
#define RAD_2_DEGREE  ((float)(180.0f / M_PI))

//=============================================================================
// Types
//=============================================================================

// Quaternion type
typedef struct Quaternion
{
    float   qw;     // Real part of the quaternion
    float   qx;     // Imaginary X part
    float   qy;     // Imaginary Y part
    float   qz;     // Imaginary Z part
} Quaternion_t;

// Euler angles type
typedef struct Euler
{
    float   pitch;
    float   roll;
    float   yaw;
} Euler_t;

//=============================================================================
// Function Prototypes
//=============================================================================
Quaternion_t  Quaternion_Euler2Quat(Euler_t &angles);
Euler_t       Quaternion_Quat2Euler(Quaternion_t &q);
Quaternion_t  Quaternion_Norm(Quaternion_t &q);
Quaternion_t  Quaternion_Mult(Quaternion_t &q1, Quaternion_t &q2);
Quaternion_t  Quaternion_Invert(Quaternion_t &q);
Quaternion_t  Quaternion_Add(Quaternion_t &q1, Quaternion_t &q2);
Quaternion_t  Quaternion_ScalarMult(Quaternion_t &q, float scalar);
Euler_t       Quaternion_Degrees2Euler(float x, float y, float z);

#endif // __QUATERNION_H

//=============================================================================
// End of file
//=============================================================================