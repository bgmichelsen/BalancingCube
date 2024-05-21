//=============================================================================
//=============================================================================
// File:    balancing_cub_controller.ino
// Brief:   Build definitions for the project
//=============================================================================
// Author:  Brandon Michelsen
// Date:    5/16/24
//=============================================================================
// This file is used under the MIT license.
//=============================================================================
//=============================================================================

#ifndef __BUILD_DEFINES_H
#define __BUILD_DEFINES_H

//=============================================================================
// Defines
//=============================================================================

//
// Build time defines
//
#define USING_CORE_1 true

//
// Queue FIFO Commands
//
#define QUEUE_SEND_ATTITUDE     0
#define QUEUE_SEND_PID_GAIN     1
#define QUEUE_SEND_LQR_GAIN     2
#define QUEUE_SEND_GYRO         3
#define QUEUE_SEND_ACCEL        4
#define QUEUE_SEND_CAL          5

//
// Attitude controller timing
// 
#define ATTITUDE_CTRL_FREQ      52              // 56Hz
#define ATTITUDE_CTRL_MS        20              // 1/freq = 1/52 = ~0.020sec = 20ms
#define ATTITUDE_CTRL_SECONDS   ((float)0.02)   // <--|
#define ATTITUDE_MADGWICK_BETA  ((float)0.07)    // Madgwick filter beta (tunable)

#endif // __BUILD_DEFINES_H

//=============================================================================
// End of file
//=============================================================================