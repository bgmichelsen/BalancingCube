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
#define QUEUE_SEND_EULER        6
#define QUEUE_SEND_LQR_TARGETS  7

//
// Attitude controller timing
// 
#define ATTITUDE_CTRL_FREQ      104             // 104Hz
#define ATTITUDE_CTRL_MS        10              // 1/freq = 1/104 = ~0.010sec = 10ms
#define ATTITUDE_CTRL_uS        10000           // 10ms = 1000us
#define ATTITUDE_CTRL_SECONDS   ((float)0.01)   // <--|
#define ATTITUDE_MADGWICK_BETA  ((float)0.5)    // Madgwick filter beta (tunable)
#define ATTITUDE_COMP_ALPHA     ((float)0.4)

#endif // __BUILD_DEFINES_H

//=============================================================================
// End of file
//=============================================================================