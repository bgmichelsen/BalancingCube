//=============================================================================
//=============================================================================
// File:    balancing_cub_controller.ino
// Brief:   Main code for the project
//=============================================================================
// Author:  Brandon Michelsen
// Date:    5/12/24
//=============================================================================
// This file is used under the MIT license.
//=============================================================================
//=============================================================================

#include "src/quaternion/Quaternion.h"
#include "src/core_queue/CoreQueue.h"
#include "LSM6DSOXSensor.h"
#include "Wire.h"
#include "PinDefs.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

//=============================================================================
// Defines
//=============================================================================
// #define USE_SECOND_CORE

//=============================================================================
// Objects / RAM
//=============================================================================

//
// IMU data
//
LSM6DSOXSensor  IMU = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

//
// Queue data
//
#ifdef USING_CORE1
QueueFIFO       Core0_FIFO = QueueFIFO(8);
QueueFIFO       Core1_FIFO = QueueFIFO(8);
#endif

//
// Attitude data
//
Quaternion_t  Attitude = { 1, 0, 0, 0 };

//
// Flags
//

//=============================================================================
// Additional function prototypes
//=============================================================================

//=============================================================================
// Core functions
//=============================================================================

// Core 0 Setup
void setup() 
{
    // Local variables

#ifndef USING_CORE1
    Serial.begin();
#endif

    // Setup pins
    pinMode(LED_BUILTIN, OUTPUT);

    // Start I2C and LSM6DSOX
    Wire.begin();
    Wire.setClock(400000);
    IMU.begin();

    // Start the gyro of the IMU
    if (LSM6DSOX_OK != IMU.Enable_G())
    {
        assert(false);
        while (1);
    }

    // Setup scale and ODR for IMU
    IMU.Set_G_FS(125);      // Scale = +/- 125 degrees per second
    IMU.Set_G_ODR(416.0f);  // ODR = 416 Hz

    // Setup IMU interrupt for gyro data ready
    IMU.Write_Reg(LSM6DSOX_INT1_CTRL, 0x02);
}

// Core 0 Main Loop
void loop() 
{
    // Local variables
    Euler_t         rads;
    Quaternion_t    q_prime;
    uint8_t         GyroDataRdy = 0;
#ifndef USING_CORE1
    char            str[80];
#endif
    struct
    {
        int32_t wx;
        int32_t wy;
        int32_t wz;
        int32_t data[3];
    } gyro;

    // Check if data is ready for gyro
    IMU.Get_G_DRDY_Status(&GyroDataRdy);
    if (GyroDataRdy)
    {
        // Read the gyro data
        GyroDataRdy = 0;
        IMU.Get_G_Axes(gyro.data);

        gyro.wx = gyro.data[0];
        gyro.wy = gyro.data[1];
        gyro.wz = gyro.data[2];

        // Convert the gyro from degrees to radians
        rads = Quaternion_Degrees2Euler((float)gyro.wx, 
                                        (float)gyro.wy, 
                                        (float)gyro.wz);
        
        // Compute the quaternion derivative
        // See STM's note: DT0060
        q_prime.qw = 0.5 * (-Attitude.qx*rads.roll - Attitude.qy*rads.pitch - Attitude.qz*rads.yaw);
        q_prime.qx = 0.5 * (Attitude.qw*rads.roll - Attitude.qz*rads.pitch + Attitude.qy*rads.yaw);
        q_prime.qy = 0.5 * (Attitude.qz*rads.roll + Attitude.qw*rads.pitch - Attitude.qx*rads.yaw);
        q_prime.qz = 0.5 * (-Attitude.qy*rads.roll + Attitude.qx*rads.pitch + Attitude.qw*rads.yaw);

        // Update the attitude
        // q_prime = Quaternion_ScalarMult(q_prime, 0.002);    // About 2ms for each timestep
        Attitude = Quaternion_Add(Attitude, q_prime);
        Attitude = Quaternion_Norm(Attitude);
#ifdef USING_CORE1
        Core1_FIFO.push(0.0f);
        Core1_FIFO.push(rads.roll);
        Core1_FIFO.push(rads.pitch);
        Core1_FIFO.push(rads.yaw);
#else
        snprintf(str, sizeof(str), "Core0:qw=%.2f,qx=%.2f,qy=%.2f,qz=%.2f",  
                  Attitude.qw, Attitude.qx, Attitude.qy, Attitude.qz);
        Serial.println(str);
#endif
    }
}

// Core 1 Setup
#ifdef USING_CORE1
void setup1() 
{
    // // Start serial
    Serial.begin();
}

// Core 1 Main Loop
void loop1()
{
    // Local variables
    char      str[256];
    float     data[4];
    uint32_t  idx;
    uint8_t   rdy;

    // If there is data in the multicore fifo, get it and send it
    idx = 0;
    while (Core1_FIFO.available())
    {
        data[idx] = Core1_FIFO.popf();
        idx++;
        if (idx >= sizeof(data))
        {
            idx = 0;
        }
        rdy = 1;
    }

    if (rdy)
    {
        rdy = 0;
        snprintf(str, sizeof(str), "qw=%f,qx=%f,qy=%f,qz=%f", data[0], data[1], data[2], data[3]);
        Serial.println(str);
    }
}
#endif

//=============================================================================
// Additional function definitions
//=============================================================================
