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

#include "LSM6DSOXSensor.h"
#include "Quaternion.h"
#include "Wire.h"
#include "PinDefs.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

//=============================================================================
// Objects / RAM
//=============================================================================

// IMU data
LSM6DSOXSensor  IMU = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

// Control data
Quaternion_t  Attitude = { 1, 0, 0, 0 };

// Flags
uint8_t GyroDataRdy = 0;

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
    Euler_t       rads;
    Quaternion_t  q_prime;
    union
    {
        int32_t data[3];
        int32_t wx;
        int32_t wy;
        int32_t wz;
    } gyro;

    // Check if data is ready for gyro
    IMU.Get_G_DRDY_Status(&GyroDataRdy);
    if (GyroDataRdy)
    {
        // Read the gyro data
        GyroDataRdy = 0;
        IMU.Get_G_Axes(gyro.data);

        // Convert the gyro from degrees to radians
        rads = Quaternion_Degrees2Euler((float)gyro.wx, 
                                        (float)gyro.wy, 
                                        (float)gyro.wz);
        
        // Compute the quaternion derivative
        // See STM's note: DT0060
        q_prime.qw = (-Attitude.qx*rads.roll - Attitude.qy*rads.pitch - Attitude.qz*rads.yaw);
        q_prime.qx = (Attitude.qw*rads.roll - Attitude.qz*rads.pitch + Attitude.qy*rads.yaw);
        q_prime.qy = (Attitude.qz*rads.roll + Attitude.qw*rads.pitch - Attitude.qx*rads.yaw);
        q_prime.qz = (-Attitude.qy*rads.roll + Attitude.qx*rads.pitch + Attitude.qw*rads.yaw);
        q_prime = Quaternion_ScalarMult(q_prime, 0.5);

        // Update the attitude
        q_prime = Quaternion_ScalarMult(q_prime, 0.002);    // About 2ms for each timestep
        Attitude = Quaternion_Add(Attitude, q_prime);
        Attitude = Quaternion_Norm(Attitude);

        rp2040.fifo.push(Attitude.qw);
        rp2040.fifo.push(Attitude.qx);
        rp2040.fifo.push(Attitude.qy);
        rp2040.fifo.push(Attitude.qz);
    }
}

// Core 1 Setup
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
    while (rp2040.fifo.available())
    {
        data[idx] = (float)rp2040.fifo.pop();
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
        snprintf(str, sizeof(str), "qw=%0.2f,qx=%0.2f,qy=%0.2f,qz=%0.2f", data[0], data[1], data[2], data[3]);
        Serial.println(str);
    }
}

//=============================================================================
// Additional function definitions
//=============================================================================
