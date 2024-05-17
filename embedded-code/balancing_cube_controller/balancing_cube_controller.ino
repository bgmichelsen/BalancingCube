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
#include "src/attitude/AttitudeEstimator.h"
#include "LSM6DSOXSensor.h"
#include "Wire.h"
#include "PinDefs.h"
#include "Definitions.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>

//=============================================================================
// Objects / RAM
//=============================================================================

//
// IMU data
//
LSM6DSOXSensor IMU = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);

//
// Queue data
//
#if USING_CORE_1==true
QueueFIFO Core0_FIFO = QueueFIFO(8);
QueueFIFO Core1_FIFO = QueueFIFO(8);
#endif

//
// Attitude data
//
AttitudeEstimator Attitude = AttitudeEstimator();

//
// Gyro and accelerometer data
//
struct
{
    int32_t wx;             // Roll velocity
    int32_t wy;             // Pitch velocity
    int32_t wz;             // Yaw velocity
    int32_t xf;             // X offset
    int32_t yf;             // Y offset
    int32_t zf;             // Z offset
    int32_t data[3];
} gyro;

struct
{
    int32_t ax;
    int32_t ay;
    int32_t az;
    int32_t data[3];
} accel;

//
// Flags
//

//=============================================================================
// Additional function prototypes
//=============================================================================

void Core0_ProcessFIFO(queue_cmd_t &cmd);
void Core1_ProcessFIFO(queue_cmd_t &cmd);
void CalibrateGyro(void);

//=============================================================================
// Core functions
//=============================================================================

//
// Core 0 Setup
//
void setup() 
{
    // Local variables

#if USING_CORE_1==false
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
    IMU.Set_G_FS(2000);      // Scale = +/- 2000 degrees per second
    IMU.Set_G_ODR(416.0f);  // ODR = 416 Hz

    // Setup IMU interrupt for gyro data ready
    // IMU.Write_Reg(LSM6DSOX_INT1_CTRL, 0x02);

    // Calibrate the IMU
    CalibrateGyro();

#if USING_CORE_1==true
    // Setup the FIFOs
    Core0_FIFO.begin();
    Core1_FIFO.begin();
#endif

    // Initialize the attitude estimator
    Attitude.begin();
}

//
// Core 0 Main Loop
//
void loop() 
{
    // Local variables
    Euler_t         rads;
    Quaternion_t    attitude = { 1, 0, 0, 0 };
    uint8_t         gyro_drdy = 0;
#if USING_CORE_1==true
    queue_cmd_t     send_cmd;
    queue_cmd_t     recv_cmd;
#else
    char            str[80];
#endif

    static uint32_t led_timer = millis();

    // Check if data is ready for gyro
    IMU.Get_G_DRDY_Status(&gyro_drdy);
    if (gyro_drdy)
    {
        // Read the gyro data
        gyro_drdy = 0;
        IMU.Get_G_Axes(gyro.data);

        gyro.wx = gyro.data[0]; //- gyro.xf;
        gyro.wy = gyro.data[1]; //- gyro.yf;
        gyro.wz = gyro.data[2]; //- gyro.zf;

        // Convert the gyro from degrees to radians
        rads = Quaternion_Degrees2Euler((float)gyro.wx, 
                                        (float)gyro.wy, 
                                        (float)gyro.wz);
        
        // Set the gyro quaternion
        Attitude.SetQ_Gyro(rads.roll, rads.pitch, rads.yaw, 0.002);

        // Update the attitude
        Attitude.Estimate();

        // Return the attitude
        attitude = Attitude.GetAttitude();

#if USING_CORE_1==true
        send_cmd.cmd        = QUEUE_SEND_ATTITUDE;
        send_cmd.data[0]    = attitude.qw;
        send_cmd.data[1]    = attitude.qx;
        send_cmd.data[2]    = attitude.qy;
        send_cmd.data[3]    = attitude.qz;
        Core1_FIFO.push(send_cmd);
#endif
    }

    // Blink the LED
    if ((millis() - led_timer) >= 1000)
    {
        led_timer = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

#if USING_CORE_1==true
//
// Core 1 Setup
//
void setup1() 
{
    // // Start serial
    Serial.begin();
}

//
// Core 1 Main Loop
//
void loop1()
{
    // Local variables
    queue_cmd_t send_cmd;
    queue_cmd_t recv_cmd;


    // If there is data in the multicore fifo, get it and send it
    if (Core1_FIFO.available())
    {
        // Pop the command, then process it
        recv_cmd = Core1_FIFO.pop();
        Core1_ProcessFIFO(recv_cmd);
    }
}
#endif

//=============================================================================
// Additional function definitions
//=============================================================================

//=========================================================================
// Name:        Core0_ProcessFIFO
// Brief:       Calibrates the gyro
// Retval:      N.A.
//=========================================================================
void CalibrateGyro(void)
{
    // Local variables
    int32_t     x_offset = 0;
    int32_t     y_offset = 0;
    int32_t     z_offset = 0;
    int32_t     data[3];
    uint8_t     drdy;

    // Take 1000 samples, average them;
    for (int16_t idx = 0; idx < 10000; idx++)
    {
        // Wait for the gyro to have data
        while (!drdy) 
        { 
            IMU.Get_G_DRDY_Status(&drdy); 
        }
        IMU.Get_G_Axes(data);
        x_offset += data[0];
        y_offset += data[1];
        z_offset += data[2];
        drdy = 0;
    }
    x_offset /= 10000;
    y_offset /= 10000;
    z_offset /= 10000;

    // Set the offsets for calibration
    gyro.xf = x_offset;
    gyro.yf = y_offset;
    gyro.zf = z_offset;
}

//=========================================================================
// Name:        Core0_ProcessFIFO
// Brief:       Processes a FIFO command coming into Core 0
// Param[in]:   cmd = The command to process
// Retval:      N.A.
//=========================================================================
void Core0_ProcessFIFO(queue_cmd_t &cmd)
{
    // Process the command
    switch (cmd.cmd)
    {
        case QUEUE_SEND_ATTITUDE:
            break;
        case QUEUE_SEND_PID_GAIN:
            break;
        case QUEUE_SEND_LQR_GAIN:
            break;
        case QUEUE_SEND_GYRO:
            break;
        case QUEUE_SEND_ACCEL:
            break;
        case QUEUE_SEND_CAL:
            break;
        default:
            break;  
    }
}

//=========================================================================
// Name:        Core0_ProcessFIFO
// Brief:       Processes a FIFO command coming into Core 1
// Param[in]:   cmd = The command to process
// Retval:      N.A.
//=========================================================================
void Core1_ProcessFIFO(queue_cmd_t &cmd)
{
    // Local variables
    char    str[256];

    // Process the command
    switch (cmd.cmd)
    {
        case QUEUE_SEND_ATTITUDE:
            snprintf(str, sizeof(str), "qw=%1.2f,qx=%1.2f,qy=%1.2f,qz=%1.2f",
                    cmd.data[0],
                    cmd.data[1],
                    cmd.data[2],
                    cmd.data[3]);
            Serial.println(str);
            break;
        case QUEUE_SEND_PID_GAIN:
            break;
        case QUEUE_SEND_LQR_GAIN:
            break;
        case QUEUE_SEND_GYRO:
            break;
        case QUEUE_SEND_ACCEL:
            break;
        case QUEUE_SEND_CAL:
            break;
        default:
            break;  
    }
}

//=============================================================================
// End of file
//=============================================================================