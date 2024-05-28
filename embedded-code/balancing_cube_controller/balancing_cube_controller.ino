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
#include "pico/stdlib.h"
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
AttitudeEstimator Attitude = AttitudeEstimator(ATTITUDE_MADGWICK_BETA);

//
// Gyro and accelerometer data
//
struct
{
    float   wx;             // Roll velocity
    float   wy;             // Pitch velocity
    float   wz;             // Yaw velocity
    int32_t xf;             // X offset
    int32_t yf;             // Y offset
    int32_t zf;             // Z offset
    int32_t data[3];
} gyro;

struct
{
    float   ax;
    float   ay;
    float   az;
    int32_t xf;
    int32_t yf;
    int32_t zf;
    int32_t data[3];
} accel;

//
// Timer IRQ structs
//
struct repeating_timer attitude_ctrl_timer;

//
// Flags
//

//=============================================================================
// Additional function prototypes
//=============================================================================

//
// Regular functions
//
void Core0_ProcessFIFO(queue_cmd_t &cmd);
void Core1_ProcessFIFO(queue_cmd_t &cmd);
void CalibrateIMU(void);

//
// ISRs
//
bool AttitudeController_cb(struct repeating_timer *t);

//=============================================================================
// Core functions
//=============================================================================

//
// Core 0 Setup
//
void setup() 
{
    // Local variables
    uint8_t     reg = 0;

#if USING_CORE_1==false
    Serial.begin();
#endif

    // Setup pins
    pinMode(LED_BUILTIN, OUTPUT);

    // Start I2C and LSM6DSOX
    Wire.begin();
    Wire.setClock(400000);
    IMU.begin();

    // Enable block data update unit
    IMU.Read_Reg(LSM6DSOX_CTRL3_C, &reg);
    reg |= 0x40;
    IMU.Write_Reg(LSM6DSOX_CTRL3_C, reg);

    // Start the accel of the IMU
    if (LSM6DSOX_OK != IMU.Enable_X())
    {
        assert(false);
        while (1);
    }

    // Start the gyro of the IMU
    if (LSM6DSOX_OK != IMU.Enable_G())
    {
        assert(false);
        while (1);
    }

    // Setup scale and ODR for IMU
    IMU.Set_X_FS(2);                        // Accel Scale = +/- 2G
    IMU.Set_X_ODR(ATTITUDE_CTRL_FREQ);      // Accel ODR = 52 Hz
    IMU.Set_G_FS(1000);                     // Gyro Scale = +/- 1000 degrees per second
    IMU.Set_G_ODR(ATTITUDE_CTRL_FREQ);      // Gyro ODR = 52 Hz

    // Calibrate the IMU
    CalibrateIMU();

#if USING_CORE_1==true
    // Setup the FIFOs
    Core0_FIFO.begin();
    Core1_FIFO.begin();
#endif

    // Initialize the attitude estimator
    Attitude.begin();

    // Start timer interrupts
    add_repeating_timer_us(ATTITUDE_CTRL_uS, AttitudeController_cb, NULL, &attitude_ctrl_timer);
}

//
// Core 0 Main Loop
//
void loop() 
{
    // Local variables
    Quaternion_t    attitude  = { 1, 0, 0, 0 };
    Euler_t         angles    = { 0, 0, 0 };
#if USING_CORE_1==true
    queue_cmd_t     send_cmd;
    queue_cmd_t     recv_cmd;
#else
    char            str[80];
#endif

    static uint32_t led_timer = millis();
    static uint32_t data_timer = millis();

    if ((millis() - data_timer) >= 100)
    {
        data_timer = millis();

        // Return the attitude
        attitude    = Attitude.GetAttitude();
        angles      = Quaternion_Quat2Euler(attitude);
#if USING_CORE_1==true
        // send_cmd.cmd        = QUEUE_SEND_ATTITUDE;
        // send_cmd.data[0]    = attitude.qw;
        // send_cmd.data[1]    = attitude.qx;
        // send_cmd.data[2]    = attitude.qy;
        // send_cmd.data[3]    = attitude.qz;
        // Core1_FIFO.push(send_cmd);

        send_cmd.cmd        = QUEUE_SEND_EULER;
        send_cmd.data[0]    = angles.roll*RAD_2_DEGREE;
        send_cmd.data[1]    = angles.pitch*RAD_2_DEGREE;
        send_cmd.data[2]    = angles.yaw*RAD_2_DEGREE;
        Core1_FIFO.push(send_cmd);

        // send_cmd.cmd        = QUEUE_SEND_ACCEL;
        // send_cmd.data[0]    = accel.ax;
        // send_cmd.data[1]    = accel.ay;
        // send_cmd.data[2]    = accel.az;
        // Core1_FIFO.push(send_cmd);

        // send_cmd.cmd        = QUEUE_SEND_GYRO;
        // send_cmd.data[0]    = gyro.wx;
        // send_cmd.data[1]    = gyro.wy;
        // send_cmd.data[2]    = gyro.wz;
        // Core1_FIFO.push(send_cmd);

        // send_cmd.cmd        = QUEUE_SEND_LQR_TARGETS;
        // send_cmd.data[0]    = (2*asin(attitude.qx))*RAD_2_DEGREE;
        // send_cmd.data[1]    = (2*asin(attitude.qy))*RAD_2_DEGREE;
        // send_cmd.data[2]    = (2*asin(attitude.qz))*RAD_2_DEGREE;
        // Core1_FIFO.push(send_cmd);
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
// Name:        AttitudeController
// Brief:       Callback for the attitude controller
// Param[in]:   t = The timer running the callback
// Retval:      true for successful completion, false otherwise
//=========================================================================
bool AttitudeController_cb(struct repeating_timer *t)
{
    // Local variables
    uint8_t         gyro_drdy = 0;
    uint8_t         accel_drdy = 0;
    float           gx_new, gy_new, gz_new;

    // Check if data is ready for 
    IMU.Get_X_DRDY_Status(&accel_drdy);
    if (accel_drdy)
    {
        // Read the accelerometer
        IMU.Get_X_Axes(accel.data);

        accel.ax = (float)(accel.data[0]) / 1000.0f;
        accel.ay = (float)(accel.data[1]) / 1000.0f;
        accel.az = (float)(accel.data[2]) / 1000.0f; // - accel.zf;

        // Set the accelerometer quaternion
        Attitude.SetQ_Accel(accel.ax, accel.ay, accel.az);
    }

    // Check if data is ready for gyro
    IMU.Get_G_DRDY_Status(&gyro_drdy);
    if (gyro_drdy)
    {
        // Read the gyro data
        IMU.Get_G_Axes(gyro.data);

        gx_new = (float)(gyro.data[0] - gyro.xf)*DEGREE_2_RAD;
        gy_new = (float)(gyro.data[1] - gyro.yf)*DEGREE_2_RAD;
        gz_new = (float)(gyro.data[2] - gyro.zf)*DEGREE_2_RAD;
        
        // Set the gyro quaternion
        Attitude.SetQ_Gyro(gyro.wx, gyro.wy, gyro.wz);
    }

    // Update the attitude
    Attitude.Estimate(ATTITUDE_CTRL_SECONDS);

    return (true);
}

//=========================================================================
// Name:        Core0_ProcessFIFO
// Brief:       Calibrates the gyro
// Retval:      N.A.
//=========================================================================
void CalibrateIMU(void)
{
    // Local variables
    int32_t     xg_offset = 0;
    int32_t     yg_offset = 0;
    int32_t     zg_offset = 0;
    int32_t     xa_offset = 0;
    int32_t     ya_offset = 0;
    int32_t     za_offset = 0;
    int32_t     data[3];
    uint8_t     gdrdy = 0;
    uint8_t     adrdy = 0;

    // Take 1000 samples, average them;
    for (int16_t idx = 0; idx < 256; idx++)
    {
        // Wait for the gyro to have data
        while (!gdrdy && !adrdy) 
        {
            IMU.Get_X_DRDY_Status(&adrdy);
            IMU.Get_G_DRDY_Status(&gdrdy); 
        }
        IMU.Get_G_Axes(data);
        xg_offset += data[0];
        yg_offset += data[1];
        zg_offset += data[2];
        IMU.Get_X_Axes(data);
        xa_offset += data[0];
        ya_offset += data[1];
        za_offset += data[2];
        gdrdy = 0;
        adrdy = 0;
    }
    xg_offset /= 256;
    yg_offset /= 256;
    zg_offset /= 256;
    xa_offset /= 256;
    ya_offset /= 256;
    za_offset /= 256;

    // Set the offsets for calibration
    gyro.xf     = xg_offset;
    gyro.yf     = yg_offset;
    gyro.zf     = zg_offset;
    accel.xf    = xa_offset;
    accel.yf    = ya_offset;
    accel.zf    = za_offset;
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
            snprintf(str, sizeof(str), "qw:%1.2f,qx:%1.2f,qy:%1.2f,qz:%1.2f",
                    cmd.data[0],
                    cmd.data[1],
                    cmd.data[2],
                    cmd.data[3]);
            Serial.print(str);
            break;
        case QUEUE_SEND_PID_GAIN:
            break;
        case QUEUE_SEND_LQR_GAIN:
            break;
        case QUEUE_SEND_GYRO:
            snprintf(str, sizeof(str), "wx:%1.2f,wy:%1.2f,wz:%1.2f",
                    cmd.data[0],
                    cmd.data[1],
                    cmd.data[2]);
            Serial.print(str);
            break;
        case QUEUE_SEND_ACCEL:
            snprintf(str, sizeof(str), "ax:%1.2f,ay:%1.2f,az:%1.2f",
                    cmd.data[0],
                    cmd.data[1],
                    cmd.data[2]);
            Serial.print(str);
            break;
        case QUEUE_SEND_CAL:
            break;
        case QUEUE_SEND_EULER:
            snprintf(str, sizeof(str), "r:%1.2f,p:%1.2f,y:%1.2f",
                    cmd.data[0],
                    cmd.data[1],
                    cmd.data[2]);
            Serial.print(str);
            break;
        case QUEUE_SEND_LQR_TARGETS:
            snprintf(str, sizeof(str), "t1:%1.2f,t2:%1.2f,t3:%1.2f",
                    cmd.data[0],
                    cmd.data[1],
                    cmd.data[2]);
            Serial.print(str);
            break;
        default:
            break;
    }
    Serial.print("\r\n");
}

//=============================================================================
// End of file
//=============================================================================