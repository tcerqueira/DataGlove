#include <Arduino.h>
#include "Utils/Utils.h"
#include "Core/Hand.h"
#include "Core/Calibration.h"
#include "Drivers/Imu.h"
#include "Drivers/I2CMux.hpp"
#include "Drivers/AnalogSensor.hpp"

#define NUMIMUS 12
#define FRAMETIME_UNLOCK    0
#define FRAMETIME_120FPS    8333
#define FRAMETIME_90FPS     11111
#define FRAMETIME_60FPS     16666
#define FRAMETIME_30FPS     33333
#define FRAMETIME_15FPS     66666

static constexpr uint32_t FRAMETIME         = FRAMETIME_UNLOCK;
static constexpr uint32_t SERIAL_FRAMETIME  = FRAMETIME_30FPS;
static uint32_t delta_serialization;

void output_data();

extern "C" uint32_t set_arm_clock(uint32_t frequency);

// Mpu9250 object
Imu imus[NUMIMUS] = {
    Imu(&Wire, Imu::I2C_ADDR_PRIM, IMU0_GX_OFFSET, IMU0_GY_OFFSET, IMU0_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_SEC, IMU1_GX_OFFSET, IMU1_GY_OFFSET, IMU1_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_PRIM, IMU2_GX_OFFSET, IMU2_GY_OFFSET, IMU2_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_SEC, IMU3_GX_OFFSET, IMU3_GY_OFFSET, IMU3_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_PRIM, IMU4_GX_OFFSET, IMU4_GY_OFFSET, IMU4_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_SEC, IMU5_GX_OFFSET, IMU5_GY_OFFSET, IMU5_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_PRIM, IMU6_GX_OFFSET, IMU6_GY_OFFSET, IMU6_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_SEC, IMU7_GX_OFFSET, IMU7_GY_OFFSET, IMU7_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_PRIM, IMU8_GX_OFFSET, IMU8_GY_OFFSET, IMU8_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_SEC, IMU9_GX_OFFSET, IMU9_GY_OFFSET, IMU9_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_PRIM, IMU10_GX_OFFSET, IMU10_GY_OFFSET, IMU10_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_SEC, IMU11_GX_OFFSET, IMU11_GY_OFFSET, IMU11_GZ_OFFSET)
};
uint8_t mux_map[NUMIMUS] = { 1,1,2,2,3,3,4,4,5,5,0,0 };
uint8_t joint_map[NUMIMUS] = { 0,1,2,3,4,5,7,8,10,11,13,14 };

I2CMux tca9548a(0x70);

Hand hand;

void setup()
{
    // Set clock speed (https://forum.pjrc.com/threads/58688-Teensy-4-0-Clock-speed-influences-delay-and-SPI)
    set_arm_clock(600000000);
    // ############# I2C #############
    // Start the I2C bus
    Wire.begin();
    Wire.setClock(400000);
    // Serial to display data
    Serial.begin(115200);
    // while(!Serial) {};
    // Initialize and configure IMU
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        tca9548a.setChannel(mux_map[i]);
        if(!imus[i].init())
        {
            Serial.print("Error initializing communication with IMU");
            Serial.println(i);
        }
        imus[i].calibrate();
    }

    // Initialize pose
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        tca9548a.setChannel(mux_map[i]);
        while(!imus[i].read());
        double ax = imus[i].accel_x();
        double ay = imus[i].accel_y();
        double az = imus[i].accel_z();
        hand.initializeJoint(i, Eigen::Vector3d(-ax, -ay, -az));
    }
}

void loop()
{
    // Start timing this frame
    Timer frame;
    // Read, filter and process Imu readings
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        tca9548a.setChannel(mux_map[i]);
        imus[i].read();
    }

    // Update hand model
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        if(!imus[i].new_data())
            continue;

        double ex = imus[i].gyro_x();
        double ey = imus[i].gyro_y();
        double ez = imus[i].gyro_z();
        double ax = imus[i].accel_x();
        double ay = imus[i].accel_y();
        double az = imus[i].accel_z();
        hand.updateJoint(joint_map[i], Eigen::Vector3d(ex, ey, ez), Eigen::Vector3d(-ax, -ay, -az));
    }

    // Interpolate each last finger phalange
    for(uint8_t i=1; i < 5; i++)
    {
        // Find the angle between phalange 0 and 1 rotate last phalange by a ratio of that amount
        Finger& finger = hand.getFinger(i);
        Quaternion diff = finger.joints[0].inverse() * finger.joints[1];

        const double angle = 65/115.0 * (-2 * atan2(diff.vec().norm(), diff.w()));
        Quaternion orientation = finger.joints[1] * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());

        finger.joints[2] = orientation;
    }

    // Max serialization frame rate
    uint32_t delta_intermidiate = frame.elapsed_now();
    delta_serialization += delta_intermidiate;
    if(delta_serialization > SERIAL_FRAMETIME)
    {
        // Serialize and send data
        output_data();
        delta_serialization = 0;
    }

    // Max processing frame rate
    uint32_t delta = frame.stop();
    if(delta < FRAMETIME)
        delayMicroseconds(FRAMETIME - delta);
}

void output_data()
{
    String payload;
    hand.serialize(payload);
    Serial.println(payload);
}
