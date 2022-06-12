#include <Arduino.h>
#include "Utils/Utils.h"
#include "Core/Calibration.h"
#include "Drivers/Imu.h"
#include "Drivers/I2CMux.hpp"

#define NUMIMUS 12

void offline_calibration(uint8_t i);

extern "C" uint32_t set_arm_clock(uint32_t frequency);

// Mpu9250 object
Imu imus[NUMIMUS] = {
    Imu(&Wire, Imu::I2C_ADDR_PRIM, IMU0_GX_OFFSET, IMU0_GY_OFFSET, IMU0_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_SEC, IMU1_GX_OFFSET, IMU1_GY_OFFSET, IMU1_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_PRIM, IMU2_GX_OFFSET, IMU2_GY_OFFSET, IMU2_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_SEC, IMU3_GX_OFFSET, IMU3_GY_OFFSET, IMU3_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_PRIM, IMU4_GX_OFFSET, IMU4_GY_OFFSET, IMU4_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_PRIM, IMU5_GX_OFFSET, IMU5_GY_OFFSET, IMU5_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_SEC, IMU6_GX_OFFSET, IMU6_GY_OFFSET, IMU6_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_PRIM, IMU7_GX_OFFSET, IMU7_GY_OFFSET, IMU7_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_SEC, IMU8_GX_OFFSET, IMU8_GY_OFFSET, IMU8_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_SEC, IMU9_GX_OFFSET, IMU9_GY_OFFSET, IMU9_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_PRIM, IMU10_GX_OFFSET, IMU10_GY_OFFSET, IMU10_GZ_OFFSET),
    Imu(&Wire, Imu::I2C_ADDR_SEC, IMU11_GX_OFFSET, IMU11_GY_OFFSET, IMU11_GZ_OFFSET)
};
uint8_t mux_map[NUMIMUS] = { 1,1,2,2,3,3,4,4,5,5,0,0 };
uint8_t joint_map[NUMIMUS] = { 0,1,2,3,4,5,7,8,10,11,13,14 };

I2CMux tca9548a(0x70);

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
    while(!Serial) {};
    // Initialize and configure IMU
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        tca9548a.setChannel(mux_map[i]);
        if(!imus[i].init())
            Serial.print("Error initializing communication with IMU");
    }

    // Offline calibration
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        offline_calibration(i);
    }
    Serial.println('{');
}

void loop()
{  
}

void offline_calibration(uint8_t i)
{
    tca9548a.setChannel(mux_map[i]);
    for(uint32_t j=0; j < 100000; j++)
    {
        while(!imus[i].read());

        static constexpr int df = 15;
        Serial.print(i);
        Serial.print(",");
        Serial.print(j);
        Serial.print(",");
        Serial.print(imus[i].raw_gyro_x(), df);
        Serial.print(",");
        Serial.print(imus[i].raw_gyro_y(), df);
        Serial.print(",");
        Serial.print(imus[i].raw_gyro_z(), df);
        Serial.print(",");
        Serial.print(imus[i].raw_accel_x(), df);
        Serial.print(",");
        Serial.print(imus[i].raw_accel_y(), df);
        Serial.print(",");
        Serial.println(imus[i].raw_accel_z(), df);
    }
}
