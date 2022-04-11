#include <Arduino.h>
// #define DISABLE_MPU9250_FIFO
#include "mpu9250.h"
#include "Hand.h"

extern "C" uint32_t set_arm_clock(uint32_t frequency);

/* Mpu9250 object, SPI bus, CS on pin 10 */
bfs::Mpu9250 imu(&SPI, 10);
bfs::Mpu9250 imu2(&SPI, 9);
Hand hand;

void setup()
{
    // Set clock speed (https://forum.pjrc.com/threads/58688-Teensy-4-0-Clock-speed-influences-delay-and-SPI)
    set_arm_clock(396000000);
    /* Serial to display data */
    Serial.begin(115200);
    while(!Serial) {}
    /* Start the SPI bus */
    SPI.begin();
    /* Initialize and configure IMU */
    if(!imu.Begin())
    {
        Serial.println("Error initializing communication with IMU1");
        while(1) {}
    }
    /* Set the sample rate divider */
    if(!imu.ConfigSrd(19))
    {
        Serial.println("Error configured SRD");
        while(1) {}
    }
    if(!imu2.Begin())
    {
        Serial.println("Error initializing communication with IMU2");
        while(1) {}
    }
    /* Set the sample rate divider */
    if(!imu2.ConfigSrd(19))
    {
        Serial.println("Error configured SRD");
        while(1) {}
    }
}

void loop()
{
    
    String str;
    hand.serialize(str);
    Serial.print(str);
    if(imu2.Read())
    {
        // Serial.print("IMU 2:");
        // Serial.print("\t");
        // Serial.print(imu2.accel_x_mps2());
        // Serial.print("\t");
        // Serial.print(imu2.accel_y_mps2());
        // Serial.print("\t");
        // Serial.print(imu2.accel_z_mps2());
        // Serial.print("\t");
        // Serial.print(imu2.gyro_x_radps());
        // Serial.print("\t");
        // Serial.print(imu2.gyro_y_radps());
        // Serial.print("\t");
        // Serial.print(imu2.gyro_z_radps());
        // Serial.print("\t");
        // Serial.print(imu2.mag_x_ut());
        // Serial.print("\t");
        // Serial.print(imu2.mag_y_ut());
        // Serial.print("\t");
        // Serial.print(imu2.mag_z_ut());
        // Serial.print("\t");
        // Serial.print(imu2.die_temp_c());
        // Serial.print("\n");
    }

    if(imu.Read())
    {
        // Serial.print("IMU 1:");
        // Serial.print("\t");
        // Serial.print(imu.accel_x_mps2());
        // Serial.print("\t");
        // Serial.print(imu.accel_y_mps2());
        // Serial.print("\t");
        // Serial.print(imu.accel_z_mps2());
        // Serial.print("\t");
        // Serial.print(imu.gyro_x_radps());
        // Serial.print("\t");
        // Serial.print(imu.gyro_y_radps());
        // Serial.print("\t");
        // Serial.print(imu.gyro_z_radps());
        // Serial.print("\t");
        // Serial.print(imu.mag_x_ut());
        // Serial.print("\t");
        // Serial.print(imu.mag_y_ut());
        // Serial.print("\t");
        // Serial.print(imu.mag_z_ut());
        // Serial.print("\t");
        // Serial.print(imu.die_temp_c());
        // Serial.print("\n");
    }
}
