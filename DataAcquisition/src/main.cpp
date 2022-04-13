#include <Arduino.h>
// #define DISABLE_MPU9250_FIFO
#include "mpu9250.h"
#include "Hand.h"
#include "Imu.h"

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
    if(imu2.Read())
    {
        
    }

    if(imu.Read())
    {
        Eigen::Vector3d dRot[3];
        double dx = imu.gyro_x_radps();
        double dy = imu.gyro_y_radps();
        double dz = imu.gyro_z_radps();
        dRot[0] = Eigen::Vector3d(dx, dy, dz);
        dRot[1] = Eigen::Vector3d(dx, dy, dz);
        dRot[2] = Eigen::Vector3d(dx, dy, dz);
        hand.updateWrist(dRot[0]);
    }

    String payload;
    hand.serialize(payload);
    Serial.print(payload);
}
