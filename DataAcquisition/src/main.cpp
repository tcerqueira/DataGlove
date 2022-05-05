#include <Arduino.h>
#include "Hand.h"
#include "Imu.h"
#include "I2CMux.hpp"

#define NUMIMUS 2
#define FRAMETIME_60FPS 16666
#define FRAMETIME_30FPS 33333

static constexpr uint32_t FRAMETIME = FRAMETIME_60FPS;

void init_hand();
void output_data();

// extern "C" uint32_t set_arm_clock(uint32_t frequency);

// /* Mpu9250 object, SPI bus, CS on pin 10 */
// Imu imus[NUMIMUS] = {
//     Imu(&SPI, 10),
//     Imu(&SPI, 9)
// };

// Mpu9250 object
Imu imus[NUMIMUS] = {
    Imu(&Wire, Imu::I2C_ADDR_PRIM),
    Imu(&Wire, Imu::I2C_ADDR_SEC)
};
uint8_t mux_channels[NUMIMUS] = { 0,0 };
I2CMux tca9548a(0x70);
Hand hand;

void setup()
{
    /* Serial to display data */
    Serial.begin(115200);
    while(!Serial) {}
    // ############# SPI #############
    // Set clock speed (https://forum.pjrc.com/threads/58688-Teensy-4-0-Clock-speed-influences-delay-and-SPI)
    // set_arm_clock(396000000);
    /* Start the SPI bus */
    // SPI.begin();
    // ############# I2C #############
    /* Start the I2C bus */
    Wire.begin();
    Wire.setClock(400000);
    /* Initialize and configure IMU */
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        imus[i].init();
        imus[i].calibrate();
    }

    // Initialize pose
    // init_hand();
}

void loop()
{
    // Start timing this frame
    Timer frame;
    // Read, filter and process Imu readings
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        tca9548a.setChannel(mux_channels[i]);
        imus[i].read();
    }

    // Update hand model
    // for(uint8_t i=0; i < NUMIMUS; i++)
    // {
    //     if(!imus[0].new_data())
    //         continue;

    //     double ex = imus[i].gyro_x();
    //     double ey = imus[i].gyro_y();
    //     double ez = imus[i].gyro_z();
    //     double ax = imus[i].accel_x();
    //     double ay = imus[i].accel_y();
    //     double az = imus[i].accel_z();
    //     hand.updateJoint(i, Eigen::Vector3d(ey, ez, ex), Eigen::Vector3d(ay, az, ax));
    // }
    
    if(imus[0].new_data())
    {
        double dx = imus[0].gyro_x();
        double dy = imus[0].gyro_y();
        double dz = imus[0].gyro_z();
        hand.updateWrist(Eigen::Vector3d(dy, dz, dx), Eigen::Vector3d(dy, dz, dx));
    }

    if(imus[1].new_data())
    {
        double dx = imus[1].gyro_x();
        double dy = imus[1].gyro_y();
        double dz = imus[1].gyro_z();
        hand.updateJoint(4, Eigen::Vector3d(dy, dz, dx), Eigen::Vector3d(dy, dz, dx));
    }

    // Serialize and send data
    output_data();

    // Max frame rate
    uint32_t delta = frame.stop();
    if(delta < FRAMETIME)
        delayMicroseconds(FRAMETIME - delta);
}

void init_hand()
{
    // NEEDS FIX !!
    while(!imus[0].read());
    double ax = imus[0].accel_x();
    double ay = imus[0].accel_y();
    double az = imus[0].accel_z();
    double ex = atan2(az, ay);
    double ey = atan2(-1 * ax, sqrt(ay*ay + az*az));
    double ez = 0;
    hand.updateWrist(Eigen::Vector3d(ey, ez, ex), Eigen::Vector3d(ey, ez, ex));

    output_data();
}

void output_data()
{
    String payload;
    hand.serialize(payload);
    Serial.print(payload);
}

