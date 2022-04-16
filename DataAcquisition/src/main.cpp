#include <Arduino.h>
// #define DISABLE_MPU9250_FIFO
#include "mpu9250.h"
#include "Hand.h"
#include "Imu.h"

#define NUMIMUS 2
#define FRAMETIME_60FPS 16666
#define FRAMETIME_30FPS 33333

static const constexpr uint32_t FRAMETIME = FRAMETIME_60FPS;

// extern "C" uint32_t set_arm_clock(uint32_t frequency);

// /* Mpu9250 object, SPI bus, CS on pin 10 */
// Imu imus[NUMIMUS] = {
//     Imu(&SPI, 10),
//     Imu(&SPI, 9)
// };

/* Mpu9250 object */
Imu imus[NUMIMUS] = {
    Imu(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM),
    Imu(&Wire, bfs::Mpu9250::I2C_ADDR_SEC)
};
Hand hand;

void setup() {
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
}

void loop() {
    Timer frame;
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        imus[i].read();
    }
    
    if(imus[0].new_data())
    {
        double dx = imus[0].gyro_x();
        double dy = imus[0].gyro_y();
        double dz = imus[0].gyro_z();
        hand.updateWrist(Eigen::Vector3d(dy, dz, dx), Eigen::Vector3d(dy, dz, dx));
    }

    String payload;
    hand.serialize(payload);
    Serial.print(payload);

    uint32_t delta = frame.stop();
    if(delta < FRAMETIME)
        delayMicroseconds(FRAMETIME - delta);
}

