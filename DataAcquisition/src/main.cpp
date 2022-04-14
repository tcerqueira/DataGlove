#include <Arduino.h>
// #define DISABLE_MPU9250_FIFO
#include "mpu9250.h"
#include "Hand.h"
#include "Imu.h"

#define NUMIMUS 2

// extern "C" uint32_t set_arm_clock(uint32_t frequency);

// /* Mpu9250 object, SPI bus, CS on pin 10 */
// bfs::Mpu9250 imu(&SPI, 10);
// bfs::Mpu9250 imu2(&SPI, 9);

/* Mpu9250 object */
bfs::Mpu9250 imus[NUMIMUS] = {
    bfs::Mpu9250(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM),
    bfs::Mpu9250(&Wire, bfs::Mpu9250::I2C_ADDR_SEC)
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
        Serial.print("Initializing IMU-");
        Serial.println(i);
        if(!imus[i].Begin())
        {
            Serial.print("Error initializing communication with IMU");
            while(1) {}
        }
        /* Set the sample rate divider */
        if(!imus[i].ConfigSrd(19))
        {
            Serial.println("Error configured SRD");
            while(1) {}
        }
    }
}

void loop() {
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        imus[i].Read();
    }
    
    if(imus[0].new_imu_data())
    {
        // https://www.youtube.com/watch?v=CHSYgLfhwUo
        Eigen::Vector3d dRot[3];
        double dx = imus[0].gyro_x_radps() * .05;
        double dy = imus[0].gyro_y_radps() * .05;
        double dz = imus[0].gyro_z_radps() * .05;
        dRot[0] = Eigen::Vector3d(dx, dy, dz);
        dRot[1] = Eigen::Vector3d(dx, dy, dz);
        dRot[2] = Eigen::Vector3d(dx, dy, dz);
        hand.updateWrist(dRot[0]);
    }

    String payload;
    hand.serialize(payload);
    Serial.print(payload);
}

