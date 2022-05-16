#include <Arduino.h>
#include "Hand.h"
#include "Drivers/Imu.h"
#include "Drivers/I2CMux.hpp"
#include "Drivers/AnalogSensor.hpp"

#define NUMIMUS 3
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
    Imu(&Wire, Imu::I2C_ADDR_SEC),
    Imu(&Wire, Imu::I2C_ADDR_PRIM)
};
uint8_t mux_map[NUMIMUS] = { 1,1,2 };
uint8_t joint_map[NUMIMUS] = { 4,0,5 };

I2CMux tca9548a(0x70);
AnalogSensor<double> flex(14, 0, 1024, 0.0, 1.0);

Hand hand;

void setup()
{
    // ############# SPI #############
    // Set clock speed (https://forum.pjrc.com/threads/58688-Teensy-4-0-Clock-speed-influences-delay-and-SPI)
    // set_arm_clock(396000000);
    /* Start the SPI bus */
    // SPI.begin();
    // ############# I2C #############
    /* Start the I2C bus */
    Wire.begin();
    Wire.setClock(400000);
    /* Serial to display data */
    Serial.begin(115200);
    while(!Serial) {}
    /* Initialize and configure IMU */
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        tca9548a.setChannel(mux_map[i]);
        imus[i].init();
        imus[i].calibrate();
    }

    // Initialize pose
    // init_hand();
    // double in = flex.read();
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
        hand.updateJoint(joint_map[i], Eigen::Vector3d(ey, ez, ex), Eigen::Vector3d(ay, az, ax));
    }

    // Interpolate each last finger phalange
    // const uint8_t tip_joints[] = { 6 };
    // const uint8_t interpolation_imus[] = { 1 };
    // for(uint8_t i=0; i < 1; i++)
    // {
    //     // FIX ME !!
    //     Imu &imu = imus[interpolation_imus[i]];
    //     Imu &imu_prev = imus[interpolation_imus[i] - 1];
    //     const double angle = 115.0/65 * (imu.gyro_y() - imu_prev.gyro_y());
    //     hand.updateJoint(tip_joints[i], Eigen::Vector3d(angle, 0, 0), Eigen::Vector3d(angle, 0, 0));
    // }

    // Interpolate each last finger phalange
    for(uint8_t i=1; i < 5; i++)
    {
        // Find the angle between phalange 0 and 1 rotate last phalange by a ratio of that amount
        Finger& finger = hand.getFinger(i);
        Quaternion diff = finger.joints[0].inverse() * finger.joints[1];
        const double angle = 65/115.0 * diff.eulerAngles().x();
        Quaternion rotation = finger.joints[1] * Eigen::AngleAxisd(angle < 0 ? 0 : angle, Eigen::Vector3d::UnitX());
        finger.joints[2] = rotation;
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

