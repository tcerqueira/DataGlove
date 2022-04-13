#pragma once
#include <stdint.h>

class Imu
{
public:
    enum class Protocol
    {
        SPI,
        I2C
    };

    void read();
    double accel_x();
    double accel_y();
    double accel_z();
    double gyro_x();
    double gyro_y();
    double gyro_z();
    
private:
    uint16_t port;
    Protocol protocol;
    double accel_mps2[3];
    double gyro_drad[3];
    uint32_t dt_us;
};