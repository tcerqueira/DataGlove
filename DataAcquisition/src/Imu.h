#pragma once
#include <stdint.h>
#include "Timer.hpp"
#include "mpu9250.h"

class Imu
{
public:
    enum class Protocol : uint8_t
    {
        SPI,
        I2C
    };

    Imu(SPIClass *spi, const uint8_t cs);
    Imu(TwoWire *i2c, const uint8_t addr);

    bool init();
    void read();
    double accel_x() { return accel_mps2[0]; }
    double accel_y() { return accel_mps2[1]; }
    double accel_z() { return accel_mps2[2]; }
    double gyro_x()  { return gyro_drad[0]; }
    double gyro_y()  { return gyro_drad[1]; }
    double gyro_z()  { return gyro_drad[2]; }
    
private:
    bfs::Mpu9250 imu;
    Protocol protocol;
    double accel_mps2[3];
    double gyro_drad[3];
    uint32_t dt_us = 1000;
    Timer timer;

public:
    static constexpr uint8_t I2C_ADDR_PRIM = 0x68;
    static constexpr uint8_t I2C_ADDR_SEC = 0x69;
};