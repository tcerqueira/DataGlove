#pragma once
#include <stdint.h>
#include "Timer.hpp"
#include "mpu9250.h"

// Wrapper class for IMU
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
    void calibrate();
    bool read();
    double accel_x() { return accel_mps2[0]; }
    double accel_y() { return accel_mps2[1]; }
    double accel_z() { return accel_mps2[2]; }
    double gyro_x()  { return gyro_drad[0];  }
    double gyro_y()  { return gyro_drad[1];  }
    double gyro_z()  { return gyro_drad[2];  }
    bool new_data()  { return recv_new;      }

private:
    void push_accel_buffer(float ax, float ay, float az);
    
public:
    static constexpr uint8_t I2C_ADDR_PRIM = 0x68;
    static constexpr uint8_t I2C_ADDR_SEC = 0x69;
    static constexpr uint8_t accel_buffer_len = 12;

private:
    bfs::Mpu9250 imu;
    Protocol protocol;
    double accel_mps2[3];
    double gyro_drad[3];
    double accel_offset[3] = {};
    double gyro_offset[3] = {};
    float accel_buffer[3][accel_buffer_len] = {{},{},{}};
    uint8_t accel_buffer_index = 0;
    bool recv_new = false;
    uint32_t delta_us = 1000;
    Timer timer;
};