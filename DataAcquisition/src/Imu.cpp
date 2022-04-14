#include "Imu.h"

Imu::Imu(SPIClass *spi, const uint8_t cs)
    : imu(spi, cs), protocol(Imu::Protocol::SPI)
{
}

Imu::Imu(TwoWire *i2c, const uint8_t addr)
    : imu(i2c, (bfs::Mpu9250::I2cAddr)addr), protocol(Imu::Protocol::I2C)
{
}


bool Imu::init()
{
    if(!imu.Begin())
    {
        Serial.print("Error initializing communication with IMU");
        return false;
    }
    /* Set the sample rate divider */
    if(!imu.ConfigSrd(19))
    {
        Serial.println("Error configured SRD");
        return false;
    }
    timer.start();
    return true;
}

void Imu::calibrate()
{
    uint16_t cycles = 15;
    for(uint16_t i=0; i < cycles;)
    {
        if(!imu.Read())
            continue;
        
        i++;
    }
}

void Imu::read() // https://www.youtube.com/watch?v=CHSYgLfhwUo
{
    delta_us = timer.stop();
    recv_new = imu.Read();

    accel_mps2[0] = imu.accel_x_mps2() * delta_us / 1000000.0;
    accel_mps2[1] = imu.accel_y_mps2() * delta_us / 1000000.0;
    accel_mps2[2] = imu.accel_z_mps2() * delta_us / 1000000.0;
    gyro_drad[0]  = imu.gyro_x_radps() * delta_us / 1000000.0;
    gyro_drad[1]  = imu.gyro_y_radps() * delta_us / 1000000.0;
    gyro_drad[2]  = imu.gyro_z_radps() * delta_us / 1000000.0;

    timer.start();
}
