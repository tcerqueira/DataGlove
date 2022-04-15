#include "Imu.h"

static double mean(float array[], uint32_t len);

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
    constexpr const uint16_t cycles = 50;
    float gx[cycles], gy[cycles], gz[cycles];
    float ax[cycles], ay[cycles], az[cycles];

    for(uint16_t i=0; i < cycles;)
    {
        if(!imu.Read())
            continue;
        ax[i] = imu.accel_x_mps2();
        ay[i] = imu.accel_y_mps2();
        az[i] = imu.accel_z_mps2();
        gx[i] = imu.gyro_x_radps();
        gy[i] = imu.gyro_y_radps();
        gz[i] = imu.gyro_z_radps();
        i++;
    }
    gyro_offset[0] = mean(gx, cycles);
    gyro_offset[1] = mean(gy, cycles);
    gyro_offset[2] = mean(gz, cycles);
}

void Imu::read() // https://www.youtube.com/watch?v=CHSYgLfhwUo
{
    delta_us = timer.stop();
    recv_new = imu.Read();

    accel_mps2[0] = imu.accel_x_mps2() * delta_us / 1000000.0;
    accel_mps2[1] = imu.accel_y_mps2() * delta_us / 1000000.0;
    accel_mps2[2] = imu.accel_z_mps2() * delta_us / 1000000.0;
    gyro_drad[0]  = (imu.gyro_x_radps() - gyro_offset[0]) * delta_us / 1000000.0;
    gyro_drad[1]  = (imu.gyro_y_radps() - gyro_offset[1]) * delta_us / 1000000.0;
    gyro_drad[2]  = (imu.gyro_z_radps() - gyro_offset[2]) * delta_us / 1000000.0;

    timer.start();
}

static double mean(float array[], uint32_t len)
{
    double sum = 0.0;
    for(uint32_t i=0; i < len; i++)
        sum += (double)array[i];

    return sum / len;
}
