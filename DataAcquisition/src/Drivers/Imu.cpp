#include "Imu.h"
#include <algorithm>
#include "Utils.h"

#define GRAVITY 9.807

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
    constexpr const uint16_t cycles = calib_cycles;
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
        push_accel_buffer(ax[i], ay[i], az[i]);
        i++;
    }

    // Eigen::Vector3d accel_mean(
    //     mean(ax, cycles),
    //     mean(ay, cycles),
    //     mean(az, cycles)
    // );
    // const double accel_ratio = accel_mean.norm() / GRAVITY;
    // accel_offset[0] = (accel_mean.x() * accel_ratio) - accel_mean.x();
    // accel_offset[1] = (accel_mean.y() * accel_ratio) - accel_mean.y();
    // accel_offset[2] = (accel_mean.z() * accel_ratio) - accel_mean.z();
    accel_offset[0] = 0;
    accel_offset[1] = 0;
    accel_offset[2] = 0;
    gyro_offset[0] = mean(gx, cycles);
    gyro_offset[1] = mean(gy, cycles);
    gyro_offset[2] = mean(gz, cycles);
}

bool Imu::read() // https://www.youtube.com/watch?v=CHSYgLfhwUo
{
    delta_us = timer.stop();
    recv_new = imu.Read();
    if(!recv_new)
        return recv_new;

    float ax_filt, ay_filt, az_filt;

    push_accel_buffer(imu.accel_x_mps2(), imu.accel_y_mps2(), imu.accel_z_mps2());
    ax_filt = median(accel_buffer[0], Imu::accel_buffer_len);
    ay_filt = median(accel_buffer[1], Imu::accel_buffer_len);
    az_filt = median(accel_buffer[2], Imu::accel_buffer_len);
    accel_mps2[0] = (ax_filt - accel_offset[0]) * delta_us / 1000000.0;
    accel_mps2[1] = (ay_filt - accel_offset[1]) * delta_us / 1000000.0;
    accel_mps2[2] = (az_filt - accel_offset[2]) * delta_us / 1000000.0;
    gyro_drad[0]  = (imu.gyro_x_radps() - gyro_offset[0]) * delta_us / 1000000.0;
    gyro_drad[1]  = (imu.gyro_y_radps() - gyro_offset[1]) * delta_us / 1000000.0;
    gyro_drad[2]  = (imu.gyro_z_radps() - gyro_offset[2]) * delta_us / 1000000.0;

    timer.start();
    return recv_new;
}

void Imu::push_accel_buffer(float ax, float ay, float az)
{
    accel_buffer[0][accel_buffer_index] = ax;
    accel_buffer[1][accel_buffer_index] = ay;
    accel_buffer[2][accel_buffer_index] = az;
    accel_buffer_index = (accel_buffer_index + 1) % Imu::accel_buffer_len;
}
