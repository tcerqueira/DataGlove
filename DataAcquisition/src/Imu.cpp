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
    return true;
}

void Imu::read()
{
    uint32_t elapsed = timer.stop();
    elapsed = 1000;


    timer.start();
}
