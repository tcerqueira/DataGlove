#pragma once

#include <Wire.h>
#include <stdint.h>

class I2CMux
{
public:
    I2CMux(uint8_t i2c_addr);
    void setChannel(uint8_t channel);

private:
    uint8_t i2c_addr;
};

I2CMux::I2CMux(uint8_t i2c_addr)
    : i2c_addr(i2c_addr) {}

void I2CMux::setChannel(uint8_t channel)
{
    Wire.beginTransmission(i2c_addr);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

