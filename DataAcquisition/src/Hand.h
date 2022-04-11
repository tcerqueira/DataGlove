#pragma once
#include "Quaternion.hpp"
#include "ArduinoJson.hpp"
#include <stdint.h>

struct Finger {
    Quaternion joints[3];
};

class Hand
{
public:
    Hand();
    Finger& getFinger(uint8_t index);
    Quaternion& getWrist();
    void serialize(String &outStr);

private:
    Quaternion wrist;
    Finger fingers[5];
    StaticJsonDocument<4096> encoded;
};
