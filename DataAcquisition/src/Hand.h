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
    enum FingerId {
        THUMB = 0,
        INDEX,
        MIDDLE,
        RING,
        PINKY
    };
public:
    Hand();
    Finger& getFinger(uint8_t index);
    Quaternion& getWrist();
    void serialize(String &outStr);
    void updateFinger(FingerId id, const Eigen::Vector3d dRotations[]);
    void updateWrist(const Eigen::Vector3d &dRotation);

private:
    Quaternion wrist;
    Finger fingers[5];
    StaticJsonDocument<4096> encoded;
};
