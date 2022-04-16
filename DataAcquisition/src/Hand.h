#pragma once
#include "Quaternion.hpp"
#include "ArduinoJson.hpp"
#include <stdint.h>

struct Finger {
    Eigen::Vector3d joints[3];
};

struct Finger_qt {
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
    Eigen::Vector3d& getWrist();
    void serialize(String &outStr);
    void updateFinger(FingerId id, const Eigen::Vector3d dEulers[], const Eigen::Vector3d accel[]);
    void updateWrist(const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel);

private:
    Eigen::Vector3d wrist;
    Quaternion wrist_qt;
    Finger fingers[5];
    Finger_qt fingers_qt[5];
    StaticJsonDocument<4096> encoded;
};
