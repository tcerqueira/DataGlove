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
        THUMB = 0, INDEX, MIDDLE, RING, PINKY
    };

public:
    Hand();
    Finger& getFinger(uint8_t index);
    Quaternion& getWrist();
    void serialize(String &outStr);
    void updateJoint(uint8_t index, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel);
    void updateJoint(Quaternion &joint, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel);
    void updateFinger(FingerId id, const Eigen::Vector3d dEulers[], const Eigen::Vector3d accel[]);
    void updateWrist(const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel);

private:
    Quaternion wrist;
    Finger fingers[5];
    StaticJsonDocument<2048> encoded;
};
