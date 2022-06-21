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

    static constexpr float STATIC_GAIN = 1-.995f;
    static constexpr float ERROR_T1 = .1f;
    static constexpr float ERROR_T2 = .2f;

public:
    Hand();
    Finger& getFinger(uint8_t index);
    Quaternion& getWrist();
    Quaternion& getJoint(uint8_t index);

    void serialize(String &outStr);

    void initializeJoint(uint8_t index, const Eigen::Vector3d &gravity);
    void initializeJoint(Quaternion &joint, const Eigen::Vector3d &gravity);

    void updateFinger(FingerId id, const Eigen::Vector3d dEuler[], const Eigen::Vector3d gravity[]);
    void updateWrist(const Eigen::Vector3d &dEuler, const Eigen::Vector3d &gravity);
    
    void updateJoint(uint8_t index, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &gravity);
    void updateJoint(Quaternion &joint, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &gravity);

private:
    struct Pose
    {
        Quaternion wrist;
        Finger fingers[5];
    };

    union
    {
        Pose pose;
        Quaternion joints[16];
    };
    StaticJsonDocument<2048> encoded;
};
