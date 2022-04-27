#include "Hand.h"

Hand::Hand()
{
    // Wrist JSON
    JsonObject wristObj = encoded.createNestedObject("wrist");
    Quaternion(wrist).serialize(wristObj);
    // Fingers JSON
    JsonArray fingersArr = encoded.createNestedArray("fingers");
    for(uint8_t i=0; i < 5; i++)
    {
        JsonObject fingerObj = fingersArr.createNestedObject();
        JsonArray jointsArr = fingerObj.createNestedArray("joints");
        for(uint8_t j=0; j < 3; j++)
        {
            Quaternion(fingers[i].joints[j]).serialize(jointsArr);
        }
    }
}

Finger& Hand::getFinger(uint8_t index)
{
    return fingers[index];
}

Quaternion& Hand::getWrist()
{
    return wrist;
}

void Hand::serialize(String &outStr)
{
    // Quaternion wrist_q = Quaternion::fromEuler(wrist);
    encoded["wrist"]["x"] = wrist.x();
    encoded["wrist"]["y"] = wrist.y();
    encoded["wrist"]["z"] = wrist.z();
    encoded["wrist"]["w"] = wrist.w();

    JsonArray fingersArr = encoded["fingers"];
    for(uint8_t i=0; i < 5; i++)
    {
        JsonArray jointsArr = fingersArr[i]["joints"];
        for(uint8_t j=0; j < 3; j++)
        {
            const Quaternion &joint = fingers[i].joints[j];
            jointsArr[j]["x"] = joint.x();
            jointsArr[j]["y"] = joint.y();
            jointsArr[j]["z"] = joint.z();
            jointsArr[j]["w"] = joint.w();
        }
    }
    serializeJson(encoded, outStr);
    outStr += '\n';
}

void Hand::updateJoint(uint8_t index, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel)
{
    if(index == 0)
    {
        updateWrist(dEuler, accel);
        return;
    }

    uint8_t finger = (index - 1) / 3;
    uint8_t joint = (index - 1) % 3;
    updateJoint(fingers[finger].joints[joint], dEuler, accel);
}

void Hand::updateJoint(Quaternion &joint, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel)
{
    joint *= Quaternion(dEuler);
}

void Hand::updateFinger(FingerId id, const Eigen::Vector3d dEulers[], const Eigen::Vector3d accel[])
{
    // Quaternion q = Eigen::AngleAxisd(dRotations[0].x(), Eigen::Vector3d::UnitX())
    //                * Eigen::AngleAxisd(dRotations[0].y(), Eigen::Vector3d::UnitY())
    //                * Eigen::AngleAxisd(dRotations[0].z(), Eigen::Vector3d::UnitZ());

    // fingers[id].joints[0] *= q;
}

void Hand::updateWrist(const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel)
{
    updateJoint(wrist, dEuler, accel);
}
