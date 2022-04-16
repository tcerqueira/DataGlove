#include "Hand.h"

// using json = nlohmann::json;

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

Eigen::Vector3d& Hand::getWrist()
{
    return wrist;
}

void Hand::serialize(String &outStr)
{
    // Quaternion wrist_q = Quaternion::fromEuler(wrist);
    Quaternion &wrist_q = wrist_qt;
    encoded["wrist"]["x"] = wrist_q.x();
    encoded["wrist"]["y"] = wrist_q.y();
    encoded["wrist"]["z"] = wrist_q.z();
    encoded["wrist"]["w"] = wrist_q.w();

    JsonArray fingersArr = encoded["fingers"];
    for(uint8_t i=0; i < 5; i++)
    {
        JsonArray jointsArr = fingersArr[i]["joints"];
        for(uint8_t j=0; j < 3; j++)
        {
            Quaternion joint_q = Quaternion(fingers[i].joints[j]);
            jointsArr[j]["x"] = joint_q.x();
            jointsArr[j]["y"] = joint_q.y();
            jointsArr[j]["z"] = joint_q.z();
            jointsArr[j]["w"] = joint_q.w();
        }
    }
    serializeJson(encoded, outStr);
    outStr += '\n';
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
    wrist_qt *= Quaternion(dEuler);
    wrist += dEuler;
}
