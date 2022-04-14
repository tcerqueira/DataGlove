#include "Hand.h"

// using json = nlohmann::json;

Hand::Hand()
{
    // Wrist JSON
    JsonObject wristObj = encoded.createNestedObject("wrist");
    wrist.serialize(wristObj);
    // Fingers JSON
    JsonArray fingersArr = encoded.createNestedArray("fingers");
    for(uint8_t i=0; i < 5; i++)
    {
        JsonObject fingerObj = fingersArr.createNestedObject();
        JsonArray jointsArr = fingerObj.createNestedArray("joints");
        for(uint8_t j=0; j < 3; j++)
        {
            fingers[i].joints[j].serialize(jointsArr);
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
            jointsArr[j]["x"] = fingers[i].joints[j].x();
            jointsArr[j]["y"] = fingers[i].joints[j].y();
            jointsArr[j]["z"] = fingers[i].joints[j].z();
            jointsArr[j]["w"] = fingers[i].joints[j].w();
        }
    }
    serializeJson(encoded, outStr);
    outStr += '\n';
}

void Hand::updateFinger(FingerId id, const Eigen::Vector3d dRotations[])
{
    Quaternion q = Eigen::AngleAxisd(dRotations[0].x(), Eigen::Vector3d::UnitX())
                   * Eigen::AngleAxisd(dRotations[0].y(), Eigen::Vector3d::UnitY())
                   * Eigen::AngleAxisd(dRotations[0].z(), Eigen::Vector3d::UnitZ());

    fingers[id].joints[0] *= q;
}

void Hand::updateWrist(const Eigen::Vector3d &dRotation)
{
    Quaternion q = Eigen::AngleAxisd(dRotation.x(), Eigen::Vector3d::UnitX())
                   * Eigen::AngleAxisd(dRotation.y(), Eigen::Vector3d::UnitY())
                   * Eigen::AngleAxisd(dRotation.z(), Eigen::Vector3d::UnitZ());
    
    wrist *= q;
}
