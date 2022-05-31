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

Quaternion& Hand::getJoint(uint8_t index)
{
    if(index == 0)
        return wrist;

    uint8_t finger = (index - 1) / 3;
    uint8_t joint = (index - 1) % 3;
    return fingers[finger].joints[joint];
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
        // Relative rotation between finger and wrist
        Quaternion wrist_diff = wrist.inverse() * fingers[i].joints[0];
        jointsArr[0]["x"] = wrist_diff.x();
        jointsArr[0]["y"] = wrist_diff.y();
        jointsArr[0]["z"] = wrist_diff.z();
        jointsArr[0]["w"] = wrist_diff.w();

        for(uint8_t j=1; j < 3; j++)
        {
            // Relative rotation between finger joints
            Quaternion diff = fingers[i].joints[j-1].inverse() * fingers[i].joints[j];
            jointsArr[j]["x"] = diff.x();
            jointsArr[j]["y"] = diff.y();
            jointsArr[j]["z"] = diff.z();
            jointsArr[j]["w"] = diff.w();
        }
    }

    serializeJson(encoded, outStr);
    encoded["debug"] = "";
    outStr += '\n';
}

void Hand::debug(const String &str)
{
    encoded["debug"] = str;
}

void Hand::updateJoint(uint8_t index, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel)
{
    updateJoint(getJoint(index), Quaternion(dEuler), accel);
}

void Hand::updateJoint(Quaternion &joint, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel)
{
    updateJoint(joint, Quaternion(dEuler), accel);
}

void Hand::updateJoint(uint8_t index, const Quaternion &rot, const Eigen::Vector3d &accel)
{
    updateJoint(getJoint(index), rot, accel);
}

void Hand::updateJoint(Quaternion &joint, const Quaternion &rot, const Eigen::Vector3d &accel)
{
    // https://ahrs.readthedocs.io/en/latest/filters/angular.html
    joint *= rot;
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
