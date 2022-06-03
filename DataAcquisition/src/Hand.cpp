#include "Hand.h"

Quaternion orientationFromGravity(const Eigen::Vector3d &gravity);

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

Quaternion orientationFromGravity(const Eigen::Vector3d &gravity)
{
    // https://youtu.be/CHSYgLfhwUo?t=1427
    double ax = gravity.y();
    double ay = gravity.x();
    double az = -gravity.z();
    double ex = atan2(-1 * ax, sqrt(ay*ay + az*az));
    double ey = atan2(ay, az);
    double ez = 0;

    return Quaternion(ex, ey, ez);

    // Eigen::Vector3d down(0, 0, -1);
    // Quaternion q = Quaternion::FromTwoVectors(down, gravity);

    // return q;

    // double w, x, y, z;
    // double ax = gravity.x();
    // double ay = gravity.y();
    // double az = -gravity.z();
    // if(az >= 0)
    // {
    //     w = sqrt((az + 1) / 2);
    //     x = - ay / sqrt(2 * (az + 1));
    //     y = ax / sqrt(2 * (az + 1));
    //     z = 0;
    // }
    // else {
    //     w = - ay / sqrt(2 * (1 - az));
    //     x = sqrt((1 - az) / 2);
    //     y = 0;
    //     z = az / sqrt(2 * (1 - az));
    // }
    
    // return Quaternion(x, z, y, w);

}

void Hand::initializeJoint(uint8_t index, const Eigen::Vector3d &gravity)
{
    initializeJoint(getJoint(index), gravity);
}

void Hand::initializeJoint(Quaternion &joint, const Eigen::Vector3d &gravity)
{
    joint = orientationFromGravity(gravity);
}

void Hand::updateJoint(Quaternion &joint, const Quaternion &rot, const Eigen::Vector3d &accel)
{
    // https://ahrs.readthedocs.io/en/latest/filters/angular.html
    Quaternion gyro_q = joint * rot;
    Quaternion accel_q = orientationFromGravity(accel);

    joint = gyro_q * GYRO_PART + accel_q * ACCEL_PART;
    joint.normalize();
    // joint *= rot;
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

void Hand::updateFinger(FingerId id, const Eigen::Vector3d dEulers[], const Eigen::Vector3d accel[])
{
    Finger &finger = fingers[id];
    for(uint8_t i = 0; i < 3; i++)
    {
        updateJoint(finger.joints[i], dEulers[i], accel[i]);
    } 
}

void Hand::updateWrist(const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel)
{
    updateJoint(wrist, dEuler, accel);
}
