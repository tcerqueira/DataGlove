#include "Hand.h"
#include "Utils/Utils.h"

static constexpr float GRAVITY = 9.807f;

static Quaternion orientationFromGravity(const Eigen::Vector3d &gravity);
static Eigen::Vector3d anglesFromGravity(const Eigen::Vector3d &gravity);

Hand::Hand()
{
    for(uint8_t i=0; i < 16; i++)
        joints[i] = Quaternion();
    // Wrist JSON
    JsonObject wristObj = encoded.createNestedObject("wrist");
    pose.wrist.serialize(wristObj);
    // Fingers JSON
    JsonArray fingersArr = encoded.createNestedArray("fingers");
    for(uint8_t i=0; i < 5; i++)
    {
        JsonObject fingerObj = fingersArr.createNestedObject();
        JsonArray jointsArr = fingerObj.createNestedArray("joints");
        for(uint8_t j=0; j < 3; j++)
        {
            pose.fingers[i].joints[j].serialize(jointsArr);
        }
    }
}

Finger& Hand::getFinger(uint8_t index)
{
    return pose.fingers[index];
}

Quaternion& Hand::getWrist()
{
    return pose.wrist;
}

Quaternion& Hand::getJoint(uint8_t index)
{
    return joints[index];
}

void Hand::serialize(String &outStr)
{
    encoded["wrist"]["x"] = pose.wrist.x();
    encoded["wrist"]["y"] = pose.wrist.y();
    encoded["wrist"]["z"] = pose.wrist.z();
    encoded["wrist"]["w"] = pose.wrist.w();

    JsonArray fingersArr = encoded["fingers"];
    for(uint8_t i=0; i < 5; i++)
    {
        JsonArray jointsArr = fingersArr[i]["joints"];
        Quaternion wrist_diff = pose.wrist.inverse() * pose.fingers[i].joints[0];
        jointsArr[0]["x"] = wrist_diff.x();
        jointsArr[0]["y"] = wrist_diff.y();
        jointsArr[0]["z"] = wrist_diff.z();
        jointsArr[0]["w"] = wrist_diff.w();

        for(uint8_t j=1; j < 3; j++)
        {
            // Relative rotation between finger joints
            Quaternion diff = pose.fingers[i].joints[j-1].inverse() * pose.fingers[i].joints[j];
            jointsArr[j]["x"] = diff.x();
            jointsArr[j]["y"] = diff.y();
            jointsArr[j]["z"] = diff.z();
            jointsArr[j]["w"] = diff.w();
        }
    }

    serializeJson(encoded, outStr);
    encoded["debug"] = "";
}

void Hand::debug(const String &str)
{
    encoded["debug"] = str;
}

inline Quaternion orientationFromGravity(const Eigen::Vector3d &gravity)
{
    return Quaternion(anglesFromGravity(gravity));
}

inline Eigen::Vector3d anglesFromGravity(const Eigen::Vector3d &gravity)
{
    // https://youtu.be/CHSYgLfhwUo?t=1427
    double ax = gravity.x();
    double ay = gravity.y();
    double az = gravity.z();
    double ex = atan2(ay, az);
    double ey = atan2(-1 * ax, sqrt(ay*ay + az*az));
    double ez = 0;

    return Eigen::Vector3d(ex, ey, ez);
}

void Hand::initializeJoint(uint8_t index, const Eigen::Vector3d &gravity)
{
    initializeJoint(joints[index], gravity);
}

void Hand::initializeJoint(Quaternion &joint, const Eigen::Vector3d &gravity)
{
    joint = orientationFromGravity(gravity);
}

void Hand::updateJoint(Quaternion &joint, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel)
{
    // https://ahrs.readthedocs.io/en/latest/filters.html
    // https://www.mdpi.com/1424-8220/15/8/19302/htm
    const float em = abs_tp(accel.norm() - GRAVITY) / GRAVITY;
    float gain_factor;
    if(em <= ERROR_T1)
        gain_factor = 1;
    else if(em >= ERROR_T2)
        gain_factor = 0;
    else {
        gain_factor = (ERROR_T2 - em) / ERROR_T1;
    }
    const float ADAPTIVE_GAIN = STATIC_GAIN * gain_factor;

    Quaternion qw = Quaternion(dEuler);
    Quaternion predicted_w = joint * qw;
    Eigen::Vector3d predicted_g = predicted_w * accel;

    Quaternion dqacc = Quaternion::FromTwoVectors(predicted_g, Eigen::Vector3d(0, 0, 1));
    dqacc = Quaternion() * (1-ADAPTIVE_GAIN) + dqacc * ADAPTIVE_GAIN;
    dqacc.normalize();

    joint = predicted_w * dqacc;

    // joint *= Quaternion(dEuler);

    // Quaternion &q = joint;
    // const Eigen::Vector3d &e = dEuler;
    // double w = q.w() - e.x()/2 * q.x() - e.y()/2 * q.y() - e.z()/2 * q.z();
    // double x = q.x() + e.x()/2 * q.w() - e.y()/2 * q.z() + e.z()/2 * q.y();
    // double y = q.y() + e.x()/2 * q.z() + e.y()/2 * q.w() - e.z()/2 * q.x();
    // double z = q.z() - e.x()/2 * q.y() + e.y()/2 * q.x() + e.z()/2 * q.w();
    // Quaternion qw = Quaternion(w, x, y, z);

    // Eigen::Vector3d acc_angles = anglesFromGravity(accel);
    // double cy = cos(acc_angles.z() * 0.5);
    // double sy = sin(acc_angles.z() * 0.5);
    // double cp = cos(acc_angles.y() * 0.5);
    // double sp = sin(acc_angles.y() * 0.5);
    // double cr = cos(acc_angles.z() * 0.5);
    // double sr = sin(acc_angles.z() * 0.5);

    // Quaternion qacc;
    // q.w() = cr * cp * cy + sr * sp * sy;
    // q.x() = sr * cp * cy - cr * sp * sy;
    // q.y() = cr * sp * cy + sr * cp * sy;
    // q.z() = cr * cp * sy - sr * sp * cy;
    // Quaternion qacc = Quaternion(acc_angles);


    // Quaternion gyro_q = joint * Quaternion(dEuler);
    // Quaternion accel_q = orientationFromGravity(accel);

    // joint = gyro_q * (1-ADAPTIVE_GAIN) + accel_q * ADAPTIVE_GAIN;
    // joint.normalize();

    // Eigen::Vector3d e = joint.eulerAngles(2, 0, 1);
    // Eigen::Vector3d a = Quaternion(anglesFromGravity(accel)).eulerAngles(2, 0, 1);

    // double ex = (e.x() + dEuler.x()) * (1-ADAPTIVE_GAIN) + a.x() * ADAPTIVE_GAIN;
    // double ey = (e.y() + dEuler.y()) * (1-ADAPTIVE_GAIN) + a.y() * ADAPTIVE_GAIN;
    // double ez = e.z() + dEuler.z();
    // std::stringstream ss; ss << ez;
    // if(&joint == &pose.wrist)
    //     debug(ss.str().c_str());

    // joint = Quaternion(ex, ey, ez);
    // joint = Eigen::AngleAxisd(ez, Eigen::Vector3d::UnitZ())
    //         * Eigen::AngleAxisd(ex, Eigen::Vector3d::UnitX())
    //         * Eigen::AngleAxisd(ey, Eigen::Vector3d::UnitY());

    // double w, x, y, z;
    // Eigen::Vector3d g = accel.normalized();
    // double ax = g.y();
    // double ay = g.x();
    // double az = -g.z();
    // if(az >= 0)
    // {
    //     w = sqrt((az + 1) / 2);
    //     x = ax / sqrt(2 * (az + 1));
    //     y = - ay / sqrt(2 * (az + 1));
    //     z = 0;
    // }
    // else {
    //     w = - ay / sqrt(2 * (1 - az));
    //     x = 0;
    //     y = sqrt((1 - az) / 2);
    //     z = -az / sqrt(2 * (1 - az));
    // }

    // Quaternion dqacc(x, y, z, w);
    // Quaternion dw = Quaternion(dEuler);
    // Eigen::Vector3d down = Eigen::Vector3d(0, 0, -1);
    // Eigen::Vector3d previous_acc = joint * down;
    // Eigen::Vector3d predicted_acc = orientationFromGravity(accel) * down;
    // Quaternion dqacc = Quaternion::FromTwoVectors(previous_acc.normalized(), predicted_acc.normalized());
    // dqacc.normalize();
    // Quaternion gyro_q = joint * Quaternion(dEuler);
    // Quaternion accel_q = joint * dqacc;

    // joint = gyro_q * (1-ADAPTIVE_GAIN) + accel_q * ADAPTIVE_GAIN;
    // joint.normalize();

    // Quaternion gyro_q = joint * Quaternion(dEuler);
    // Quaternion gravity_q = orientationFromGravity(accel);
    // const double yaw = gyro_q.eulerAngles(2, 0, 1)[0];
    // // const double yaw = atan2(2.0*(gyro_q.y*gyro_q.z + gyro_q.w*gyro_q.x),
    // //                         gyro_q.w*gyro_q.w - gyro_q.x*gyro_q.x - gyro_q.y*gyro_q.y + gyro_q.z*gyro_q.z);
    // Quaternion accel_q = gravity_q * Quaternion(0, 0, yaw);

    // // std::stringstream ss; ss << yaw;
    // // if(&joint == &pose.wrist)
    // //     debug(ss.str().c_str());

    // joint = gyro_q * (1-ADAPTIVE_GAIN) + accel_q * ADAPTIVE_GAIN;
    // joint.normalize();
}

void Hand::updateJoint(Quaternion &joint, const Quaternion &rot, const Eigen::Vector3d &accel)
{
    updateJoint(joint, rot.eulerAngles(), accel);
}

void Hand::updateJoint(uint8_t index, const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel)
{
    updateJoint(joints[index], dEuler, accel);
}

void Hand::updateJoint(uint8_t index, const Quaternion &rot, const Eigen::Vector3d &accel)
{
    updateJoint(joints[index], rot.eulerAngles(), accel);
}

void Hand::updateFinger(FingerId id, const Eigen::Vector3d dEulers[], const Eigen::Vector3d accel[])
{
    Finger &finger = pose.fingers[id];
    for(uint8_t i = 0; i < 3; i++)
    {
        updateJoint(finger.joints[i], dEulers[i], accel[i]);
    } 
}

void Hand::updateWrist(const Eigen::Vector3d &dEuler, const Eigen::Vector3d &accel)
{
    updateJoint(pose.wrist, dEuler, accel);
}
