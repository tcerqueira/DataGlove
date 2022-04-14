#pragma once
#include "ArduinoJson.hpp"
#include <Eigen/Geometry>
#include <Eigen/Dense>

class Quaternion : public Eigen::Quaterniond
{
public:
    Quaternion()
    : Eigen::Quaterniond(1,0,0,0)
    {
    }

    Quaternion(float x, float y, float z, float w)
    : Eigen::Quaterniond(w,x,y,z)
    {
    }

    // This constructor allows you to construct Quaternion from Eigen expressions
    Quaternion(const Eigen::Quaterniond& other)
    : Eigen::Quaterniond(other)
    {
    }

    // This method allows you to assign Eigen expressions to Quaternion
    Quaternion& operator=(const Eigen::Quaterniond& other)
    {
        this->Eigen::Quaterniond::operator=(other);
        return *this;
    }

    inline static Quaternion fromEuler(const Eigen::Vector3d &euler)
    {
        return Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX())
               * Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ());
    }

    inline static Quaternion fromEuler(double x, double y, double z)
    {
        return Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX())
               * Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY())
               * Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ());
    }

    JsonObject serialize(JsonObject obj)
    {
        obj["x"] = x();
        obj["y"] = y();
        obj["z"] = z();
        obj["w"] = w();
        return obj;
    }

    JsonObject serialize(JsonArray parent)
    {
        JsonObject obj = parent.createNestedObject();
        obj["x"] = x();
        obj["y"] = y();
        obj["z"] = z();
        obj["w"] = w();
        return obj;
    }

    template<unsigned int Capacity>
    JsonObject serialize(StaticJsonDocument<Capacity> parent)
    {
        JsonObject obj = parent.createNestedObject();
        obj["x"] = x();
        obj["y"] = y();
        obj["z"] = z();
        obj["w"] = w();
        return obj;
    }
};
