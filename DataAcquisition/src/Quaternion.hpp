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

    Quaternion(double x, double y, double z, double w)
    : Eigen::Quaterniond(w,x,y,z)
    {
    }

    // This constructor allows you to construct Quaternion from Eigen expressions
    Quaternion(const Eigen::Quaterniond& other)
    : Eigen::Quaterniond(other)
    {
    }

    Quaternion(const Eigen::Vector3d &euler)
    : Eigen::Quaterniond(Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()))
    {
    }

    Quaternion(double ex, double ey, double ez)
    : Eigen::Quaterniond(Eigen::AngleAxisd(ex, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(ey, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(ez, Eigen::Vector3d::UnitZ()))
    {
    }

    // This method allows you to assign Eigen expressions to Quaternion
    Quaternion& operator=(const Eigen::Quaterniond& other)
    {
        this->Eigen::Quaterniond::operator=(other);
        return *this;
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
