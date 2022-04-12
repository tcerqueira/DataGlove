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
