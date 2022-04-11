#pragma once
#include "ArduinoJson.hpp"

struct Quaternion
{
    float x, y, z, w;

    JsonObject serialize(JsonObject obj)
    {
        obj["x"] = x;
        obj["y"] = y;
        obj["z"] = z;
        obj["w"] = w;
        return obj;
    }

    JsonObject serialize(JsonArray parent)
    {
        JsonObject obj = parent.createNestedObject();
        obj["x"] = x;
        obj["y"] = y;
        obj["z"] = z;
        obj["w"] = w;
        return obj;
    }

    template<unsigned int Capacity>
    JsonObject serialize(StaticJsonDocument<Capacity> parent)
    {
        JsonObject obj = parent.createNestedObject();
        obj["x"] = x;
        obj["y"] = y;
        obj["z"] = z;
        obj["w"] = w;
        return obj;
    }
};
