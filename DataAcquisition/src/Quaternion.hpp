#pragma once
#include "ArduinoJson.hpp"

struct Quaternion
{
    float x, y, z, w;

    JsonObject serialize(JsonObject parent)
    {
        JsonObject obj = parent.createNestedObject("Quaternion");
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

    template<unsigned int C>
    JsonObject serialize(StaticJsonDocument<C> parent)
    {
        JsonObject obj = parent["wrist"];
        obj["x"] = x;
        obj["y"] = y;
        obj["z"] = z;
        obj["w"] = w;
        return obj;
    }
};
