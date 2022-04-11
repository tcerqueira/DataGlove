#include "Hand.h"
#include "ArduinoJson.hpp"

// using json = nlohmann::json;

Hand::Hand()
{
    wrist.serialize(encoded);
    JsonArray fingersArr = encoded.createNestedArray("fingers");
    for(uint8_t i=0; i < 5; i++)
    {
        JsonObject fingerObj = fingersArr.createNestedObject();
        JsonArray jointsArr = fingerObj.createNestedArray("joints");
        fingersArr.add(fingerObj);
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
    outStr = serializeJson(encoded, Serial);
}