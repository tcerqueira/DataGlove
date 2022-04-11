#include "Hand.h"
#include "ArduinoJson.hpp"

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
    serializeJson(encoded, outStr);
    outStr += '\n';
}