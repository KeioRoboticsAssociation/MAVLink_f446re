#pragma once

#include "ServoMotor.hpp"
#include <cstdint>
#include <cstring>

class ServoConfigParser {
public:
    static ServoStatus parseFromJson(const char* jsonString, ServoConfig& config);
    static ServoStatus parseFromJsonForId(const char* jsonString, uint8_t servoId, ServoConfig& config);
    static ServoStatus loadFromFile(const char* filePath, ServoConfig& config);
    static ServoStatus loadFromFileForId(const char* filePath, uint8_t servoId, ServoConfig& config);
    static ServoStatus saveToFile(const char* filePath, const ServoConfig& config);
    
private:
    static bool parseFloat(const char* json, const char* key, float& value);
    static bool parseUint16(const char* json, const char* key, uint16_t& value);
    static bool parseUint32(const char* json, const char* key, uint32_t& value);
    static bool parseBool(const char* json, const char* key, bool& value);
    static bool parseFailSafeBehavior(const char* json, const char* key, FailSafeBehavior& value);
    
    static const char* findKey(const char* json, const char* key);
    static const char* findServoSection(const char* json, uint8_t servoId);
    static bool parseValue(const char* valueStr, float& value);
    static bool parseValue(const char* valueStr, uint16_t& value);
    static bool parseValue(const char* valueStr, uint32_t& value);
    static bool parseValue(const char* valueStr, bool& value);
    static void skipWhitespace(const char*& ptr);
    static const char* findValueEnd(const char* valueStart);
    static const char* findSectionEnd(const char* sectionStart);
};

struct ServoConfigDefaults {
    static const ServoConfig STANDARD_SERVO;
    static const ServoConfig HIGH_TORQUE_SERVO;
    static const ServoConfig CONTINUOUS_ROTATION_SERVO;
    static const ServoConfig DIGITAL_SERVO;
};