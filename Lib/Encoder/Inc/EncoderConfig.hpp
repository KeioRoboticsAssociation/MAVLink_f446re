#pragma once

#include "Encoder.hpp"
#include <cstdint>
#include <cstring>

class EncoderConfigParser {
public:
    static EncoderStatus parseFromJson(const char* jsonString, EncoderConfig& config);
    static EncoderStatus parseFromJsonForId(const char* jsonString, uint8_t encoderId, EncoderConfig& config);
    static EncoderStatus loadFromFile(const char* filePath, EncoderConfig& config);
    static EncoderStatus loadFromFileForId(const char* filePath, uint8_t encoderId, EncoderConfig& config);
    static EncoderStatus saveToFile(const char* filePath, const EncoderConfig& config);
    
private:
    static bool parseUint16(const char* json, const char* key, uint16_t& value);
    static bool parseUint32(const char* json, const char* key, uint32_t& value);
    static bool parseInt32(const char* json, const char* key, int32_t& value);
    static bool parseBool(const char* json, const char* key, bool& value);
    static bool parseEncoderMode(const char* json, const char* key, EncoderMode& value);
    
    static const char* findKey(const char* json, const char* key);
    static const char* findEncoderSection(const char* json, uint8_t encoderId);
    static bool parseValue(const char* valueStr, uint16_t& value);
    static bool parseValue(const char* valueStr, uint32_t& value);
    static bool parseValue(const char* valueStr, int32_t& value);
    static bool parseValue(const char* valueStr, bool& value);
    static void skipWhitespace(const char*& ptr);
    static const char* findValueEnd(const char* valueStart);
    static const char* findSectionEnd(const char* sectionStart);
};

struct EncoderConfigDefaults {
    static const EncoderConfig STANDARD_ENCODER;
    static const EncoderConfig HIGH_RESOLUTION_ENCODER;
    static const EncoderConfig ABSOLUTE_ENCODER;
};