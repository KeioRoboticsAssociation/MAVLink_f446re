#include "EncoderConfig.hpp"
#include <cstdlib>
#include <cstdio>

const EncoderConfig EncoderConfigDefaults::STANDARD_ENCODER = {
    .cpr = 1024,
    .invertA = false,
    .invertB = false,
    .useZ = true,
    .mode = EncoderMode::TIM_ENCODER_MODE,
    .watchdogTimeoutMs = 500,
    .offsetCounts = 0,
    .wrapAround = true
};

const EncoderConfig EncoderConfigDefaults::HIGH_RESOLUTION_ENCODER = {
    .cpr = 4096,
    .invertA = false,
    .invertB = false,
    .useZ = true,
    .mode = EncoderMode::TIM_ENCODER_MODE,
    .watchdogTimeoutMs = 200,
    .offsetCounts = 0,
    .wrapAround = true
};

const EncoderConfig EncoderConfigDefaults::ABSOLUTE_ENCODER = {
    .cpr = 8192,
    .invertA = false,
    .invertB = false,
    .useZ = false,
    .mode = EncoderMode::TIM_ENCODER_MODE,
    .watchdogTimeoutMs = 1000,
    .offsetCounts = 0,
    .wrapAround = false
};

EncoderStatus EncoderConfigParser::parseFromJson(const char* jsonString, EncoderConfig& config) {
    if (jsonString == nullptr) {
        return EncoderStatus::CONFIG_ERROR;
    }

    config = EncoderConfigDefaults::STANDARD_ENCODER;

    parseUint16(jsonString, "cpr", config.cpr);
    parseBool(jsonString, "invertA", config.invertA);
    parseBool(jsonString, "invertB", config.invertB);
    parseBool(jsonString, "useZ", config.useZ);
    parseEncoderMode(jsonString, "mode", config.mode);
    parseUint32(jsonString, "watchdogTimeoutMs", config.watchdogTimeoutMs);
    parseInt32(jsonString, "offsetCounts", config.offsetCounts);
    parseBool(jsonString, "wrapAround", config.wrapAround);

    if (config.cpr == 0 || config.watchdogTimeoutMs == 0) {
        return EncoderStatus::CONFIG_ERROR;
    }

    return EncoderStatus::OK;
}

EncoderStatus EncoderConfigParser::parseFromJsonForId(const char* jsonString, uint8_t encoderId, EncoderConfig& config) {
    if (jsonString == nullptr) {
        return EncoderStatus::CONFIG_ERROR;
    }

    const char* encoderSection = findEncoderSection(jsonString, encoderId);
    if (encoderSection == nullptr) {
        return EncoderStatus::CONFIG_ERROR;
    }

    config = EncoderConfigDefaults::STANDARD_ENCODER;

    parseUint16(encoderSection, "cpr", config.cpr);
    parseBool(encoderSection, "invertA", config.invertA);
    parseBool(encoderSection, "invertB", config.invertB);
    parseBool(encoderSection, "useZ", config.useZ);
    parseEncoderMode(encoderSection, "mode", config.mode);
    parseUint32(encoderSection, "watchdogTimeoutMs", config.watchdogTimeoutMs);
    parseInt32(encoderSection, "offsetCounts", config.offsetCounts);
    parseBool(encoderSection, "wrapAround", config.wrapAround);

    if (config.cpr == 0 || config.watchdogTimeoutMs == 0) {
        return EncoderStatus::CONFIG_ERROR;
    }

    return EncoderStatus::OK;
}

EncoderStatus EncoderConfigParser::loadFromFile(const char* filePath, EncoderConfig& config) {
    if (filePath == nullptr) {
        return EncoderStatus::CONFIG_ERROR;
    }

    FILE* file = fopen(filePath, "r");
    if (file == nullptr) {
        return EncoderStatus::CONFIG_ERROR;
    }

    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (fileSize <= 0 || fileSize > 2048) {
        fclose(file);
        return EncoderStatus::CONFIG_ERROR;
    }

    char* jsonBuffer = new char[fileSize + 1];
    size_t bytesRead = fread(jsonBuffer, 1, fileSize, file);
    fclose(file);

    if (bytesRead != static_cast<size_t>(fileSize)) {
        delete[] jsonBuffer;
        return EncoderStatus::CONFIG_ERROR;
    }

    jsonBuffer[fileSize] = '\0';

    EncoderStatus result = parseFromJson(jsonBuffer, config);
    delete[] jsonBuffer;

    return result;
}

EncoderStatus EncoderConfigParser::loadFromFileForId(const char* filePath, uint8_t encoderId, EncoderConfig& config) {
    if (filePath == nullptr) {
        return EncoderStatus::CONFIG_ERROR;
    }

    FILE* file = fopen(filePath, "r");
    if (file == nullptr) {
        return EncoderStatus::CONFIG_ERROR;
    }

    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (fileSize <= 0 || fileSize > 2048) {
        fclose(file);
        return EncoderStatus::CONFIG_ERROR;
    }

    char* jsonBuffer = new char[fileSize + 1];
    size_t bytesRead = fread(jsonBuffer, 1, fileSize, file);
    fclose(file);

    if (bytesRead != static_cast<size_t>(fileSize)) {
        delete[] jsonBuffer;
        return EncoderStatus::CONFIG_ERROR;
    }

    jsonBuffer[fileSize] = '\0';

    EncoderStatus result = parseFromJsonForId(jsonBuffer, encoderId, config);
    delete[] jsonBuffer;

    return result;
}

EncoderStatus EncoderConfigParser::saveToFile(const char* filePath, const EncoderConfig& config) {
    if (filePath == nullptr) {
        return EncoderStatus::CONFIG_ERROR;
    }

    FILE* file = fopen(filePath, "w");
    if (file == nullptr) {
        return EncoderStatus::CONFIG_ERROR;
    }

    fprintf(file, "{\n");
    fprintf(file, "  \"encoder\": {\n");
    fprintf(file, "    \"cpr\": %u,\n", config.cpr);
    fprintf(file, "    \"invertA\": %s,\n", config.invertA ? "true" : "false");
    fprintf(file, "    \"invertB\": %s,\n", config.invertB ? "true" : "false");
    fprintf(file, "    \"useZ\": %s,\n", config.useZ ? "true" : "false");
    
    const char* modeStr = "\"TIM_ENCODER_MODE\"";
    switch (config.mode) {
        case EncoderMode::TIM_ENCODER_MODE:
            modeStr = "\"TIM_ENCODER_MODE\"";
            break;
        case EncoderMode::EXTERNAL_COUNTER:
            modeStr = "\"EXTERNAL_COUNTER\"";
            break;
    }
    
    fprintf(file, "    \"mode\": %s,\n", modeStr);
    fprintf(file, "    \"watchdogTimeoutMs\": %u,\n", config.watchdogTimeoutMs);
    fprintf(file, "    \"offsetCounts\": %d,\n", config.offsetCounts);
    fprintf(file, "    \"wrapAround\": %s\n", config.wrapAround ? "true" : "false");
    fprintf(file, "  }\n");
    fprintf(file, "}\n");

    fclose(file);
    return EncoderStatus::OK;
}

bool EncoderConfigParser::parseUint16(const char* json, const char* key, uint16_t& value) {
    const char* valueStr = findKey(json, key);
    if (valueStr == nullptr) {
        return false;
    }
    return parseValue(valueStr, value);
}

bool EncoderConfigParser::parseUint32(const char* json, const char* key, uint32_t& value) {
    const char* valueStr = findKey(json, key);
    if (valueStr == nullptr) {
        return false;
    }
    return parseValue(valueStr, value);
}

bool EncoderConfigParser::parseInt32(const char* json, const char* key, int32_t& value) {
    const char* valueStr = findKey(json, key);
    if (valueStr == nullptr) {
        return false;
    }
    return parseValue(valueStr, value);
}

bool EncoderConfigParser::parseBool(const char* json, const char* key, bool& value) {
    const char* valueStr = findKey(json, key);
    if (valueStr == nullptr) {
        return false;
    }
    return parseValue(valueStr, value);
}

bool EncoderConfigParser::parseEncoderMode(const char* json, const char* key, EncoderMode& value) {
    const char* valueStr = findKey(json, key);
    if (valueStr == nullptr) {
        return false;
    }

    skipWhitespace(valueStr);
    if (*valueStr != '"') {
        return false;
    }
    valueStr++;

    if (strncmp(valueStr, "TIM_ENCODER_MODE", 16) == 0) {
        value = EncoderMode::TIM_ENCODER_MODE;
    } else if (strncmp(valueStr, "EXTERNAL_COUNTER", 16) == 0) {
        value = EncoderMode::EXTERNAL_COUNTER;
    } else {
        return false;
    }

    return true;
}

const char* EncoderConfigParser::findKey(const char* json, const char* key) {
    const char* keyPos = strstr(json, key);
    if (keyPos == nullptr) {
        return nullptr;
    }

    const char* colonPos = strchr(keyPos, ':');
    if (colonPos == nullptr) {
        return nullptr;
    }

    const char* valueStart = colonPos + 1;
    skipWhitespace(valueStart);

    return valueStart;
}

bool EncoderConfigParser::parseValue(const char* valueStr, uint16_t& value) {
    char* endPtr;
    unsigned long temp = strtoul(valueStr, &endPtr, 10);
    if (endPtr == valueStr || temp > UINT16_MAX) {
        return false;
    }
    value = static_cast<uint16_t>(temp);
    return true;
}

bool EncoderConfigParser::parseValue(const char* valueStr, uint32_t& value) {
    char* endPtr;
    value = strtoul(valueStr, &endPtr, 10);
    return endPtr != valueStr;
}

bool EncoderConfigParser::parseValue(const char* valueStr, int32_t& value) {
    char* endPtr;
    value = strtol(valueStr, &endPtr, 10);
    return endPtr != valueStr;
}

bool EncoderConfigParser::parseValue(const char* valueStr, bool& value) {
    skipWhitespace(valueStr);
    
    if (strncmp(valueStr, "true", 4) == 0) {
        value = true;
        return true;
    } else if (strncmp(valueStr, "false", 5) == 0) {
        value = false;
        return true;
    }
    
    return false;
}

void EncoderConfigParser::skipWhitespace(const char*& ptr) {
    while (*ptr == ' ' || *ptr == '\t' || *ptr == '\n' || *ptr == '\r') {
        ptr++;
    }
}

const char* EncoderConfigParser::findValueEnd(const char* valueStart) {
    const char* ptr = valueStart;
    skipWhitespace(ptr);
    
    if (*ptr == '"') {
        ptr++;
        while (*ptr != '\0' && *ptr != '"') {
            ptr++;
        }
        if (*ptr == '"') {
            ptr++;
        }
    } else {
        while (*ptr != '\0' && *ptr != ',' && *ptr != '}' && *ptr != '\n' && *ptr != '\r') {
            ptr++;
        }
    }
    
    return ptr;
}

const char* EncoderConfigParser::findEncoderSection(const char* json, uint8_t encoderId) {
    char idStr[8];
    sprintf(idStr, "\"%d\"", encoderId);
    
    const char* idPos = strstr(json, idStr);
    if (idPos == nullptr) {
        return nullptr;
    }
    
    const char* colonPos = strchr(idPos, ':');
    if (colonPos == nullptr) {
        return nullptr;
    }
    
    const char* bracePos = strchr(colonPos, '{');
    if (bracePos == nullptr) {
        return nullptr;
    }
    
    return bracePos + 1;
}

const char* EncoderConfigParser::findSectionEnd(const char* sectionStart) {
    const char* ptr = sectionStart;
    int braceCount = 1;
    
    while (*ptr != '\0' && braceCount > 0) {
        if (*ptr == '{') {
            braceCount++;
        } else if (*ptr == '}') {
            braceCount--;
        }
        ptr++;
    }
    
    return (braceCount == 0) ? ptr - 1 : nullptr;
}