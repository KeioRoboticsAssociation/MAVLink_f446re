#include "ServoConfig.hpp"
#include <cstdlib>
#include <cstdio>

const ServoConfig ServoConfigDefaults::STANDARD_SERVO = {
    .angleMinDeg = -90.0f,
    .angleMaxDeg = 90.0f,
    .pulseMinUs = 1000,
    .pulseMaxUs = 2000,
    .pulseNeutralUs = 1500,
    .directionInverted = false,
    .offsetDeg = 0.0f,
    .maxVelocityDegPerS = 180.0f,
    .maxAccelerationDegPerS2 = 360.0f,
    .watchdogTimeoutMs = 500,
    .failSafeBehavior = FailSafeBehavior::NEUTRAL_POSITION,
    .startupAngleDeg = 0.0f,
    .startDisabled = false
};

const ServoConfig ServoConfigDefaults::HIGH_TORQUE_SERVO = {
    .angleMinDeg = -90.0f,
    .angleMaxDeg = 90.0f,
    .pulseMinUs = 500,
    .pulseMaxUs = 2500,
    .pulseNeutralUs = 1500,
    .directionInverted = false,
    .offsetDeg = 0.0f,
    .maxVelocityDegPerS = 120.0f,
    .maxAccelerationDegPerS2 = 240.0f,
    .watchdogTimeoutMs = 1000,
    .failSafeBehavior = FailSafeBehavior::HOLD_POSITION,
    .startupAngleDeg = 0.0f,
    .startDisabled = false
};

const ServoConfig ServoConfigDefaults::CONTINUOUS_ROTATION_SERVO = {
    .angleMinDeg = -180.0f,
    .angleMaxDeg = 180.0f,
    .pulseMinUs = 1000,
    .pulseMaxUs = 2000,
    .pulseNeutralUs = 1500,
    .directionInverted = false,
    .offsetDeg = 0.0f,
    .maxVelocityDegPerS = 360.0f,
    .maxAccelerationDegPerS2 = 720.0f,
    .watchdogTimeoutMs = 500,
    .failSafeBehavior = FailSafeBehavior::NEUTRAL_POSITION,
    .startupAngleDeg = 0.0f,
    .startDisabled = false
};

const ServoConfig ServoConfigDefaults::DIGITAL_SERVO = {
    .angleMinDeg = -90.0f,
    .angleMaxDeg = 90.0f,
    .pulseMinUs = 600,
    .pulseMaxUs = 2400,
    .pulseNeutralUs = 1500,
    .directionInverted = false,
    .offsetDeg = 0.0f,
    .maxVelocityDegPerS = 300.0f,
    .maxAccelerationDegPerS2 = 600.0f,
    .watchdogTimeoutMs = 200,
    .failSafeBehavior = FailSafeBehavior::NEUTRAL_POSITION,
    .startupAngleDeg = 0.0f,
    .startDisabled = false
};

ServoStatus ServoConfigParser::parseFromJson(const char* jsonString, ServoConfig& config) {
    if (jsonString == nullptr) {
        return ServoStatus::CONFIG_ERROR;
    }

    config = ServoConfigDefaults::STANDARD_SERVO;

    parseFloat(jsonString, "angleMinDeg", config.angleMinDeg);
    parseFloat(jsonString, "angleMaxDeg", config.angleMaxDeg);
    parseUint16(jsonString, "pulseMinUs", config.pulseMinUs);
    parseUint16(jsonString, "pulseMaxUs", config.pulseMaxUs);
    parseUint16(jsonString, "pulseNeutralUs", config.pulseNeutralUs);
    parseBool(jsonString, "directionInverted", config.directionInverted);
    parseFloat(jsonString, "offsetDeg", config.offsetDeg);
    parseFloat(jsonString, "maxVelocityDegPerS", config.maxVelocityDegPerS);
    parseFloat(jsonString, "maxAccelerationDegPerS2", config.maxAccelerationDegPerS2);
    parseUint32(jsonString, "watchdogTimeoutMs", config.watchdogTimeoutMs);
    parseFailSafeBehavior(jsonString, "failSafeBehavior", config.failSafeBehavior);
    parseFloat(jsonString, "startupAngleDeg", config.startupAngleDeg);
    parseBool(jsonString, "startDisabled", config.startDisabled);

    if (config.angleMinDeg >= config.angleMaxDeg ||
        config.pulseMinUs >= config.pulseMaxUs ||
        config.pulseNeutralUs < config.pulseMinUs ||
        config.pulseNeutralUs > config.pulseMaxUs) {
        return ServoStatus::CONFIG_ERROR;
    }

    return ServoStatus::OK;
}

ServoStatus ServoConfigParser::parseFromJsonForId(const char* jsonString, uint8_t servoId, ServoConfig& config) {
    if (jsonString == nullptr) {
        return ServoStatus::CONFIG_ERROR;
    }

    const char* servoSection = findServoSection(jsonString, servoId);
    if (servoSection == nullptr) {
        return ServoStatus::CONFIG_ERROR;
    }

    config = ServoConfigDefaults::STANDARD_SERVO;

    parseFloat(servoSection, "initial_offset", config.offsetDeg);
    parseFloat(servoSection, "min_angle", config.angleMinDeg);
    parseFloat(servoSection, "max_angle", config.angleMaxDeg);
    
    parseFloat(servoSection, "angleMinDeg", config.angleMinDeg);
    parseFloat(servoSection, "angleMaxDeg", config.angleMaxDeg);
    parseUint16(servoSection, "pulseMinUs", config.pulseMinUs);
    parseUint16(servoSection, "pulseMaxUs", config.pulseMaxUs);
    parseUint16(servoSection, "pulseNeutralUs", config.pulseNeutralUs);
    parseBool(servoSection, "directionInverted", config.directionInverted);
    parseFloat(servoSection, "offsetDeg", config.offsetDeg);
    parseFloat(servoSection, "maxVelocityDegPerS", config.maxVelocityDegPerS);
    parseFloat(servoSection, "maxAccelerationDegPerS2", config.maxAccelerationDegPerS2);
    parseUint32(servoSection, "watchdogTimeoutMs", config.watchdogTimeoutMs);
    parseFailSafeBehavior(servoSection, "failSafeBehavior", config.failSafeBehavior);
    parseFloat(servoSection, "startupAngleDeg", config.startupAngleDeg);
    parseBool(servoSection, "startDisabled", config.startDisabled);

    if (config.angleMinDeg >= config.angleMaxDeg ||
        config.pulseMinUs >= config.pulseMaxUs ||
        config.pulseNeutralUs < config.pulseMinUs ||
        config.pulseNeutralUs > config.pulseMaxUs) {
        return ServoStatus::CONFIG_ERROR;
    }

    return ServoStatus::OK;
}

ServoStatus ServoConfigParser::loadFromFile(const char* filePath, ServoConfig& config) {
    if (filePath == nullptr) {
        return ServoStatus::CONFIG_ERROR;
    }

    FILE* file = fopen(filePath, "r");
    if (file == nullptr) {
        return ServoStatus::CONFIG_ERROR;
    }

    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (fileSize <= 0 || fileSize > 2048) {
        fclose(file);
        return ServoStatus::CONFIG_ERROR;
    }

    char* jsonBuffer = new char[fileSize + 1];
    size_t bytesRead = fread(jsonBuffer, 1, fileSize, file);
    fclose(file);

    if (bytesRead != static_cast<size_t>(fileSize)) {
        delete[] jsonBuffer;
        return ServoStatus::CONFIG_ERROR;
    }

    jsonBuffer[fileSize] = '\0';

    ServoStatus result = parseFromJson(jsonBuffer, config);
    delete[] jsonBuffer;

    return result;
}

ServoStatus ServoConfigParser::loadFromFileForId(const char* filePath, uint8_t servoId, ServoConfig& config) {
    if (filePath == nullptr) {
        return ServoStatus::CONFIG_ERROR;
    }

    FILE* file = fopen(filePath, "r");
    if (file == nullptr) {
        return ServoStatus::CONFIG_ERROR;
    }

    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    if (fileSize <= 0 || fileSize > 2048) {
        fclose(file);
        return ServoStatus::CONFIG_ERROR;
    }

    char* jsonBuffer = new char[fileSize + 1];
    size_t bytesRead = fread(jsonBuffer, 1, fileSize, file);
    fclose(file);

    if (bytesRead != static_cast<size_t>(fileSize)) {
        delete[] jsonBuffer;
        return ServoStatus::CONFIG_ERROR;
    }

    jsonBuffer[fileSize] = '\0';

    ServoStatus result = parseFromJsonForId(jsonBuffer, servoId, config);
    delete[] jsonBuffer;

    return result;
}

ServoStatus ServoConfigParser::saveToFile(const char* filePath, const ServoConfig& config) {
    if (filePath == nullptr) {
        return ServoStatus::CONFIG_ERROR;
    }

    FILE* file = fopen(filePath, "w");
    if (file == nullptr) {
        return ServoStatus::CONFIG_ERROR;
    }

    fprintf(file, "{\n");
    fprintf(file, "  \"angleMinDeg\": %.2f,\n", config.angleMinDeg);
    fprintf(file, "  \"angleMaxDeg\": %.2f,\n", config.angleMaxDeg);
    fprintf(file, "  \"pulseMinUs\": %u,\n", config.pulseMinUs);
    fprintf(file, "  \"pulseMaxUs\": %u,\n", config.pulseMaxUs);
    fprintf(file, "  \"pulseNeutralUs\": %u,\n", config.pulseNeutralUs);
    fprintf(file, "  \"directionInverted\": %s,\n", config.directionInverted ? "true" : "false");
    fprintf(file, "  \"offsetDeg\": %.2f,\n", config.offsetDeg);
    fprintf(file, "  \"maxVelocityDegPerS\": %.2f,\n", config.maxVelocityDegPerS);
    fprintf(file, "  \"maxAccelerationDegPerS2\": %.2f,\n", config.maxAccelerationDegPerS2);
    fprintf(file, "  \"watchdogTimeoutMs\": %u,\n", config.watchdogTimeoutMs);
    
    const char* failSafeBehaviorStr = "\"NEUTRAL_POSITION\"";
    switch (config.failSafeBehavior) {
        case FailSafeBehavior::HOLD_POSITION:
            failSafeBehaviorStr = "\"HOLD_POSITION\"";
            break;
        case FailSafeBehavior::NEUTRAL_POSITION:
            failSafeBehaviorStr = "\"NEUTRAL_POSITION\"";
            break;
        case FailSafeBehavior::DISABLE_OUTPUT:
            failSafeBehaviorStr = "\"DISABLE_OUTPUT\"";
            break;
    }
    
    fprintf(file, "  \"failSafeBehavior\": %s,\n", failSafeBehaviorStr);
    fprintf(file, "  \"startupAngleDeg\": %.2f,\n", config.startupAngleDeg);
    fprintf(file, "  \"startDisabled\": %s\n", config.startDisabled ? "true" : "false");
    fprintf(file, "}\n");

    fclose(file);
    return ServoStatus::OK;
}

bool ServoConfigParser::parseFloat(const char* json, const char* key, float& value) {
    const char* valueStr = findKey(json, key);
    if (valueStr == nullptr) {
        return false;
    }
    return parseValue(valueStr, value);
}

bool ServoConfigParser::parseUint16(const char* json, const char* key, uint16_t& value) {
    const char* valueStr = findKey(json, key);
    if (valueStr == nullptr) {
        return false;
    }
    return parseValue(valueStr, value);
}

bool ServoConfigParser::parseUint32(const char* json, const char* key, uint32_t& value) {
    const char* valueStr = findKey(json, key);
    if (valueStr == nullptr) {
        return false;
    }
    return parseValue(valueStr, value);
}

bool ServoConfigParser::parseBool(const char* json, const char* key, bool& value) {
    const char* valueStr = findKey(json, key);
    if (valueStr == nullptr) {
        return false;
    }
    return parseValue(valueStr, value);
}

bool ServoConfigParser::parseFailSafeBehavior(const char* json, const char* key, FailSafeBehavior& value) {
    const char* valueStr = findKey(json, key);
    if (valueStr == nullptr) {
        return false;
    }

    skipWhitespace(valueStr);
    if (*valueStr != '"') {
        return false;
    }
    valueStr++;

    if (strncmp(valueStr, "HOLD_POSITION", 13) == 0) {
        value = FailSafeBehavior::HOLD_POSITION;
    } else if (strncmp(valueStr, "NEUTRAL_POSITION", 16) == 0) {
        value = FailSafeBehavior::NEUTRAL_POSITION;
    } else if (strncmp(valueStr, "DISABLE_OUTPUT", 14) == 0) {
        value = FailSafeBehavior::DISABLE_OUTPUT;
    } else {
        return false;
    }

    return true;
}

const char* ServoConfigParser::findKey(const char* json, const char* key) {
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

bool ServoConfigParser::parseValue(const char* valueStr, float& value) {
    char* endPtr;
    value = strtof(valueStr, &endPtr);
    return endPtr != valueStr;
}

bool ServoConfigParser::parseValue(const char* valueStr, uint16_t& value) {
    char* endPtr;
    unsigned long temp = strtoul(valueStr, &endPtr, 10);
    if (endPtr == valueStr || temp > UINT16_MAX) {
        return false;
    }
    value = static_cast<uint16_t>(temp);
    return true;
}

bool ServoConfigParser::parseValue(const char* valueStr, uint32_t& value) {
    char* endPtr;
    value = strtoul(valueStr, &endPtr, 10);
    return endPtr != valueStr;
}

bool ServoConfigParser::parseValue(const char* valueStr, bool& value) {
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

void ServoConfigParser::skipWhitespace(const char*& ptr) {
    while (*ptr == ' ' || *ptr == '\t' || *ptr == '\n' || *ptr == '\r') {
        ptr++;
    }
}

const char* ServoConfigParser::findValueEnd(const char* valueStart) {
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

const char* ServoConfigParser::findServoSection(const char* json, uint8_t servoId) {
    char idStr[8];
    sprintf(idStr, "\"%d\"", servoId);
    
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

const char* ServoConfigParser::findSectionEnd(const char* sectionStart) {
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