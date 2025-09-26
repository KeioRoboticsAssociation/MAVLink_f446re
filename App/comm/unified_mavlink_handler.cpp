// Phase 1 stub implementation for UnifiedMAVLinkHandler
// Full implementation will be completed in Phase 2-3 after architecture cleanup

#include "unified_mavlink_handler.hpp"

namespace Communication {

// Stub implementation to allow Phase 1 architecture cleanup to complete
UnifiedMAVLinkHandler::UnifiedMAVLinkHandler(HAL::HardwareManager* /*hwManager*/, uint8_t /*systemId*/, uint8_t /*componentId*/) {
    // Phase 1 stub - no initialization needed
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::initialize() {
    // Phase 1 stub - always succeed
    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::update() {
    // Phase 1 stub - always succeed
    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::registerDevice(IMAVLinkDevice* /*device*/) {
    // Phase 1 stub - always succeed
    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::unregisterDevice(uint8_t /*deviceId*/) {
    // Phase 1 stub - always succeed
    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::sendMessage(const mavlink_message_t& /*message*/) {
    // Phase 1 stub - always succeed
    return Config::ErrorCode::OK;
}

Config::Result<Config::ErrorCode> UnifiedMAVLinkHandler::queueMessage(const mavlink_message_t& /*message*/) {
    // Phase 1 stub - always succeed
    return Config::ErrorCode::OK;
}

void UnifiedMAVLinkHandler::processReceivedByte(uint8_t /*byte*/) {
    // Phase 1 stub - no processing
}

void UnifiedMAVLinkHandler::handleReceivedMessage(const mavlink_message_t& /*msg*/) {
    // Phase 1 stub - no processing
}

} // namespace Communication