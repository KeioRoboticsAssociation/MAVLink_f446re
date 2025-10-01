#pragma once

#include "storage_config.hpp"
#include "../config/system_config.hpp"

extern "C" {
#include "stm32f4xx_hal.h"
}

#include <cstdint>
#include <cstring>

namespace Storage {

class FlashStorage {
private:
    struct SectorInfo {
        uint32_t address;
        uint32_t size;
        uint32_t sector_number;
        uint32_t write_count;
        bool is_bad;
    };

    SectorInfo sectors_[2];  // Two sectors for ping-pong
    uint8_t active_sector_;
    uint32_t total_write_cycles_;
    bool initialized_;

public:
    FlashStorage();
    ~FlashStorage() = default;

    // Initialization and management
    StorageResult<bool> initialize();
    StorageResult<bool> isInitialized() const { return initialized_; }

    // Low-level flash operations
    StorageResult<bool> eraseSector(uint8_t sector_index);
    StorageResult<bool> writeBlock(uint32_t address, const uint8_t* data, size_t size);
    StorageResult<bool> readBlock(uint32_t address, uint8_t* data, size_t size);

    // Block-level operations
    StorageResult<bool> eraseBlock(uint32_t block_address);
    StorageResult<bool> writeBlockData(uint32_t block_address, const uint8_t* data, size_t size);
    StorageResult<bool> readBlockData(uint32_t block_address, uint8_t* data, size_t size);

    // Sector management
    StorageResult<uint8_t> getActiveSector() const;
    StorageResult<bool> switchToNextSector();
    StorageResult<uint32_t> getWriteCount(uint8_t sector_index) const;

    // Health monitoring
    bool isSectorBad(uint8_t sector_index) const;
    StorageResult<uint32_t> getTotalWriteCycles() const;
    StorageResult<float> getWearLevel() const;

    // Utility functions
    StorageResult<bool> verifyBlock(uint32_t address, const uint8_t* expected_data, size_t size);
    StorageResult<uint32_t> calculateCRC32(const uint8_t* data, size_t size);

private:
    // Internal helpers
    StorageResult<bool> unlockFlash();
    StorageResult<bool> lockFlash();
    StorageResult<uint32_t> getSectorNumber(uint32_t address);
    StorageResult<bool> waitForFlashReady(uint32_t timeout_ms = 1000);
    void markSectorBad(uint8_t sector_index);
    StorageResult<bool> validateAddress(uint32_t address, size_t size);

    // CRC32 calculation (software implementation)
    static constexpr uint32_t CRC32_POLYNOMIAL = 0xEDB88320;
    uint32_t crc32_table_[256];
    void initCRC32Table();
    uint32_t updateCRC32(uint32_t crc, const uint8_t* data, size_t size);
};

// Singleton instance for global access
extern FlashStorage g_flash_storage;

} // namespace Storage