#pragma once

#include <cstdint>

namespace Storage {

// Flash memory layout for STM32F446RETx (512KB total)
namespace Memory {
    // Flash sectors for STM32F446RETx
    // Sectors 0-5: 16KB each, Sectors 6-7: 64KB each, Sectors 8-11: 128KB each
    constexpr uint32_t FLASH_BASE_ADDR = 0x08000000;
    constexpr uint32_t FLASH_SIZE = 512 * 1024;  // 512KB

    // Reserve last two 64KB sectors (6-7) for parameter storage
    constexpr uint32_t PARAM_SECTOR_6_ADDR = 0x08040000;  // Sector 6 (64KB)
    constexpr uint32_t PARAM_SECTOR_7_ADDR = 0x08050000;  // Sector 7 (64KB)
    constexpr uint32_t PARAM_SECTOR_SIZE = 64 * 1024;     // 64KB per sector

    // Use smaller 8KB blocks within sectors for wear leveling
    constexpr uint32_t PARAM_BLOCK_SIZE = 8 * 1024;       // 8KB blocks
    constexpr uint32_t BLOCKS_PER_SECTOR = PARAM_SECTOR_SIZE / PARAM_BLOCK_SIZE; // 8 blocks per sector
    constexpr uint32_t TOTAL_PARAM_BLOCKS = 2 * BLOCKS_PER_SECTOR;  // 16 total blocks

    // Current implementation uses 2-block ping-pong for simplicity
    constexpr uint32_t ACTIVE_BLOCK_COUNT = 2;
    constexpr uint32_t PRIMARY_BLOCK_ADDR = PARAM_SECTOR_6_ADDR;
    constexpr uint32_t SECONDARY_BLOCK_ADDR = PARAM_SECTOR_6_ADDR + PARAM_BLOCK_SIZE;
}

// Parameter storage configuration
namespace Config {
    constexpr uint32_t MAGIC_NUMBER = 0xDEADBEEF;
    constexpr uint16_t STORAGE_VERSION = 1;
    constexpr uint16_t MAX_PARAMETERS = 64;
    constexpr uint8_t MAX_PARAM_NAME_LEN = 16;

    // Safety limits
    constexpr uint32_t MAX_WRITE_CYCLES = 10000;  // Flash endurance limit
    constexpr uint32_t CORRUPTION_THRESHOLD = 3;   // Max consecutive corruption errors
    constexpr uint32_t BACKUP_INTERVAL_MS = 30000; // Auto-backup every 30 seconds

    // Performance settings
    constexpr uint32_t CACHE_SIZE = 128;           // Parameter cache size in RAM
    constexpr bool ENABLE_WEAR_LEVELING = true;
    constexpr bool ENABLE_AUTO_BACKUP = true;
}

// Error codes specific to storage operations
enum class StorageError : uint8_t {
    SUCCESS = 0,
    FLASH_ERROR,
    CORRUPTION_DETECTED,
    INVALID_PARAMETER,
    STORAGE_FULL,
    WRITE_PROTECTED,
    SECTOR_BAD,
    CRC_MISMATCH,
    VERSION_MISMATCH,
    TIMEOUT
};

// Storage operation results
template<typename T>
struct StorageResult {
    StorageError error;
    T value;

    StorageResult(StorageError err) : error(err), value{} {}
    StorageResult(T val) : error(StorageError::SUCCESS), value(val) {}

    bool isSuccess() const { return error == StorageError::SUCCESS; }
    bool isError() const { return error != StorageError::SUCCESS; }
};

} // namespace Storage