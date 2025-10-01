/**
 * @file storage_config.hpp
 * @brief Defines the configuration for flash storage and parameter management.
 */

#pragma once

#include <cstdint>

namespace Storage {

/**
 * @brief Defines the flash memory layout for the STM32F446RETx.
 */
namespace Memory {
    constexpr uint32_t FLASH_BASE_ADDR = 0x08000000;
    constexpr uint32_t FLASH_SIZE = 512 * 1024;

    constexpr uint32_t PARAM_SECTOR_6_ADDR = 0x08040000;
    constexpr uint32_t PARAM_SECTOR_7_ADDR = 0x08050000;
    constexpr uint32_t PARAM_SECTOR_SIZE = 64 * 1024;

    constexpr uint32_t PARAM_BLOCK_SIZE = 8 * 1024;
    constexpr uint32_t BLOCKS_PER_SECTOR = PARAM_SECTOR_SIZE / PARAM_BLOCK_SIZE;
    constexpr uint32_t TOTAL_PARAM_BLOCKS = 2 * BLOCKS_PER_SECTOR;

    constexpr uint32_t ACTIVE_BLOCK_COUNT = 2;
    constexpr uint32_t PRIMARY_BLOCK_ADDR = PARAM_SECTOR_6_ADDR;
    constexpr uint32_t SECONDARY_BLOCK_ADDR = PARAM_SECTOR_6_ADDR + PARAM_BLOCK_SIZE;
}

/**
 * @brief Configuration for parameter storage.
 */
namespace Config {
    constexpr uint32_t MAGIC_NUMBER = 0xDEADBEEF;
    constexpr uint16_t STORAGE_VERSION = 1;
    constexpr uint16_t MAX_PARAMETERS = 64;
    constexpr uint8_t MAX_PARAM_NAME_LEN = 16;

    constexpr uint32_t MAX_WRITE_CYCLES = 10000;
    constexpr uint32_t CORRUPTION_THRESHOLD = 3;
    constexpr uint32_t BACKUP_INTERVAL_MS = 30000;

    constexpr uint32_t CACHE_SIZE = 128;
    constexpr bool ENABLE_WEAR_LEVELING = true;
    constexpr bool ENABLE_AUTO_BACKUP = true;
}

/**
 * @brief Defines error codes specific to storage operations.
 */
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

/**
 * @brief Represents the result of a storage operation.
 * @tparam T The type of the value returned on success.
 */
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