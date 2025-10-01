/**
 * @file flash_storage.hpp
 * @brief Defines the low-level flash storage driver.
 */

#pragma once

#include "storage_config.hpp"
#include "../config/system_config.hpp"

extern "C" {
#include "stm32f4xx_hal.h"
}

#include <cstdint>
#include <cstring>

namespace Storage {

/**
 * @brief Provides an interface for reading and writing to flash memory.
 */
class FlashStorage {
private:
    struct SectorInfo {
        uint32_t address;
        uint32_t size;
        uint32_t sector_number;
        uint32_t write_count;
        bool is_bad;
    };

    SectorInfo sectors_[2];
    uint8_t active_sector_;
    uint32_t total_write_cycles_;
    bool initialized_;

public:
    FlashStorage();
    ~FlashStorage() = default;

    /**
     * @brief Initializes the flash storage driver.
     * @return A StorageResult indicating success or failure.
     */
    StorageResult<bool> initialize();

    /**
     * @brief Checks if the flash storage is initialized.
     * @return A StorageResult containing true if initialized, false otherwise.
     */
    StorageResult<bool> isInitialized() const { return initialized_; }

    /**
     * @brief Erases a flash sector.
     * @param sector_index The index of the sector to erase.
     * @return A StorageResult indicating success or failure.
     */
    StorageResult<bool> eraseSector(uint8_t sector_index);

    /**
     * @brief Writes a block of data to a flash address.
     * @param address The address to write to.
     * @param data A pointer to the data to write.
     * @param size The size of the data to write.
     * @return A StorageResult indicating success or failure.
     */
    StorageResult<bool> writeBlock(uint32_t address, const uint8_t* data, size_t size);

    /**
     * @brief Reads a block of data from a flash address.
     * @param address The address to read from.
     * @param data A pointer to the buffer to store the read data.
     * @param size The size of the data to read.
     * @return A StorageResult indicating success or failure.
     */
    StorageResult<bool> readBlock(uint32_t address, uint8_t* data, size_t size);

    StorageResult<bool> eraseBlock(uint32_t block_address);
    StorageResult<bool> writeBlockData(uint32_t block_address, const uint8_t* data, size_t size);
    StorageResult<bool> readBlockData(uint32_t block_address, uint8_t* data, size_t size);

    StorageResult<uint8_t> getActiveSector() const;
    StorageResult<bool> switchToNextSector();
    StorageResult<uint32_t> getWriteCount(uint8_t sector_index) const;

    bool isSectorBad(uint8_t sector_index) const;
    StorageResult<uint32_t> getTotalWriteCycles() const;
    StorageResult<float> getWearLevel() const;

    StorageResult<bool> verifyBlock(uint32_t address, const uint8_t* expected_data, size_t size);
    StorageResult<uint32_t> calculateCRC32(const uint8_t* data, size_t size);

private:
    StorageResult<bool> unlockFlash();
    StorageResult<bool> lockFlash();
    StorageResult<uint32_t> getSectorNumber(uint32_t address);
    StorageResult<bool> waitForFlashReady(uint32_t timeout_ms = 1000);
    void markSectorBad(uint8_t sector_index);
    StorageResult<bool> validateAddress(uint32_t address, size_t size);

    static constexpr uint32_t CRC32_POLYNOMIAL = 0xEDB88320;
    uint32_t crc32_table_[256];
    void initCRC32Table();
    uint32_t updateCRC32(uint32_t crc, const uint8_t* data, size_t size);
};

extern FlashStorage g_flash_storage;

} // namespace Storage