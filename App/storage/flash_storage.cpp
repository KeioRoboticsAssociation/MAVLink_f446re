/**
 * @file flash_storage.cpp
 * @brief Implements the low-level flash storage driver.
 */

#include "flash_storage.hpp"

extern "C" {
#include "stm32f4xx_hal.h"
}

namespace Storage {

FlashStorage g_flash_storage;

/**
 * @brief Construct a new FlashStorage object.
 */
FlashStorage::FlashStorage()
    : active_sector_(0), total_write_cycles_(0), initialized_(false) {
    sectors_[0] = {Memory::PARAM_SECTOR_6_ADDR, Memory::PARAM_SECTOR_SIZE, 6, 0, false};
    sectors_[1] = {Memory::PARAM_SECTOR_7_ADDR, Memory::PARAM_SECTOR_SIZE, 7, 0, false};
    initCRC32Table();
}

/**
 * @brief Initializes the flash storage driver.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> FlashStorage::initialize() {
    if (initialized_) {
        return true;
    }

    if (Memory::PARAM_SECTOR_6_ADDR < Memory::FLASH_BASE_ADDR ||
        Memory::PARAM_SECTOR_7_ADDR >= (Memory::FLASH_BASE_ADDR + Memory::FLASH_SIZE)) {
        return StorageError::INVALID_PARAMETER;
    }

    uint32_t test_data;
    for (uint8_t i = 0; i < 2; i++) {
        auto result = readBlock(sectors_[i].address, reinterpret_cast<uint8_t*>(&test_data), sizeof(test_data));
        if (result.isError()) {
            markSectorBad(i);
        }
    }

    if (sectors_[0].is_bad && sectors_[1].is_bad) {
        return StorageError::SECTOR_BAD;
    }

    active_sector_ = sectors_[0].is_bad ? 1 : 0;
    initialized_ = true;
    return true;
}

/**
 * @brief Erases a flash sector.
 * @param sector_index The index of the sector to erase.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> FlashStorage::eraseSector(uint8_t sector_index) {
    if (sector_index >= 2) return StorageError::INVALID_PARAMETER;
    if (sectors_[sector_index].is_bad) return StorageError::SECTOR_BAD;

    auto unlock_result = unlockFlash();
    if (unlock_result.isError()) return unlock_result.error;

    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase_init.Sector = sectors_[sector_index].sector_number;
    erase_init.NbSectors = 1;

    uint32_t sector_error = 0;
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &sector_error);
    auto lock_result = lockFlash();

    if (status != HAL_OK || sector_error != 0xFFFFFFFF) {
        markSectorBad(sector_index);
        return StorageError::FLASH_ERROR;
    }

    sectors_[sector_index].write_count++;
    total_write_cycles_++;
    return lock_result;
}

/**
 * @brief Writes a block of data to a flash address.
 * @param address The address to write to.
 * @param data A pointer to the data to write.
 * @param size The size of the data to write.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> FlashStorage::writeBlock(uint32_t address, const uint8_t* data, size_t size) {
    if (!data || size == 0) return StorageError::INVALID_PARAMETER;
    auto validation = validateAddress(address, size);
    if (validation.isError()) return validation.error;

    auto unlock_result = unlockFlash();
    if (unlock_result.isError()) return unlock_result.error;

    const uint32_t* word_data = reinterpret_cast<const uint32_t*>(data);
    size_t word_count = (size + 3) / 4;
    HAL_StatusTypeDef status = HAL_OK;

    for (size_t i = 0; i < word_count && status == HAL_OK; i++) {
        uint32_t word_to_write = (i * 4 + 3 < size) ? word_data[i] : 0xFFFFFFFF;
        if (i == word_count - 1 && size % 4 != 0) {
            const uint8_t* byte_data = data + i * 4;
            word_to_write = 0xFFFFFFFF;
            for (size_t j = 0; j < size % 4; j++) {
                word_to_write = (word_to_write & ~(0xFF << (j * 8))) | (byte_data[j] << (j * 8));
            }
        }
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i * 4, word_to_write);
    }

    auto lock_result = lockFlash();
    if (status != HAL_OK) return StorageError::FLASH_ERROR;
    return lock_result;
}

/**
 * @brief Reads a block of data from a flash address.
 * @param address The address to read from.
 * @param data A pointer to the buffer to store the read data.
 * @param size The size of the data to read.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> FlashStorage::readBlock(uint32_t address, uint8_t* data, size_t size) {
    if (!data || size == 0) return StorageError::INVALID_PARAMETER;
    auto validation = validateAddress(address, size);
    if (validation.isError()) return validation.error;
    std::memcpy(data, reinterpret_cast<const uint8_t*>(address), size);
    return true;
}

/**
 * @brief Erases a block of flash memory.
 * @param block_address The starting address of the block to erase.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> FlashStorage::eraseBlock(uint32_t block_address) {
    uint8_t sector_index = (block_address >= Memory::PARAM_SECTOR_7_ADDR) ? 1 : 0;
    return eraseSector(sector_index);
}

/**
 * @brief Writes data to a block in flash memory.
 * @param block_address The starting address of the block to write to.
 * @param data A pointer to the data to write.
 * @param size The size of the data to write.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> FlashStorage::writeBlockData(uint32_t block_address, const uint8_t* data, size_t size) {
    if (size > Memory::PARAM_BLOCK_SIZE) return StorageError::INVALID_PARAMETER;
    return writeBlock(block_address, data, size);
}

/**
 * @brief Reads data from a block in flash memory.
 * @param block_address The starting address of the block to read from.
 * @param data A pointer to the buffer to store the read data.
 * @param size The size of the data to read.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> FlashStorage::readBlockData(uint32_t block_address, uint8_t* data, size_t size) {
    if (size > Memory::PARAM_BLOCK_SIZE) return StorageError::INVALID_PARAMETER;
    return readBlock(block_address, data, size);
}

/**
 * @brief Gets the index of the active flash sector.
 * @return A StorageResult containing the active sector index or an error.
 */
StorageResult<uint8_t> FlashStorage::getActiveSector() const {
    if (!initialized_) return StorageError::FLASH_ERROR;
    return active_sector_;
}

/**
 * @brief Switches to the next available flash sector.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> FlashStorage::switchToNextSector() {
    if (!initialized_) return StorageError::FLASH_ERROR;
    uint8_t next_sector = (active_sector_ + 1) % 2;
    if (sectors_[next_sector].is_bad) return StorageError::SECTOR_BAD;
    active_sector_ = next_sector;
    return true;
}

/**
 * @brief Gets the write count for a flash sector.
 * @param sector_index The index of the sector.
 * @return A StorageResult containing the write count or an error.
 */
StorageResult<uint32_t> FlashStorage::getWriteCount(uint8_t sector_index) const {
    if (sector_index >= 2) return StorageError::INVALID_PARAMETER;
    return sectors_[sector_index].write_count;
}

/**
 * @brief Checks if a flash sector is marked as bad.
 * @param sector_index The index of the sector.
 * @return true if the sector is bad, false otherwise.
 */
bool FlashStorage::isSectorBad(uint8_t sector_index) const {
    return (sector_index >= 2) || sectors_[sector_index].is_bad;
}

/**
 * @brief Gets the total number of write cycles performed on the flash.
 * @return A StorageResult containing the total write cycles or an error.
 */
StorageResult<uint32_t> FlashStorage::getTotalWriteCycles() const {
    return total_write_cycles_;
}

/**
 * @brief Gets the wear level of the flash memory.
 * @return A StorageResult containing the wear level as a percentage or an error.
 */
StorageResult<float> FlashStorage::getWearLevel() const {
    if (!initialized_) return StorageError::FLASH_ERROR;
    return (static_cast<float>(total_write_cycles_) / static_cast<float>(Config::MAX_WRITE_CYCLES)) * 100.0f;
}

/**
 * @brief Verifies the contents of a block in flash memory.
 * @param address The starting address of the block.
 * @param expected_data A pointer to the expected data.
 * @param size The size of the data to verify.
 * @return A StorageResult containing true if the data matches, false otherwise, or an error.
 */
StorageResult<bool> FlashStorage::verifyBlock(uint32_t address, const uint8_t* expected_data, size_t size) {
    if (!expected_data || size == 0) return StorageError::INVALID_PARAMETER;
    uint8_t* read_buffer = new uint8_t[size];
    auto read_result = readBlock(address, read_buffer, size);
    if (read_result.isError()) {
        delete[] read_buffer;
        return read_result.error;
    }
    bool matches = std::memcmp(read_buffer, expected_data, size) == 0;
    delete[] read_buffer;
    return matches;
}

/**
 * @brief Calculates the CRC32 checksum of a block of data.
 * @param data A pointer to the data.
 * @param size The size of the data.
 * @return A StorageResult containing the CRC32 checksum or an error.
 */
StorageResult<uint32_t> FlashStorage::calculateCRC32(const uint8_t* data, size_t size) {
    if (!data || size == 0) return StorageError::INVALID_PARAMETER;
    return updateCRC32(0xFFFFFFFF, data, size) ^ 0xFFFFFFFF;
}

StorageResult<bool> FlashStorage::unlockFlash() {
    return (HAL_FLASH_Unlock() == HAL_OK) ? StorageResult<bool>(true) : StorageResult<bool>(StorageError::FLASH_ERROR);
}

StorageResult<bool> FlashStorage::lockFlash() {
    return (HAL_FLASH_Lock() == HAL_OK) ? StorageResult<bool>(true) : StorageResult<bool>(StorageError::FLASH_ERROR);
}

StorageResult<uint32_t> FlashStorage::getSectorNumber(uint32_t address) {
    if (address >= Memory::PARAM_SECTOR_6_ADDR && address < Memory::PARAM_SECTOR_7_ADDR) return 6;
    if (address >= Memory::PARAM_SECTOR_7_ADDR && address < (Memory::PARAM_SECTOR_7_ADDR + Memory::PARAM_SECTOR_SIZE)) return 7;
    return StorageError::INVALID_PARAMETER;
}

StorageResult<bool> FlashStorage::waitForFlashReady(uint32_t timeout_ms) {
    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if ((FLASH->SR & FLASH_SR_BSY) == 0) return true;
        HAL_Delay(1);
    }
    return StorageError::TIMEOUT;
}

void FlashStorage::markSectorBad(uint8_t sector_index) {
    if (sector_index < 2) sectors_[sector_index].is_bad = true;
}

StorageResult<bool> FlashStorage::validateAddress(uint32_t address, size_t size) {
    if (address < Memory::PARAM_SECTOR_6_ADDR || address + size > Memory::PARAM_SECTOR_7_ADDR + Memory::PARAM_SECTOR_SIZE) {
        return StorageError::INVALID_PARAMETER;
    }
    return true;
}

void FlashStorage::initCRC32Table() {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc = i;
        for (uint32_t j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ CRC32_POLYNOMIAL : (crc >> 1);
        }
        crc32_table_[i] = crc;
    }
}

uint32_t FlashStorage::updateCRC32(uint32_t crc, const uint8_t* data, size_t size) {
    for (size_t i = 0; i < size; i++) {
        crc = crc32_table_[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }
    return crc;
}

} // namespace Storage