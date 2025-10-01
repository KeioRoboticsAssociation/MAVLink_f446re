#include "flash_storage.hpp"

extern "C" {
#include "stm32f4xx_hal.h"
}

namespace Storage {

// Global instance
FlashStorage g_flash_storage;

FlashStorage::FlashStorage()
    : active_sector_(0), total_write_cycles_(0), initialized_(false) {

    // Initialize sector information
    sectors_[0] = {
        .address = Memory::PARAM_SECTOR_6_ADDR,
        .size = Memory::PARAM_SECTOR_SIZE,
        .sector_number = 6,
        .write_count = 0,
        .is_bad = false
    };

    sectors_[1] = {
        .address = Memory::PARAM_SECTOR_7_ADDR,
        .size = Memory::PARAM_SECTOR_SIZE,
        .sector_number = 7,
        .write_count = 0,
        .is_bad = false
    };

    initCRC32Table();
}

StorageResult<bool> FlashStorage::initialize() {
    if (initialized_) {
        return true;
    }

    // Validate memory layout
    if (Memory::PARAM_SECTOR_6_ADDR < Memory::FLASH_BASE_ADDR ||
        Memory::PARAM_SECTOR_7_ADDR >= (Memory::FLASH_BASE_ADDR + Memory::FLASH_SIZE)) {
        return StorageError::INVALID_PARAMETER;
    }

    // Check sector health by reading first few bytes
    uint32_t test_data;
    for (uint8_t i = 0; i < 2; i++) {
        auto result = readBlock(sectors_[i].address, reinterpret_cast<uint8_t*>(&test_data), sizeof(test_data));
        if (result.isError()) {
            markSectorBad(i);
        }
    }

    // Ensure at least one sector is good
    if (sectors_[0].is_bad && sectors_[1].is_bad) {
        return StorageError::SECTOR_BAD;
    }

    // Select the first good sector as active
    active_sector_ = sectors_[0].is_bad ? 1 : 0;
    initialized_ = true;

    return true;
}

StorageResult<bool> FlashStorage::eraseSector(uint8_t sector_index) {
    if (sector_index >= 2) {
        return StorageError::INVALID_PARAMETER;
    }

    if (sectors_[sector_index].is_bad) {
        return StorageError::SECTOR_BAD;
    }

    auto unlock_result = unlockFlash();
    if (unlock_result.isError()) {
        return unlock_result.error;
    }

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

StorageResult<bool> FlashStorage::writeBlock(uint32_t address, const uint8_t* data, size_t size) {
    if (!data || size == 0) {
        return StorageError::INVALID_PARAMETER;
    }

    auto validation = validateAddress(address, size);
    if (validation.isError()) {
        return validation.error;
    }

    auto unlock_result = unlockFlash();
    if (unlock_result.isError()) {
        return unlock_result.error;
    }

    // Write data in 32-bit words (STM32F4 requirement)
    const uint32_t* word_data = reinterpret_cast<const uint32_t*>(data);
    size_t word_count = (size + 3) / 4;  // Round up to word boundary

    HAL_StatusTypeDef status = HAL_OK;
    for (size_t i = 0; i < word_count && status == HAL_OK; i++) {
        uint32_t word_to_write = (i * 4 + 3 < size) ? word_data[i] : 0xFFFFFFFF;

        // Handle partial last word
        if (i == word_count - 1 && size % 4 != 0) {
            const uint8_t* byte_data = data + i * 4;
            word_to_write = 0xFFFFFFFF;
            for (size_t j = 0; j < size % 4; j++) {
                word_to_write = (word_to_write & ~(0xFF << (j * 8))) | (byte_data[j] << (j * 8));
            }
        }

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i * 4, word_to_write);

        if (status != HAL_OK) {
            break;
        }
    }

    auto lock_result = lockFlash();

    if (status != HAL_OK) {
        return StorageError::FLASH_ERROR;
    }

    return lock_result;
}

StorageResult<bool> FlashStorage::readBlock(uint32_t address, uint8_t* data, size_t size) {
    if (!data || size == 0) {
        return StorageError::INVALID_PARAMETER;
    }

    auto validation = validateAddress(address, size);
    if (validation.isError()) {
        return validation.error;
    }

    // Direct memory read from flash
    const uint8_t* flash_ptr = reinterpret_cast<const uint8_t*>(address);
    std::memcpy(data, flash_ptr, size);

    return true;
}

StorageResult<bool> FlashStorage::eraseBlock(uint32_t block_address) {
    // For STM32F4, we need to erase entire sectors
    // This is a simplified implementation - in practice, you'd want
    // to track which blocks are dirty and erase sectors only when needed

    uint8_t sector_index = 0;
    if (block_address >= Memory::PARAM_SECTOR_7_ADDR) {
        sector_index = 1;
    }

    return eraseSector(sector_index);
}

StorageResult<bool> FlashStorage::writeBlockData(uint32_t block_address, const uint8_t* data, size_t size) {
    if (size > Memory::PARAM_BLOCK_SIZE) {
        return StorageError::INVALID_PARAMETER;
    }

    return writeBlock(block_address, data, size);
}

StorageResult<bool> FlashStorage::readBlockData(uint32_t block_address, uint8_t* data, size_t size) {
    if (size > Memory::PARAM_BLOCK_SIZE) {
        return StorageError::INVALID_PARAMETER;
    }

    return readBlock(block_address, data, size);
}

StorageResult<uint8_t> FlashStorage::getActiveSector() const {
    if (!initialized_) {
        return StorageError::FLASH_ERROR;
    }
    return active_sector_;
}

StorageResult<bool> FlashStorage::switchToNextSector() {
    if (!initialized_) {
        return StorageError::FLASH_ERROR;
    }

    uint8_t next_sector = (active_sector_ + 1) % 2;

    if (sectors_[next_sector].is_bad) {
        return StorageError::SECTOR_BAD;
    }

    active_sector_ = next_sector;
    return true;
}

StorageResult<uint32_t> FlashStorage::getWriteCount(uint8_t sector_index) const {
    if (sector_index >= 2) {
        return StorageError::INVALID_PARAMETER;
    }
    return sectors_[sector_index].write_count;
}

bool FlashStorage::isSectorBad(uint8_t sector_index) const {
    if (sector_index >= 2) {
        return true;
    }
    return sectors_[sector_index].is_bad;
}

StorageResult<uint32_t> FlashStorage::getTotalWriteCycles() const {
    return total_write_cycles_;
}

StorageResult<float> FlashStorage::getWearLevel() const {
    if (!initialized_) {
        return StorageError::FLASH_ERROR;
    }

    float max_cycles = static_cast<float>(Config::MAX_WRITE_CYCLES);
    float current_cycles = static_cast<float>(total_write_cycles_);

    return (current_cycles / max_cycles) * 100.0f;
}

StorageResult<bool> FlashStorage::verifyBlock(uint32_t address, const uint8_t* expected_data, size_t size) {
    if (!expected_data || size == 0) {
        return StorageError::INVALID_PARAMETER;
    }

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

StorageResult<uint32_t> FlashStorage::calculateCRC32(const uint8_t* data, size_t size) {
    if (!data || size == 0) {
        return StorageError::INVALID_PARAMETER;
    }

    uint32_t crc = 0xFFFFFFFF;
    crc = updateCRC32(crc, data, size);
    return crc ^ 0xFFFFFFFF;
}

// Private methods

StorageResult<bool> FlashStorage::unlockFlash() {
    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    return (status == HAL_OK) ? StorageResult<bool>(true) : StorageResult<bool>(StorageError::FLASH_ERROR);
}

StorageResult<bool> FlashStorage::lockFlash() {
    HAL_StatusTypeDef status = HAL_FLASH_Lock();
    return (status == HAL_OK) ? StorageResult<bool>(true) : StorageResult<bool>(StorageError::FLASH_ERROR);
}

StorageResult<uint32_t> FlashStorage::getSectorNumber(uint32_t address) {
    if (address >= Memory::PARAM_SECTOR_6_ADDR && address < Memory::PARAM_SECTOR_7_ADDR) {
        return 6;
    }
    if (address >= Memory::PARAM_SECTOR_7_ADDR && address < (Memory::PARAM_SECTOR_7_ADDR + Memory::PARAM_SECTOR_SIZE)) {
        return 7;
    }
    return StorageError::INVALID_PARAMETER;
}

StorageResult<bool> FlashStorage::waitForFlashReady(uint32_t timeout_ms) {
    uint32_t start_time = HAL_GetTick();

    while (HAL_GetTick() - start_time < timeout_ms) {
        if ((FLASH->SR & FLASH_SR_BSY) == 0) {
            return true;
        }
        HAL_Delay(1);
    }

    return StorageError::TIMEOUT;
}

void FlashStorage::markSectorBad(uint8_t sector_index) {
    if (sector_index < 2) {
        sectors_[sector_index].is_bad = true;
    }
}

StorageResult<bool> FlashStorage::validateAddress(uint32_t address, size_t size) {
    if (address < Memory::PARAM_SECTOR_6_ADDR ||
        address + size > Memory::PARAM_SECTOR_7_ADDR + Memory::PARAM_SECTOR_SIZE) {
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