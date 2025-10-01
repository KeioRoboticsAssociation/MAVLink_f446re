#include "parameter_storage.hpp"

extern "C" {
#include "stm32f4xx_hal.h"
}

#include <algorithm>
#include <cstring>

namespace Storage {

// Global instance
ParameterStorage g_parameter_storage;

ParameterStorage::ParameterStorage(FlashStorage* flash_storage)
    : flash_storage_(flash_storage), cache_size_(0),
      active_block_address_(Memory::PRIMARY_BLOCK_ADDR),
      backup_block_address_(Memory::SECONDARY_BLOCK_ADDR),
      current_sequence_number_(0), last_save_time_(0),
      initialized_(false), auto_save_enabled_(Config::ENABLE_AUTO_BACKUP) {
}

StorageResult<bool> ParameterStorage::initialize() {
    if (initialized_) {
        return true;
    }

    // Initialize flash storage
    auto flash_init = flash_storage_->initialize();
    if (flash_init.isError()) {
        return flash_init.error;
    }

    // Find the most recent valid parameter block
    auto valid_block_result = findValidBlock();
    if (valid_block_result.isSuccess()) {
        active_block_address_ = valid_block_result.value;
        backup_block_address_ = (active_block_address_ == Memory::PRIMARY_BLOCK_ADDR) ?
                               Memory::SECONDARY_BLOCK_ADDR : Memory::PRIMARY_BLOCK_ADDR;

        // Load parameters from the valid block
        auto load_result = loadParameters();
        if (load_result.isError()) {
            // If loading fails, try to recover or load defaults
            auto recovery_result = recoverFromCorruption();
            if (recovery_result.isError()) {
                auto defaults_result = loadDefaults();
                if (defaults_result.isError()) {
                    return defaults_result.error;
                }
            }
        }
    } else {
        // No valid block found, load defaults
        auto defaults_result = loadDefaults();
        if (defaults_result.isError()) {
            return defaults_result.error;
        }
    }

    initialized_ = true;
    return true;
}

StorageResult<bool> ParameterStorage::loadDefaults() {
    // Clear cache
    cache_size_ = 0;
    current_sequence_number_ = 1;

    // Register default system parameters
    auto results = {
        registerParameter("SYS_ID", 1.0f, MAV_PARAM_TYPE_UINT8, 1.0f, 255.0f),
        registerParameter("COMP_ID", 1.0f, MAV_PARAM_TYPE_UINT8, 1.0f, 255.0f),
        registerParameter("HEARTBEAT_RATE", 1.0f, MAV_PARAM_TYPE_REAL32, 0.1f, 10.0f),
        registerParameter("TELEMETRY_RATE", 10.0f, MAV_PARAM_TYPE_REAL32, 1.0f, 100.0f),
        registerParameter("AUTO_SAVE", 1.0f, MAV_PARAM_TYPE_UINT8, 0.0f, 1.0f)
    };

    for (const auto& result : results) {
        if (result.isError()) {
            return result.error;
        }
    }

    // Save defaults to flash
    return saveParameters();
}

StorageResult<bool> ParameterStorage::factoryReset() {
    // Clear flash blocks
    auto erase1 = flash_storage_->eraseBlock(Memory::PRIMARY_BLOCK_ADDR);
    auto erase2 = flash_storage_->eraseBlock(Memory::SECONDARY_BLOCK_ADDR);

    if (erase1.isError()) return erase1.error;
    if (erase2.isError()) return erase2.error;

    // Reset internal state
    cache_size_ = 0;
    current_sequence_number_ = 1;
    active_block_address_ = Memory::PRIMARY_BLOCK_ADDR;
    backup_block_address_ = Memory::SECONDARY_BLOCK_ADDR;

    // Load defaults
    return loadDefaults();
}

StorageResult<bool> ParameterStorage::setParameter(const char* name, float value, bool force_save) {
    if (!name || !initialized_) {
        return StorageError::INVALID_PARAMETER;
    }

    // Find parameter in cache
    auto cache_entry = findInCache(name);
    if (cache_entry.isError()) {
        return StorageError::INVALID_PARAMETER;
    }

    ParameterCacheEntry* entry = cache_entry.value;
    if (!entry) {
        return StorageError::INVALID_PARAMETER;
    }

    // Validate new value
    if (value < entry->stored_param.min_value || value > entry->stored_param.max_value) {
        return StorageError::INVALID_PARAMETER;
    }

    // Check if read-only
    if (entry->stored_param.isReadOnly()) {
        return StorageError::WRITE_PROTECTED;
    }

    // Update value
    entry->stored_param.value = value;
    entry->dirty = true;
    updateAccessTime(entry);

    // Call setter callback if available
    if (entry->setter) {
        entry->setter(value);
    }

    // Force save if requested
    if (force_save) {
        return saveSingleParameter(name);
    }

    return true;
}

StorageResult<float> ParameterStorage::getParameter(const char* name) {
    if (!name || !initialized_) {
        return StorageError::INVALID_PARAMETER;
    }

    auto cache_entry = findInCache(name);
    if (cache_entry.isError()) {
        return StorageError::INVALID_PARAMETER;
    }

    ParameterCacheEntry* entry = cache_entry.value;
    if (!entry) {
        return StorageError::INVALID_PARAMETER;
    }

    updateAccessTime(entry);

    // Use getter callback if available
    if (entry->getter) {
        float current_value = entry->getter();
        // Update cached value if it changed
        if (current_value != entry->stored_param.value) {
            entry->stored_param.value = current_value;
            entry->dirty = true;
        }
        return current_value;
    }

    return entry->stored_param.value;
}

StorageResult<bool> ParameterStorage::hasParameter(const char* name) {
    if (!name || !initialized_) {
        return StorageError::INVALID_PARAMETER;
    }

    auto cache_entry = findInCache(name);
    return cache_entry.isSuccess() && cache_entry.value != nullptr;
}

StorageResult<bool> ParameterStorage::registerParameter(const char* name, float default_value,
                                                      uint8_t type, float min_val, float max_val,
                                                      uint8_t flags) {
    if (!name || cache_size_ >= Config::CACHE_SIZE) {
        return StorageError::INVALID_PARAMETER;
    }

    // Check if parameter already exists
    auto existing = findInCache(name);
    if (existing.isSuccess() && existing.value != nullptr) {
        return true; // Already registered
    }

    // Create new parameter
    StoredParameter param;
    auto copy_result = copyString(param.name, name, Config::MAX_PARAM_NAME_LEN);
    if (copy_result.isError()) {
        return copy_result.error;
    }

    param.value = default_value;
    param.default_value = default_value;
    param.min_value = min_val;
    param.max_value = max_val;
    param.type = type;
    param.flags = flags;
    param.updateCRC();

    // Validate parameter
    auto validation = validateParameter(param);
    if (validation.isError()) {
        return validation.error;
    }

    // Add to cache
    auto cache_result = addToCache(param);
    if (cache_result.isError()) {
        return cache_result.error;
    }

    return true;
}

StorageResult<bool> ParameterStorage::registerParameterWithCallbacks(
    const char* name, float default_value,
    std::function<void(float)> setter, std::function<float()> getter,
    uint8_t type, float min_val, float max_val, uint8_t flags) {

    auto reg_result = registerParameter(name, default_value, type, min_val, max_val, flags);
    if (reg_result.isError()) {
        return reg_result.error;
    }

    // Add callbacks
    auto cache_entry = findInCache(name);
    if (cache_entry.isSuccess() && cache_entry.value) {
        cache_entry.value->setter = setter;
        cache_entry.value->getter = getter;
    }

    return true;
}

StorageResult<bool> ParameterStorage::saveParameters() {
    if (!initialized_) {
        return StorageError::FLASH_ERROR;
    }

    // Create parameter block
    ParameterBlock block;
    block.header.param_count = cache_size_;
    block.header.timestamp = getCurrentTime();
    block.header.sequence_number = ++current_sequence_number_;

    // Copy parameters from cache
    for (uint8_t i = 0; i < cache_size_; i++) {
        if (cache_[i].loaded) {
            block.parameters[i] = cache_[i].stored_param;
            cache_[i].dirty = false;
        }
    }

    // Update CRCs
    block.updateCRCs();

    // Switch to backup block for wear leveling
    auto switch_result = switchBlocks();
    if (switch_result.isError()) {
        return switch_result.error;
    }

    // Erase and write new block
    auto erase_result = flash_storage_->eraseBlock(active_block_address_);
    if (erase_result.isError()) {
        return erase_result.error;
    }

    auto save_result = saveBlock(block, active_block_address_);
    if (save_result.isError()) {
        return save_result.error;
    }

    last_save_time_ = getCurrentTime();
    return true;
}

StorageResult<bool> ParameterStorage::loadParameters() {
    if (!initialized_) {
        return StorageError::FLASH_ERROR;
    }

    auto block_result = loadBlock(active_block_address_);
    if (block_result.isError()) {
        return block_result.error;
    }

    const ParameterBlock& block = block_result.value;
    current_sequence_number_ = block.header.sequence_number;

    // Load parameters into cache
    cache_size_ = 0;
    for (uint16_t i = 0; i < block.header.param_count && i < Config::MAX_PARAMETERS; i++) {
        if (cache_size_ >= Config::CACHE_SIZE) {
            break;
        }

        const StoredParameter& param = block.parameters[i];
        if (param.isValid() && param.verifyCRC()) {
            cache_[cache_size_].stored_param = param;
            cache_[cache_size_].dirty = false;
            cache_[cache_size_].loaded = true;
            cache_[cache_size_].last_access_time = getCurrentTime();
            cache_size_++;
        }
    }

    return true;
}

StorageResult<bool> ParameterStorage::saveSingleParameter(const char* name) {
    // For simplicity, save all parameters
    // In a production system, you might implement delta saves
    return saveParameters();
}

StorageResult<uint16_t> ParameterStorage::getParameterCount() const {
    return cache_size_;
}

StorageResult<bool> ParameterStorage::isDirty() const {
    for (uint8_t i = 0; i < cache_size_; i++) {
        if (cache_[i].dirty) {
            return true;
        }
    }
    return false;
}

StorageResult<float> ParameterStorage::getStorageHealth() const {
    if (!flash_storage_) {
        return StorageError::FLASH_ERROR;
    }

    return flash_storage_->getWearLevel();
}

StorageResult<uint32_t> ParameterStorage::getLastSaveTime() const {
    return last_save_time_;
}

StorageResult<bool> ParameterStorage::update() {
    if (!initialized_) {
        return StorageError::FLASH_ERROR;
    }

    // Check for auto-save
    if (shouldAutoSave()) {
        return saveParameters();
    }

    return true;
}

StorageResult<bool> ParameterStorage::compact() {
    // For current implementation, compaction is the same as saving
    return saveParameters();
}

// Private methods

StorageResult<ParameterCacheEntry*> ParameterStorage::findInCache(const char* name) {
    if (!name) {
        return StorageError::INVALID_PARAMETER;
    }

    for (uint8_t i = 0; i < cache_size_; i++) {
        if (cache_[i].loaded && std::strncmp(cache_[i].stored_param.name, name, Config::MAX_PARAM_NAME_LEN) == 0) {
            return &cache_[i];
        }
    }

    return static_cast<ParameterCacheEntry*>(nullptr);
}

StorageResult<ParameterCacheEntry*> ParameterStorage::addToCache(const StoredParameter& param) {
    if (cache_size_ >= Config::CACHE_SIZE) {
        auto evict_result = evictLRU();
        if (evict_result.isError()) {
            return evict_result.error;
        }
    }

    cache_[cache_size_].stored_param = param;
    cache_[cache_size_].dirty = true;
    cache_[cache_size_].loaded = true;
    cache_[cache_size_].last_access_time = getCurrentTime();

    return &cache_[cache_size_++];
}

StorageResult<bool> ParameterStorage::evictLRU() {
    if (cache_size_ == 0) {
        return StorageError::STORAGE_FULL;
    }

    // Find least recently used entry
    uint8_t lru_index = 0;
    uint32_t oldest_time = cache_[0].last_access_time;

    for (uint8_t i = 1; i < cache_size_; i++) {
        if (cache_[i].last_access_time < oldest_time) {
            oldest_time = cache_[i].last_access_time;
            lru_index = i;
        }
    }

    // Save if dirty
    if (cache_[lru_index].dirty) {
        // In a full implementation, you'd save just this parameter
        // For simplicity, we'll mark it as not dirty
        cache_[lru_index].dirty = false;
    }

    // Move last entry to evicted position
    if (lru_index < cache_size_ - 1) {
        cache_[lru_index] = cache_[cache_size_ - 1];
    }

    cache_size_--;
    return true;
}

void ParameterStorage::updateAccessTime(ParameterCacheEntry* entry) {
    if (entry) {
        entry->last_access_time = getCurrentTime();
    }
}

StorageResult<bool> ParameterStorage::saveBlock(const ParameterBlock& block, uint32_t address) {
    size_t block_size = block.getStorageSize();
    return flash_storage_->writeBlockData(address, reinterpret_cast<const uint8_t*>(&block), block_size);
}

StorageResult<ParameterBlock> ParameterStorage::loadBlock(uint32_t address) {
    ParameterBlock block;
    auto read_result = flash_storage_->readBlockData(address, reinterpret_cast<uint8_t*>(&block), sizeof(block));

    if (read_result.isError()) {
        return read_result.error;
    }

    if (!block.isValid()) {
        return StorageError::CORRUPTION_DETECTED;
    }

    return block;
}

StorageResult<uint32_t> ParameterStorage::findValidBlock() {
    auto primary_block = loadBlock(Memory::PRIMARY_BLOCK_ADDR);
    auto secondary_block = loadBlock(Memory::SECONDARY_BLOCK_ADDR);

    bool primary_valid = primary_block.isSuccess();
    bool secondary_valid = secondary_block.isSuccess();

    if (primary_valid && secondary_valid) {
        // Choose the one with higher sequence number
        if (primary_block.value.header.sequence_number >= secondary_block.value.header.sequence_number) {
            return Memory::PRIMARY_BLOCK_ADDR;
        } else {
            return Memory::SECONDARY_BLOCK_ADDR;
        }
    } else if (primary_valid) {
        return Memory::PRIMARY_BLOCK_ADDR;
    } else if (secondary_valid) {
        return Memory::SECONDARY_BLOCK_ADDR;
    }

    return StorageError::CORRUPTION_DETECTED;
}

StorageResult<bool> ParameterStorage::switchBlocks() {
    backup_block_address_ = active_block_address_;
    active_block_address_ = (active_block_address_ == Memory::PRIMARY_BLOCK_ADDR) ?
                           Memory::SECONDARY_BLOCK_ADDR : Memory::PRIMARY_BLOCK_ADDR;
    return true;
}

StorageResult<bool> ParameterStorage::validateParameter(const StoredParameter& param) {
    return param.isValid() ? StorageResult<bool>(true) : StorageResult<bool>(StorageError::INVALID_PARAMETER);
}

StorageResult<bool> ParameterStorage::recoverFromCorruption() {
    // Try to load from backup block
    uint32_t backup_addr = (active_block_address_ == Memory::PRIMARY_BLOCK_ADDR) ?
                          Memory::SECONDARY_BLOCK_ADDR : Memory::PRIMARY_BLOCK_ADDR;

    auto backup_result = loadBlock(backup_addr);
    if (backup_result.isSuccess()) {
        active_block_address_ = backup_addr;
        return loadParameters();
    }

    // Both blocks corrupted, load defaults
    return loadDefaults();
}

bool ParameterStorage::shouldAutoSave() const {
    if (!auto_save_enabled_ || !isDirty().value) {
        return false;
    }

    uint32_t current_time = getCurrentTime();
    return (current_time - last_save_time_) >= Config::BACKUP_INTERVAL_MS;
}

uint32_t ParameterStorage::getCurrentTime() const {
    return HAL_GetTick();
}

StorageResult<bool> ParameterStorage::copyString(char* dest, const char* src, size_t max_len) {
    if (!dest || !src || max_len == 0) {
        return StorageError::INVALID_PARAMETER;
    }

    size_t len = std::strlen(src);
    if (len >= max_len) {
        return StorageError::INVALID_PARAMETER;
    }

    std::strncpy(dest, src, max_len - 1);
    dest[max_len - 1] = '\0';
    return true;
}

} // namespace Storage