/**
 * @file parameter_storage.cpp
 * @brief Implements the high-level parameter storage manager.
 */

#include "parameter_storage.hpp"

extern "C" {
#include "stm32f4xx_hal.h"
}

#include <algorithm>
#include <cstring>

namespace Storage {

ParameterStorage g_parameter_storage;

/**
 * @brief Construct a new ParameterStorage object.
 * @param flash_storage A pointer to the flash storage driver.
 */
ParameterStorage::ParameterStorage(FlashStorage* flash_storage)
    : flash_storage_(flash_storage), cache_size_(0),
      active_block_address_(Memory::PRIMARY_BLOCK_ADDR),
      backup_block_address_(Memory::SECONDARY_BLOCK_ADDR),
      current_sequence_number_(0), last_save_time_(0),
      initialized_(false), auto_save_enabled_(Config::ENABLE_AUTO_BACKUP),
      current_auth_level_(AuthorizationLevel::USER), safety_critical_enabled_(false) {
}

/**
 * @brief Initializes the parameter storage.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> ParameterStorage::initialize() {
    if (initialized_) {
        return true;
    }

    auto flash_init = flash_storage_->initialize();
    if (flash_init.isError()) {
        return flash_init.error;
    }

    auto valid_block_result = findValidBlock();
    if (valid_block_result.isSuccess()) {
        active_block_address_ = valid_block_result.value;
        backup_block_address_ = (active_block_address_ == Memory::PRIMARY_BLOCK_ADDR) ?
                               Memory::SECONDARY_BLOCK_ADDR : Memory::PRIMARY_BLOCK_ADDR;

        auto load_result = loadParameters();
        if (load_result.isError()) {
            auto recovery_result = recoverFromCorruption();
            if (recovery_result.isError()) {
                auto defaults_result = loadDefaults();
                if (defaults_result.isError()) {
                    return defaults_result.error;
                }
            }
        }
    } else {
        auto defaults_result = loadDefaults();
        if (defaults_result.isError()) {
            return defaults_result.error;
        }
    }

    initialized_ = true;
    return true;
}

/**
 * @brief Loads the default parameters.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> ParameterStorage::loadDefaults() {
    cache_size_ = 0;
    current_sequence_number_ = 1;

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
    return saveParameters();
}

/**
 * @brief Resets all parameters to their factory default values.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> ParameterStorage::factoryReset() {
    auto erase1 = flash_storage_->eraseBlock(Memory::PRIMARY_BLOCK_ADDR);
    auto erase2 = flash_storage_->eraseBlock(Memory::SECONDARY_BLOCK_ADDR);

    if (erase1.isError()) return erase1.error;
    if (erase2.isError()) return erase2.error;

    cache_size_ = 0;
    current_sequence_number_ = 1;
    active_block_address_ = Memory::PRIMARY_BLOCK_ADDR;
    backup_block_address_ = Memory::SECONDARY_BLOCK_ADDR;

    return loadDefaults();
}

/**
 * @brief Sets the value of a parameter.
 * @param name The name of the parameter.
 * @param value The new value of the parameter.
 * @param force_save Whether to force a save to flash.
 * @return A StorageResult containing the validation result or an error.
 */
StorageResult<ValidationResult> ParameterStorage::setParameter(const char* name, float value, bool force_save) {
    return setParameterWithAuth(name, value, current_auth_level_, safety_critical_enabled_, force_save);
}

/**
 * @brief Sets the value of a parameter with a specific authorization level.
 * @param name The name of the parameter.
 * @param value The new value of the parameter.
 * @param auth_level The authorization level of the user.
 * @param allow_safety_critical Whether to allow modification of safety-critical parameters.
 * @param force_save Whether to force a save to flash.
 * @return A StorageResult containing the validation result or an error.
 */
StorageResult<ValidationResult> ParameterStorage::setParameterWithAuth(const char* name, float value,
                                                                     AuthorizationLevel auth_level,
                                                                     bool allow_safety_critical,
                                                                     bool force_save) {
    if (!name || !initialized_) return StorageError::INVALID_PARAMETER;

    auto validation = validateParameterChange(name, value, auth_level, allow_safety_critical);
    if (validation.isError()) return validation.error;
    if (validation.value != ValidationResult::SUCCESS) return validation.value;

    auto cache_entry = findInCache(name);
    if (cache_entry.isError()) return StorageError::INVALID_PARAMETER;

    ParameterCacheEntry* entry = cache_entry.value;
    if (!entry) return StorageError::INVALID_PARAMETER;

    if (active_transaction_.active) {
        auto add_result = addToTransaction(name, entry->stored_param.value, value,
                                         entry->stored_param.requiresReboot(),
                                         entry->stored_param.isSafetyCritical());
        if (add_result.isError()) return add_result.error;
        return ValidationResult::SUCCESS;
    }

    entry->stored_param.value = value;
    entry->dirty = true;
    updateAccessTime(entry);

    if (entry->setter) entry->setter(value);
    if (force_save) {
        auto save_result = saveSingleParameter(name);
        if (save_result.isError()) return save_result.error;
    }

    return ValidationResult::SUCCESS;
}

/**
 * @brief Gets the value of a parameter.
 * @param name The name of the parameter.
 * @return A StorageResult containing the parameter value or an error.
 */
StorageResult<float> ParameterStorage::getParameter(const char* name) {
    if (!name || !initialized_) return StorageError::INVALID_PARAMETER;
    auto cache_entry = findInCache(name);
    if (cache_entry.isError()) return StorageError::INVALID_PARAMETER;
    ParameterCacheEntry* entry = cache_entry.value;
    if (!entry) return StorageError::INVALID_PARAMETER;
    updateAccessTime(entry);
    if (entry->getter) {
        float current_value = entry->getter();
        if (current_value != entry->stored_param.value) {
            entry->stored_param.value = current_value;
            entry->dirty = true;
        }
        return current_value;
    }
    return entry->stored_param.value;
}

/**
 * @brief Checks if a parameter exists.
 * @param name The name of the parameter.
 * @return A StorageResult containing true if the parameter exists, false otherwise, or an error.
 */
StorageResult<bool> ParameterStorage::hasParameter(const char* name) {
    if (!name || !initialized_) return StorageError::INVALID_PARAMETER;
    auto cache_entry = findInCache(name);
    return cache_entry.isSuccess() && cache_entry.value != nullptr;
}

/**
 * @brief Registers a new parameter.
 * @param name The name of the parameter.
 * @param default_value The default value of the parameter.
 * @param type The MAVLink parameter type.
 * @param min_val The minimum allowed value.
 * @param max_val The maximum allowed value.
 * @param flags Parameter flags.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> ParameterStorage::registerParameter(const char* name, float default_value,
                                                      uint8_t type, float min_val, float max_val,
                                                      uint8_t flags) {
    if (!name || cache_size_ >= Config::CACHE_SIZE) return StorageError::INVALID_PARAMETER;
    auto existing = findInCache(name);
    if (existing.isSuccess() && existing.value != nullptr) return true;

    StoredParameter param;
    auto copy_result = copyString(param.name, name, Config::MAX_PARAM_NAME_LEN);
    if (copy_result.isError()) return copy_result.error;

    param.value = default_value;
    param.default_value = default_value;
    param.min_value = min_val;
    param.max_value = max_val;
    param.type = type;
    param.flags = flags;
    param.updateCRC();

    auto validation = validateParameter(param);
    if (validation.isError()) return validation.error;
    auto cache_result = addToCache(param);
    if (cache_result.isError()) return cache_result.error;
    return true;
}

/**
 * @brief Registers a new parameter with callbacks.
 * @param name The name of the parameter.
 * @param default_value The default value of the parameter.
 * @param setter A callback function to call when the parameter is set.
 * @param getter A callback function to call when the parameter is read.
 * @param type The MAVLink parameter type.
 * @param min_val The minimum allowed value.
 * @param max_val The maximum allowed value.
 * @param flags Parameter flags.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> ParameterStorage::registerParameterWithCallbacks(
    const char* name, float default_value,
    std::function<void(float)> setter, std::function<float()> getter,
    uint8_t type, float min_val, float max_val, uint8_t flags) {

    auto reg_result = registerParameter(name, default_value, type, min_val, max_val, flags);
    if (reg_result.isError()) return reg_result.error;

    auto cache_entry = findInCache(name);
    if (cache_entry.isSuccess() && cache_entry.value) {
        cache_entry.value->setter = setter;
        cache_entry.value->getter = getter;
    }
    return true;
}

/**
 * @brief Saves all dirty parameters to flash.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> ParameterStorage::saveParameters() {
    if (!initialized_) return StorageError::FLASH_ERROR;

    ParameterBlock block;
    block.header.param_count = cache_size_;
    block.header.timestamp = getCurrentTime();
    block.header.sequence_number = ++current_sequence_number_;

    for (uint8_t i = 0; i < cache_size_; i++) {
        if (cache_[i].loaded) {
            block.parameters[i] = cache_[i].stored_param;
            cache_[i].dirty = false;
        }
    }

    block.updateCRCs();
    auto switch_result = switchBlocks();
    if (switch_result.isError()) return switch_result.error;
    auto erase_result = flash_storage_->eraseBlock(active_block_address_);
    if (erase_result.isError()) return erase_result.error;
    auto save_result = saveBlock(block, active_block_address_);
    if (save_result.isError()) return save_result.error;
    last_save_time_ = getCurrentTime();
    return true;
}

/**
 * @brief Loads all parameters from flash.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> ParameterStorage::loadParameters() {
    if (!initialized_) return StorageError::FLASH_ERROR;
    auto block_result = loadBlock(active_block_address_);
    if (block_result.isError()) return block_result.error;
    const ParameterBlock& block = block_result.value;
    current_sequence_number_ = block.header.sequence_number;
    cache_size_ = 0;
    for (uint16_t i = 0; i < block.header.param_count && i < Config::MAX_PARAMETERS; i++) {
        if (cache_size_ >= Config::CACHE_SIZE) break;
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

/**
 * @brief Saves a single parameter to flash.
 * @param name The name of the parameter to save.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> ParameterStorage::saveSingleParameter(const char* name) {
    return saveParameters();
}

/**
 * @brief Gets the number of registered parameters.
 * @return A StorageResult containing the parameter count or an error.
 */
StorageResult<uint16_t> ParameterStorage::getParameterCount() const {
    return cache_size_;
}

/**
 * @brief Checks if any parameters are dirty (modified but not saved).
 * @return A StorageResult containing true if any parameters are dirty, false otherwise, or an error.
 */
StorageResult<bool> ParameterStorage::isDirty() const {
    for (uint8_t i = 0; i < cache_size_; i++) {
        if (cache_[i].dirty) return true;
    }
    return false;
}

/**
 * @brief Gets the health of the flash storage.
 * @return A StorageResult containing the storage health as a percentage or an error.
 */
StorageResult<float> ParameterStorage::getStorageHealth() const {
    if (!flash_storage_) return StorageError::FLASH_ERROR;
    return flash_storage_->getWearLevel();
}

/**
 * @brief Gets the timestamp of the last save operation.
 * @return A StorageResult containing the timestamp or an error.
 */
StorageResult<uint32_t> ParameterStorage::getLastSaveTime() const {
    return last_save_time_;
}

/**
 * @brief Updates the parameter storage, performing periodic tasks like auto-saving.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> ParameterStorage::update() {
    if (!initialized_) return StorageError::FLASH_ERROR;
    if (shouldAutoSave()) return saveParameters();
    return true;
}

/**
 * @brief Compacts the parameter storage.
 * @return A StorageResult indicating success or failure.
 */
StorageResult<bool> ParameterStorage::compact() {
    return saveParameters();
}

StorageResult<ParameterCacheEntry*> ParameterStorage::findInCache(const char* name) {
    if (!name) return StorageError::INVALID_PARAMETER;
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
        if (evict_result.isError()) return evict_result.error;
    }
    cache_[cache_size_].stored_param = param;
    cache_[cache_size_].dirty = true;
    cache_[cache_size_].loaded = true;
    cache_[cache_size_].last_access_time = getCurrentTime();
    return &cache_[cache_size_++];
}

StorageResult<bool> ParameterStorage::evictLRU() {
    if (cache_size_ == 0) return StorageError::STORAGE_FULL;
    uint8_t lru_index = 0;
    uint32_t oldest_time = cache_[0].last_access_time;
    for (uint8_t i = 1; i < cache_size_; i++) {
        if (cache_[i].last_access_time < oldest_time) {
            oldest_time = cache_[i].last_access_time;
            lru_index = i;
        }
    }
    if (cache_[lru_index].dirty) {
        cache_[lru_index].dirty = false;
    }
    if (lru_index < cache_size_ - 1) {
        cache_[lru_index] = cache_[cache_size_ - 1];
    }
    cache_size_--;
    return true;
}

void ParameterStorage::updateAccessTime(ParameterCacheEntry* entry) {
    if (entry) entry->last_access_time = getCurrentTime();
}

StorageResult<bool> ParameterStorage::saveBlock(const ParameterBlock& block, uint32_t address) {
    return flash_storage_->writeBlockData(address, reinterpret_cast<const uint8_t*>(&block), block.getStorageSize());
}

StorageResult<ParameterBlock> ParameterStorage::loadBlock(uint32_t address) {
    ParameterBlock block;
    auto read_result = flash_storage_->readBlockData(address, reinterpret_cast<uint8_t*>(&block), sizeof(block));
    if (read_result.isError()) return read_result.error;
    if (!block.isValid()) return StorageError::CORRUPTION_DETECTED;
    return block;
}

StorageResult<uint32_t> ParameterStorage::findValidBlock() {
    auto primary_block = loadBlock(Memory::PRIMARY_BLOCK_ADDR);
    auto secondary_block = loadBlock(Memory::SECONDARY_BLOCK_ADDR);
    bool primary_valid = primary_block.isSuccess();
    bool secondary_valid = secondary_block.isSuccess();
    if (primary_valid && secondary_valid) {
        return (primary_block.value.header.sequence_number >= secondary_block.value.header.sequence_number) ?
               Memory::PRIMARY_BLOCK_ADDR : Memory::SECONDARY_BLOCK_ADDR;
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
    uint32_t backup_addr = (active_block_address_ == Memory::PRIMARY_BLOCK_ADDR) ?
                          Memory::SECONDARY_BLOCK_ADDR : Memory::PRIMARY_BLOCK_ADDR;
    auto backup_result = loadBlock(backup_addr);
    if (backup_result.isSuccess()) {
        active_block_address_ = backup_addr;
        return loadParameters();
    }
    return loadDefaults();
}

bool ParameterStorage::shouldAutoSave() const {
    if (!auto_save_enabled_) return false;
    auto dirty_result = isDirty();
    if (dirty_result.isError() || !dirty_result.value) return false;
    return (getCurrentTime() - last_save_time_) >= Config::BACKUP_INTERVAL_MS;
}

uint32_t ParameterStorage::getCurrentTime() const {
    return HAL_GetTick();
}

StorageResult<bool> ParameterStorage::copyString(char* dest, const char* src, size_t max_len) {
    if (!dest || !src || max_len == 0) return StorageError::INVALID_PARAMETER;
    size_t len = std::strlen(src);
    if (len >= max_len) return StorageError::INVALID_PARAMETER;
    std::strncpy(dest, src, max_len - 1);
    dest[max_len - 1] = '\0';
    return true;
}

StorageResult<ValidationResult> ParameterStorage::validateParameterChange(const char* name, float value,
                                                                        AuthorizationLevel auth_level,
                                                                        bool allow_safety_critical) {
    if (!name || !initialized_) return StorageError::INVALID_PARAMETER;
    auto cache_entry = findInCache(name);
    if (cache_entry.isError()) return StorageError::INVALID_PARAMETER;
    ParameterCacheEntry* entry = cache_entry.value;
    if (!entry) return StorageError::INVALID_PARAMETER;
    const StoredParameter& param = entry->stored_param;
    auto access_result = param.validateAccess(auth_level, allow_safety_critical);
    if (access_result != ValidationResult::SUCCESS) return access_result;
    auto value_result = param.validateValue(value);
    if (value_result != ValidationResult::SUCCESS) return value_result;
    return ValidationResult::SUCCESS;
}

StorageResult<uint32_t> ParameterStorage::beginTransaction() {
    if (active_transaction_.active) return StorageError::INVALID_PARAMETER;
    active_transaction_.active = true;
    active_transaction_.change_count = 0;
    active_transaction_.transaction_id = ++current_sequence_number_;
    return active_transaction_.transaction_id;
}

StorageResult<bool> ParameterStorage::commitTransaction(uint32_t transaction_id) {
    if (!active_transaction_.active || active_transaction_.transaction_id != transaction_id) {
        return StorageError::INVALID_PARAMETER;
    }
    auto apply_result = applyTransactionChanges();
    if (apply_result.isError()) {
        clearTransaction();
        return apply_result.error;
    }
    clearTransaction();
    return true;
}

StorageResult<bool> ParameterStorage::rollbackTransaction(uint32_t transaction_id) {
    if (!active_transaction_.active || active_transaction_.transaction_id != transaction_id) {
        return StorageError::INVALID_PARAMETER;
    }
    clearTransaction();
    return true;
}

StorageResult<std::vector<ParameterChangeNotification>> ParameterStorage::analyzeParameterChange(const char* name, float new_value) {
    std::vector<ParameterChangeNotification> notifications;
    if (!name || !initialized_) return StorageError::INVALID_PARAMETER;
    auto cache_entry = findInCache(name);
    if (cache_entry.isError() || !cache_entry.value) return StorageError::INVALID_PARAMETER;
    const StoredParameter& param = cache_entry.value->stored_param;
    ParameterChangeNotification notification;
    copyString(notification.name, name, Config::MAX_PARAM_NAME_LEN);
    notification.old_value = param.value;
    notification.new_value = new_value;
    notification.requires_reboot = param.requiresReboot();
    notification.is_safety_critical = param.isSafetyCritical();
    notification.timestamp = getCurrentTime();
    notifications.push_back(notification);
    return notifications;
}

StorageResult<bool> ParameterStorage::addToTransaction(const char* name, float old_value, float new_value,
                                                     bool requires_reboot, bool is_safety_critical) {
    if (!active_transaction_.active) return StorageError::INVALID_PARAMETER;
    if (active_transaction_.change_count >= 16) return StorageError::STORAGE_FULL;
    auto& change = active_transaction_.pending_changes[active_transaction_.change_count++];
    copyString(change.name, name, Config::MAX_PARAM_NAME_LEN);
    change.old_value = old_value;
    change.new_value = new_value;
    change.requires_reboot = requires_reboot;
    change.is_safety_critical = is_safety_critical;
    change.timestamp = getCurrentTime();
    return true;
}

StorageResult<bool> ParameterStorage::applyTransactionChanges() {
    if (!active_transaction_.active) return StorageError::INVALID_PARAMETER;
    for (uint8_t i = 0; i < active_transaction_.change_count; i++) {
        const auto& change = active_transaction_.pending_changes[i];
        auto cache_entry = findInCache(change.name);
        if (cache_entry.isError() || !cache_entry.value) return StorageError::INVALID_PARAMETER;
        ParameterCacheEntry* entry = cache_entry.value;
        entry->stored_param.value = change.new_value;
        entry->dirty = true;
        updateAccessTime(entry);
        if (entry->setter) entry->setter(change.new_value);
    }
    return true;
}

void ParameterStorage::clearTransaction() {
    active_transaction_.active = false;
    active_transaction_.change_count = 0;
    active_transaction_.transaction_id = 0;
}

} // namespace Storage