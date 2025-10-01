/**
 * @file parameter_storage.hpp
 * @brief Defines the high-level parameter storage manager.
 */

#pragma once

#include "storage_config.hpp"
#include "flash_storage.hpp"

extern "C" {
#include "mavlink/c_library_v2/common/mavlink.h"
}

#include <cstdint>
#include <cstring>
#include <array>
#include <functional>
#include <vector>

namespace Storage {

/**
 * @brief Defines the authorization levels for parameter access.
 */
enum class AuthorizationLevel : uint8_t {
    USER = 0,
    ADMIN = 1,
    FACTORY = 2
};

/**
 * @brief Defines the result of a parameter validation check.
 */
enum class ValidationResult : uint8_t {
    SUCCESS = 0,
    OUT_OF_RANGE = 1,
    INSUFFICIENT_AUTHORIZATION = 2,
    SAFETY_CRITICAL_DENIED = 3,
    READ_ONLY_VIOLATION = 4,
    ATOMIC_TRANSACTION_REQUIRED = 5
};

/**
 * @brief Represents a parameter as it is stored in flash memory.
 */
struct __attribute__((packed)) StoredParameter {
    char name[Config::MAX_PARAM_NAME_LEN];
    float value;
    float default_value;
    float min_value;
    float max_value;
    uint8_t type;
    uint8_t flags;
    uint16_t reserved;
    uint32_t crc32;

    static constexpr uint8_t FLAG_READ_ONLY = 0x01;
    static constexpr uint8_t FLAG_CRITICAL = 0x02;
    static constexpr uint8_t FLAG_PERSISTENT = 0x04;
    static constexpr uint8_t FLAG_REQUIRES_REBOOT = 0x08;
    static constexpr uint8_t FLAG_USER_LEVEL = 0x10;
    static constexpr uint8_t FLAG_ADMIN_LEVEL = 0x20;
    static constexpr uint8_t FLAG_FACTORY_LEVEL = 0x40;
    static constexpr uint8_t FLAG_SAFETY_CRITICAL = 0x80;

    StoredParameter();
    bool isValid() const;
    bool isReadOnly() const;
    bool isCritical() const;
    bool isPersistent() const;
    bool requiresReboot() const;
    bool isUserLevel() const;
    bool isAdminLevel() const;
    bool isFactoryLevel() const;
    bool isSafetyCritical() const;
    AuthorizationLevel getRequiredAuthLevel() const;
    ValidationResult validateAccess(AuthorizationLevel userLevel, bool allowSafetyCritical = false) const;
    ValidationResult validateValue(float new_value) const;
    void updateCRC();
    bool verifyCRC() const;
};

/**
 * @brief Represents the header of a parameter block in flash memory.
 */
struct __attribute__((packed)) ParameterBlockHeader {
    uint32_t magic;
    uint16_t version;
    uint16_t param_count;
    uint32_t timestamp;
    uint32_t sequence_number;
    uint32_t header_crc;
    uint32_t data_crc;
    uint8_t reserved[16];

    ParameterBlockHeader();
    bool isValid() const;
    void updateHeaderCRC();
    bool verifyHeaderCRC() const;
};

/**
 * @brief Represents a complete block of parameters for storage.
 */
struct ParameterBlock {
    ParameterBlockHeader header;
    StoredParameter parameters[Config::MAX_PARAMETERS];

    ParameterBlock() = default;
    bool isValid() const;
    void updateCRCs();
    size_t getStorageSize() const;
};

/**
 * @brief Represents a parameter cache entry in RAM.
 */
struct ParameterCacheEntry {
    StoredParameter stored_param;
    bool dirty;
    bool loaded;
    uint32_t last_access_time;
    std::function<void(float)> setter;
    std::function<float()> getter;

    ParameterCacheEntry();
};

/**
 * @brief Represents a notification of a parameter change.
 */
struct ParameterChangeNotification {
    char name[Config::MAX_PARAM_NAME_LEN];
    float old_value;
    float new_value;
    bool requires_reboot;
    bool is_safety_critical;
    uint32_t timestamp;
};

/**
 * @brief Represents the context of an atomic parameter transaction.
 */
struct AtomicTransaction {
    bool active;
    ParameterChangeNotification pending_changes[16];
    uint8_t change_count;
    uint32_t transaction_id;

    AtomicTransaction();
};

/**
 * @brief Manages the storage of parameters in flash memory.
 */
class ParameterStorage {
private:
    FlashStorage* flash_storage_;
    ParameterCacheEntry cache_[Config::CACHE_SIZE];
    uint8_t cache_size_;
    uint32_t active_block_address_;
    uint32_t backup_block_address_;
    uint32_t current_sequence_number_;
    uint32_t last_save_time_;
    bool initialized_;
    bool auto_save_enabled_;
    AuthorizationLevel current_auth_level_;
    bool safety_critical_enabled_;
    AtomicTransaction active_transaction_;

public:
    ParameterStorage(FlashStorage* flash_storage = &g_flash_storage);
    ~ParameterStorage() = default;

    StorageResult<bool> initialize();
    StorageResult<bool> loadDefaults();
    StorageResult<bool> factoryReset();
    StorageResult<ValidationResult> setParameter(const char* name, float value, bool force_save = false);
    StorageResult<ValidationResult> setParameterWithAuth(const char* name, float value,
                                                       AuthorizationLevel auth_level,
                                                       bool allow_safety_critical = false,
                                                       bool force_save = false);
    StorageResult<float> getParameter(const char* name);
    StorageResult<bool> hasParameter(const char* name);
    StorageResult<bool> registerParameter(const char* name, float default_value,
                                        uint8_t type = MAV_PARAM_TYPE_REAL32,
                                        float min_val = -1000000.0f, float max_val = 1000000.0f,
                                        uint8_t flags = StoredParameter::FLAG_PERSISTENT);
    StorageResult<bool> registerParameterWithCallbacks(const char* name, float default_value,
                                                      std::function<void(float)> setter,
                                                      std::function<float()> getter,
                                                      uint8_t type = MAV_PARAM_TYPE_REAL32,
                                                      float min_val = -1000000.0f, float max_val = 1000000.0f,
                                                      uint8_t flags = StoredParameter::FLAG_PERSISTENT);
    StorageResult<bool> saveParameters();
    StorageResult<bool> loadParameters();
    StorageResult<bool> saveSingleParameter(const char* name);
    StorageResult<uint16_t> getParameterCount() const;
    StorageResult<bool> isDirty() const;
    StorageResult<float> getStorageHealth() const;
    StorageResult<uint32_t> getLastSaveTime() const;
    void setAuthorizationLevel(AuthorizationLevel level);
    AuthorizationLevel getAuthorizationLevel() const;
    void setSafetyCriticalEnabled(bool enabled);
    bool isSafetyCriticalEnabled() const;
    StorageResult<uint32_t> beginTransaction();
    StorageResult<bool> commitTransaction(uint32_t transaction_id);
    StorageResult<bool> rollbackTransaction(uint32_t transaction_id);
    bool isTransactionActive() const;
    StorageResult<std::vector<ParameterChangeNotification>> analyzeParameterChange(const char* name, float new_value);
    void setAutoSave(bool enabled);
    bool isAutoSaveEnabled() const;
    StorageResult<bool> update();
    StorageResult<bool> compact();

private:
    StorageResult<ParameterCacheEntry*> findInCache(const char* name);
    StorageResult<ParameterCacheEntry*> addToCache(const StoredParameter& param);
    StorageResult<bool> evictLRU();
    void updateAccessTime(ParameterCacheEntry* entry);
    StorageResult<bool> saveBlock(const ParameterBlock& block, uint32_t address);
    StorageResult<ParameterBlock> loadBlock(uint32_t address);
    StorageResult<uint32_t> findValidBlock();
    StorageResult<bool> switchBlocks();
    StorageResult<bool> validateParameter(const StoredParameter& param);
    StorageResult<ValidationResult> validateParameterChange(const char* name, float value,
                                                          AuthorizationLevel auth_level,
                                                          bool allow_safety_critical);
    StorageResult<bool> recoverFromCorruption();
    bool shouldAutoSave() const;
    StorageResult<bool> addToTransaction(const char* name, float old_value, float new_value, bool requires_reboot, bool is_safety_critical);
    StorageResult<bool> applyTransactionChanges();
    void clearTransaction();
    uint32_t getCurrentTime() const;
    StorageResult<bool> copyString(char* dest, const char* src, size_t max_len);
};

extern ParameterStorage g_parameter_storage;

} // namespace Storage