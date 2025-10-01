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

// Authorization levels for parameter access
enum class AuthorizationLevel : uint8_t {
    USER = 0,       // Basic user level - can modify USER_LEVEL parameters
    ADMIN = 1,      // Administrator level - can modify USER and ADMIN level parameters
    FACTORY = 2     // Factory level - can modify all parameters including FACTORY_LEVEL
};

// Parameter validation result
enum class ValidationResult : uint8_t {
    SUCCESS = 0,
    OUT_OF_RANGE = 1,
    INSUFFICIENT_AUTHORIZATION = 2,
    SAFETY_CRITICAL_DENIED = 3,
    READ_ONLY_VIOLATION = 4,
    ATOMIC_TRANSACTION_REQUIRED = 5
};

// Stored parameter structure (packed for flash storage)
struct __attribute__((packed)) StoredParameter {
    char name[Config::MAX_PARAM_NAME_LEN];  // Null-terminated parameter name
    float value;                            // Current parameter value
    float default_value;                    // Factory default value
    float min_value;                        // Minimum allowed value
    float max_value;                        // Maximum allowed value
    uint8_t type;                          // MAVLink parameter type
    uint8_t flags;                         // Parameter flags (read-only, critical, etc.)
    uint16_t reserved;                     // Reserved for future use
    uint32_t crc32;                        // Parameter integrity check

    // Parameter flags
    static constexpr uint8_t FLAG_READ_ONLY = 0x01;
    static constexpr uint8_t FLAG_CRITICAL = 0x02;
    static constexpr uint8_t FLAG_PERSISTENT = 0x04;
    static constexpr uint8_t FLAG_REQUIRES_REBOOT = 0x08;
    static constexpr uint8_t FLAG_USER_LEVEL = 0x10;
    static constexpr uint8_t FLAG_ADMIN_LEVEL = 0x20;
    static constexpr uint8_t FLAG_FACTORY_LEVEL = 0x40;
    static constexpr uint8_t FLAG_SAFETY_CRITICAL = 0x80;

    StoredParameter() {
        std::memset(this, 0, sizeof(*this));
        min_value = -1000000.0f;
        max_value = 1000000.0f;
        type = MAV_PARAM_TYPE_REAL32;
        flags = FLAG_PERSISTENT;
    }

    bool isValid() const {
        // Check name is null-terminated
        bool name_valid = false;
        for (uint8_t i = 0; i < Config::MAX_PARAM_NAME_LEN; i++) {
            if (name[i] == '\0') {
                name_valid = true;
                break;
            }
        }

        return name_valid &&
               value >= min_value &&
               value <= max_value &&
               type <= MAV_PARAM_TYPE_UINT64;
    }

    bool isReadOnly() const { return (flags & FLAG_READ_ONLY) != 0; }
    bool isCritical() const { return (flags & FLAG_CRITICAL) != 0; }
    bool isPersistent() const { return (flags & FLAG_PERSISTENT) != 0; }
    bool requiresReboot() const { return (flags & FLAG_REQUIRES_REBOOT) != 0; }
    bool isUserLevel() const { return (flags & FLAG_USER_LEVEL) != 0; }
    bool isAdminLevel() const { return (flags & FLAG_ADMIN_LEVEL) != 0; }
    bool isFactoryLevel() const { return (flags & FLAG_FACTORY_LEVEL) != 0; }
    bool isSafetyCritical() const { return (flags & FLAG_SAFETY_CRITICAL) != 0; }

    // Authorization validation
    AuthorizationLevel getRequiredAuthLevel() const {
        if (flags & FLAG_FACTORY_LEVEL) return AuthorizationLevel::FACTORY;
        if (flags & FLAG_ADMIN_LEVEL) return AuthorizationLevel::ADMIN;
        return AuthorizationLevel::USER;
    }

    ValidationResult validateAccess(AuthorizationLevel userLevel, bool allowSafetyCritical = false) const {
        // Check read-only access
        if (isReadOnly()) {
            return ValidationResult::READ_ONLY_VIOLATION;
        }

        // Check authorization level
        if (static_cast<uint8_t>(userLevel) < static_cast<uint8_t>(getRequiredAuthLevel())) {
            return ValidationResult::INSUFFICIENT_AUTHORIZATION;
        }

        // Check safety critical parameters
        if (isSafetyCritical() && !allowSafetyCritical) {
            return ValidationResult::SAFETY_CRITICAL_DENIED;
        }

        return ValidationResult::SUCCESS;
    }

    ValidationResult validateValue(float new_value) const {
        if (new_value < min_value || new_value > max_value) {
            return ValidationResult::OUT_OF_RANGE;
        }
        return ValidationResult::SUCCESS;
    }

    void updateCRC() {
        crc32 = 0;
        auto crc_result = g_flash_storage.calculateCRC32(reinterpret_cast<const uint8_t*>(this), sizeof(*this));
        crc32 = crc_result.isSuccess() ? crc_result.value : 0;
    }

    bool verifyCRC() const {
        uint32_t stored_crc = crc32;
        const_cast<StoredParameter*>(this)->crc32 = 0;
        auto crc_result = g_flash_storage.calculateCRC32(reinterpret_cast<const uint8_t*>(this), sizeof(*this));
        const_cast<StoredParameter*>(this)->crc32 = stored_crc;
        return crc_result.isSuccess() && crc_result.value == stored_crc;
    }
};

// Parameter block header (packed for flash storage)
struct __attribute__((packed)) ParameterBlockHeader {
    uint32_t magic;                        // Magic number for identification
    uint16_t version;                      // Storage format version
    uint16_t param_count;                  // Number of parameters in block
    uint32_t timestamp;                    // Last update timestamp
    uint32_t sequence_number;              // Incrementing sequence for wear leveling
    uint32_t header_crc;                   // Header integrity check
    uint32_t data_crc;                     // Data section CRC
    uint8_t reserved[16];                  // Reserved for future expansion

    ParameterBlockHeader() {
        std::memset(this, 0, sizeof(*this));
        magic = Config::MAGIC_NUMBER;
        version = Config::STORAGE_VERSION;
    }

    bool isValid() const {
        return magic == Config::MAGIC_NUMBER &&
               version == Config::STORAGE_VERSION &&
               param_count <= Config::MAX_PARAMETERS;
    }

    void updateHeaderCRC() {
        uint32_t temp_crc = header_crc;
        header_crc = 0;
        auto crc_result = g_flash_storage.calculateCRC32(reinterpret_cast<const uint8_t*>(this), sizeof(*this));
        header_crc = crc_result.isSuccess() ? crc_result.value : temp_crc;
    }

    bool verifyHeaderCRC() const {
        uint32_t stored_crc = header_crc;
        const_cast<ParameterBlockHeader*>(this)->header_crc = 0;
        auto crc_result = g_flash_storage.calculateCRC32(reinterpret_cast<const uint8_t*>(this), sizeof(*this));
        const_cast<ParameterBlockHeader*>(this)->header_crc = stored_crc;
        return crc_result.isSuccess() && crc_result.value == stored_crc;
    }
};

// Complete parameter block for storage
struct ParameterBlock {
    ParameterBlockHeader header;
    StoredParameter parameters[Config::MAX_PARAMETERS];

    ParameterBlock() = default;

    bool isValid() const {
        if (!header.isValid() || !header.verifyHeaderCRC()) {
            return false;
        }

        // Verify data CRC
        auto data_crc = g_flash_storage.calculateCRC32(
            reinterpret_cast<const uint8_t*>(parameters),
            header.param_count * sizeof(StoredParameter)
        );

        if (data_crc.isError() || data_crc.value != header.data_crc) {
            return false;
        }

        // Verify individual parameter CRCs
        for (uint16_t i = 0; i < header.param_count; i++) {
            if (!parameters[i].verifyCRC()) {
                return false;
            }
        }

        return true;
    }

    void updateCRCs() {
        // Update parameter CRCs
        for (uint16_t i = 0; i < header.param_count; i++) {
            parameters[i].updateCRC();
        }

        // Update data CRC
        auto data_crc = g_flash_storage.calculateCRC32(
            reinterpret_cast<const uint8_t*>(parameters),
            header.param_count * sizeof(StoredParameter)
        );
        header.data_crc = data_crc.isSuccess() ? data_crc.value : 0;

        // Update header CRC last
        header.updateHeaderCRC();
    }

    size_t getStorageSize() const {
        return sizeof(ParameterBlockHeader) + header.param_count * sizeof(StoredParameter);
    }
};

// Runtime parameter cache entry
struct ParameterCacheEntry {
    StoredParameter stored_param;
    bool dirty;                           // Needs to be written to flash
    bool loaded;                          // Has been loaded from flash
    uint32_t last_access_time;           // For LRU eviction
    std::function<void(float)> setter;   // Optional callback when value changes
    std::function<float()> getter;       // Optional callback to get current value

    ParameterCacheEntry() : dirty(false), loaded(false), last_access_time(0) {}
};

// Parameter change notification
struct ParameterChangeNotification {
    char name[Config::MAX_PARAM_NAME_LEN];
    float old_value;
    float new_value;
    bool requires_reboot;
    bool is_safety_critical;
    uint32_t timestamp;
};

// Atomic transaction context
struct AtomicTransaction {
    bool active;
    ParameterChangeNotification pending_changes[16];
    uint8_t change_count;
    uint32_t transaction_id;

    AtomicTransaction() : active(false), change_count(0), transaction_id(0) {}
};

// High-level parameter storage manager
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

    // Initialization and management
    StorageResult<bool> initialize();
    StorageResult<bool> loadDefaults();
    StorageResult<bool> factoryReset();

    // Parameter operations (enhanced with validation)
    StorageResult<ValidationResult> setParameter(const char* name, float value, bool force_save = false);
    StorageResult<ValidationResult> setParameterWithAuth(const char* name, float value,
                                                       AuthorizationLevel auth_level,
                                                       bool allow_safety_critical = false,
                                                       bool force_save = false);
    StorageResult<float> getParameter(const char* name);
    StorageResult<bool> hasParameter(const char* name);

    // Parameter registration
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

    // Persistence operations
    StorageResult<bool> saveParameters();
    StorageResult<bool> loadParameters();
    StorageResult<bool> saveSingleParameter(const char* name);

    // Status and diagnostics
    StorageResult<uint16_t> getParameterCount() const;
    StorageResult<bool> isDirty() const;
    StorageResult<float> getStorageHealth() const;
    StorageResult<uint32_t> getLastSaveTime() const;

    // Authorization management
    void setAuthorizationLevel(AuthorizationLevel level) { current_auth_level_ = level; }
    AuthorizationLevel getAuthorizationLevel() const { return current_auth_level_; }
    void setSafetyCriticalEnabled(bool enabled) { safety_critical_enabled_ = enabled; }
    bool isSafetyCriticalEnabled() const { return safety_critical_enabled_; }

    // Atomic transactions
    StorageResult<uint32_t> beginTransaction();
    StorageResult<bool> commitTransaction(uint32_t transaction_id);
    StorageResult<bool> rollbackTransaction(uint32_t transaction_id);
    bool isTransactionActive() const { return active_transaction_.active; }

    // Parameter impact analysis
    StorageResult<std::vector<ParameterChangeNotification>> analyzeParameterChange(const char* name, float new_value);

    // Configuration
    void setAutoSave(bool enabled) { auto_save_enabled_ = enabled; }
    bool isAutoSaveEnabled() const { return auto_save_enabled_; }

    // Maintenance
    StorageResult<bool> update();  // Call periodically for auto-save
    StorageResult<bool> compact(); // Defragment storage

private:
    // Cache management
    StorageResult<ParameterCacheEntry*> findInCache(const char* name);
    StorageResult<ParameterCacheEntry*> addToCache(const StoredParameter& param);
    StorageResult<bool> evictLRU();
    void updateAccessTime(ParameterCacheEntry* entry);

    // Storage operations
    StorageResult<bool> saveBlock(const ParameterBlock& block, uint32_t address);
    StorageResult<ParameterBlock> loadBlock(uint32_t address);
    StorageResult<uint32_t> findValidBlock();
    StorageResult<bool> switchBlocks();

    // Validation and recovery
    StorageResult<bool> validateParameter(const StoredParameter& param);
    StorageResult<ValidationResult> validateParameterChange(const char* name, float value,
                                                          AuthorizationLevel auth_level,
                                                          bool allow_safety_critical);
    StorageResult<bool> recoverFromCorruption();
    bool shouldAutoSave() const;

    // Transaction helpers
    StorageResult<bool> addToTransaction(const char* name, float old_value, float new_value, bool requires_reboot, bool is_safety_critical);
    StorageResult<bool> applyTransactionChanges();
    void clearTransaction();

    // Utility functions
    uint32_t getCurrentTime() const;
    StorageResult<bool> copyString(char* dest, const char* src, size_t max_len);
};

// Global instance
extern ParameterStorage g_parameter_storage;

} // namespace Storage