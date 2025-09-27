#pragma once

#include "system_config.hpp"
#include "timeout_manager.hpp"
#include <cstdint>
#include <functional>
#include <array>

namespace Config {

// Recovery action types
enum class RecoveryAction : uint8_t {
    NONE = 0,           // No action needed
    RETRY = 1,          // Retry the operation
    RESET = 2,          // Reset the component
    REINITIALIZE = 3,   // Reinitialize the component
    FALLBACK = 4,       // Switch to fallback mode
    EMERGENCY_STOP = 5, // Trigger emergency stop
    SYSTEM_RESTART = 6  // Restart entire system
};

// Recovery strategy configuration
struct RecoveryStrategy {
    ErrorCode errorCode;
    RecoveryAction primaryAction;
    RecoveryAction fallbackAction;
    uint32_t maxRetries;
    uint32_t retryDelayMs;
    uint32_t cooldownMs;
    bool requiresUserConfirmation;
    const char* description;
};

// Recovery context for tracking ongoing recovery
struct RecoveryContext {
    ErrorCode lastError = ErrorCode::OK;
    RecoveryAction currentAction = RecoveryAction::NONE;
    uint32_t retryCount = 0;
    uint32_t lastRecoveryTime = 0;
    bool inCooldown = false;
    bool userConfirmationRequired = false;
    bool recoveryInProgress = false;
};

// Recovery callback types
using RecoveryCallback = std::function<Config::Result<void>(ErrorCode, RecoveryAction)>;
using ProgressCallback = std::function<void(ErrorCode, RecoveryAction, uint32_t currentStep, uint32_t totalSteps)>;

// Error recovery manager
class ErrorRecoveryManager {
private:
    std::array<RecoveryStrategy, 256> strategies_;
    RecoveryContext context_;

    // Callbacks
    RecoveryCallback recoveryCallback_;
    ProgressCallback progressCallback_;
    std::function<void(const char*)> logCallback_;

    // Statistics
    uint32_t totalRecoveries_ = 0;
    uint32_t successfulRecoveries_ = 0;
    uint32_t failedRecoveries_ = 0;

public:
    ErrorRecoveryManager();

    // Configuration
    void configureStrategy(ErrorCode errorCode, const RecoveryStrategy& strategy);
    void setRecoveryCallback(RecoveryCallback callback) { recoveryCallback_ = callback; }
    void setProgressCallback(ProgressCallback callback) { progressCallback_ = callback; }
    void setLogCallback(std::function<void(const char*)> callback) { logCallback_ = callback; }

    // Recovery execution
    Config::Result<void> handleError(ErrorCode errorCode, const char* context = nullptr);
    Config::Result<void> executeRecovery(ErrorCode errorCode, RecoveryAction action);

    // Status and control
    bool isRecoveryInProgress() const { return context_.recoveryInProgress; }
    bool requiresUserConfirmation() const { return context_.userConfirmationRequired; }
    void confirmUserAction();
    void cancelRecovery();

    // Progress tracking
    void reportProgress(uint32_t currentStep, uint32_t totalSteps);

    // Statistics
    uint32_t getTotalRecoveries() const { return totalRecoveries_; }
    uint32_t getSuccessfulRecoveries() const { return successfulRecoveries_; }
    uint32_t getFailedRecoveries() const { return failedRecoveries_; }
    float getSuccessRate() const {
        return totalRecoveries_ > 0 ?
            static_cast<float>(successfulRecoveries_) / totalRecoveries_ : 0.0f;
    }

    // Utility functions
    static RecoveryStrategy getDefaultStrategy(ErrorCode errorCode);
    static const char* getActionName(RecoveryAction action);

    // Singleton access
    static ErrorRecoveryManager& getInstance();

private:
    bool canAttemptRecovery(ErrorCode errorCode) const;
    void updateContext(ErrorCode errorCode, RecoveryAction action);
    void logRecoveryAttempt(ErrorCode errorCode, RecoveryAction action, const char* context);
    void resetContext();
    Config::Result<void> performRetry(ErrorCode errorCode);
    Config::Result<void> performReset(ErrorCode errorCode);
    Config::Result<void> performReinitialize(ErrorCode errorCode);
    Config::Result<void> performFallback(ErrorCode errorCode);
    Config::Result<void> performEmergencyStop(ErrorCode errorCode);
    Config::Result<void> performSystemRestart(ErrorCode errorCode);
};

// RAII Recovery guard for automatic error handling
class RecoveryGuard {
private:
    ErrorRecoveryManager& manager_;
    ErrorCode monitoredError_;
    bool autoRecover_;

public:
    RecoveryGuard(ErrorRecoveryManager& manager, ErrorCode errorToMonitor, bool autoRecover = true)
        : manager_(manager), monitoredError_(errorToMonitor), autoRecover_(autoRecover) {}

    ~RecoveryGuard() {
        if (autoRecover_ && monitoredError_ != ErrorCode::OK) {
            manager_.handleError(monitoredError_, "RecoveryGuard");
        }
    }

    void setError(ErrorCode error) { monitoredError_ = error; }
    void clearError() { monitoredError_ = ErrorCode::OK; }

    // Non-copyable
    RecoveryGuard(const RecoveryGuard&) = delete;
    RecoveryGuard& operator=(const RecoveryGuard&) = delete;
};

// Component-specific recovery interfaces
class IRecoverableComponent {
public:
    virtual ~IRecoverableComponent() = default;
    virtual Config::Result<void> retry() = 0;
    virtual Config::Result<void> reset() = 0;
    virtual Config::Result<void> reinitialize() = 0;
    virtual Config::Result<void> enterFallbackMode() = 0;
    virtual bool isInFallbackMode() const = 0;
    virtual const char* getComponentName() const = 0;
};

// Recovery coordinator for system-wide recovery
class RecoveryCoordinator {
private:
    std::array<IRecoverableComponent*, 16> components_;
    size_t componentCount_ = 0;
    ErrorRecoveryManager& recoveryManager_;

public:
    explicit RecoveryCoordinator(ErrorRecoveryManager& manager = ErrorRecoveryManager::getInstance())
        : recoveryManager_(manager) {}

    // Component registration
    Config::Result<void> registerComponent(IRecoverableComponent* component);
    void unregisterComponent(IRecoverableComponent* component);

    // System-wide recovery
    Config::Result<void> performSystemRecovery(ErrorCode errorCode);
    Config::Result<void> resetAllComponents();
    Config::Result<void> reinitializeAllComponents();

    // Status
    size_t getComponentCount() const { return componentCount_; }
    bool areAllComponentsHealthy() const;
};

// Global recovery manager instance
extern ErrorRecoveryManager& recoveryManager;
extern RecoveryCoordinator& recoveryCoordinator;

} // namespace Config