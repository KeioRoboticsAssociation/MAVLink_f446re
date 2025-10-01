/**
 * @file error_recovery.hpp
 * @brief Defines the error recovery mechanisms for the system.
 */

#pragma once

#include "system_config.hpp"
#include "timeout_manager.hpp"
#include <cstdint>
#include <functional>
#include <array>

namespace Config {

/**
 * @brief Defines the possible recovery actions for an error.
 */
enum class RecoveryAction : uint8_t {
    NONE = 0,
    RETRY = 1,
    RESET = 2,
    REINITIALIZE = 3,
    FALLBACK = 4,
    EMERGENCY_STOP = 5,
    SYSTEM_RESTART = 6
};

/**
 * @brief Defines a recovery strategy for a specific error code.
 */
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

/**
 * @brief Tracks the context of an ongoing recovery process.
 */
struct RecoveryContext {
    ErrorCode lastError = ErrorCode::OK;
    RecoveryAction currentAction = RecoveryAction::NONE;
    uint32_t retryCount = 0;
    uint32_t lastRecoveryTime = 0;
    bool inCooldown = false;
    bool userConfirmationRequired = false;
    bool recoveryInProgress = false;
};

using RecoveryCallback = std::function<Config::Result<void>(ErrorCode, RecoveryAction)>;
using ProgressCallback = std::function<void(ErrorCode, RecoveryAction, uint32_t currentStep, uint32_t totalSteps)>;

/**
 * @brief Manages error recovery strategies and execution.
 */
class ErrorRecoveryManager {
private:
    std::array<RecoveryStrategy, 256> strategies_;
    RecoveryContext context_;
    RecoveryCallback recoveryCallback_;
    ProgressCallback progressCallback_;
    std::function<void(const char*)> logCallback_;
    uint32_t totalRecoveries_ = 0;
    uint32_t successfulRecoveries_ = 0;
    uint32_t failedRecoveries_ = 0;

public:
    ErrorRecoveryManager();

    /**
     * @brief Configures a recovery strategy for a given error code.
     * @param errorCode The error code to configure.
     * @param strategy The recovery strategy.
     */
    void configureStrategy(ErrorCode errorCode, const RecoveryStrategy& strategy);

    /**
     * @brief Sets the callback function for executing recovery actions.
     * @param callback The recovery callback.
     */
    void setRecoveryCallback(RecoveryCallback callback) { recoveryCallback_ = callback; }

    /**
     * @brief Sets the callback function for reporting recovery progress.
     * @param callback The progress callback.
     */
    void setProgressCallback(ProgressCallback callback) { progressCallback_ = callback; }

    /**
     * @brief Sets the callback function for logging.
     * @param callback The log callback.
     */
    void setLogCallback(std::function<void(const char*)> callback) { logCallback_ = callback; }

    /**
     * @brief Handles an error by triggering the appropriate recovery strategy.
     * @param errorCode The error code that occurred.
     * @param context Optional context information.
     * @return Result of the operation.
     */
    Config::Result<void> handleError(ErrorCode errorCode, const char* context = nullptr);

    /**
     * @brief Executes a specific recovery action.
     * @param errorCode The error code.
     * @param action The recovery action to execute.
     * @return Result of the operation.
     */
    Config::Result<void> executeRecovery(ErrorCode errorCode, RecoveryAction action);

    bool isRecoveryInProgress() const { return context_.recoveryInProgress; }
    bool requiresUserConfirmation() const { return context_.userConfirmationRequired; }
    void confirmUserAction();
    void cancelRecovery();
    void reportProgress(uint32_t currentStep, uint32_t totalSteps);
    uint32_t getTotalRecoveries() const { return totalRecoveries_; }
    uint32_t getSuccessfulRecoveries() const { return successfulRecoveries_; }
    uint32_t getFailedRecoveries() const { return failedRecoveries_; }
    float getSuccessRate() const;

    static RecoveryStrategy getDefaultStrategy(ErrorCode errorCode);
    static const char* getActionName(RecoveryAction action);
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

/**
 * @brief An RAII guard for automatic error handling.
 */
class RecoveryGuard {
private:
    ErrorRecoveryManager& manager_;
    ErrorCode monitoredError_;
    bool autoRecover_;

public:
    RecoveryGuard(ErrorRecoveryManager& manager, ErrorCode errorToMonitor, bool autoRecover = true)
        : manager_(manager), monitoredError_(errorToMonitor), autoRecover_(autoRecover) {}

    ~RecoveryGuard();

    void setError(ErrorCode error) { monitoredError_ = error; }
    void clearError() { monitoredError_ = ErrorCode::OK; }

    RecoveryGuard(const RecoveryGuard&) = delete;
    RecoveryGuard& operator=(const RecoveryGuard&) = delete;
};

/**
 * @brief Interface for components that can be recovered.
 */
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

/**
 * @brief Coordinates recovery actions across multiple components.
 */
class RecoveryCoordinator {
private:
    std::array<IRecoverableComponent*, 16> components_;
    size_t componentCount_ = 0;
    ErrorRecoveryManager& recoveryManager_;

public:
    explicit RecoveryCoordinator(ErrorRecoveryManager& manager = ErrorRecoveryManager::getInstance())
        : recoveryManager_(manager) {}

    Config::Result<void> registerComponent(IRecoverableComponent* component);
    void unregisterComponent(IRecoverableComponent* component);
    Config::Result<void> performSystemRecovery(ErrorCode errorCode);
    Config::Result<void> resetAllComponents();
    Config::Result<void> reinitializeAllComponents();
    size_t getComponentCount() const { return componentCount_; }
    bool areAllComponentsHealthy() const;
};

extern ErrorRecoveryManager& recoveryManager;
extern RecoveryCoordinator& recoveryCoordinator;

} // namespace Config