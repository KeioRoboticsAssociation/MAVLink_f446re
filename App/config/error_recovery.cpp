#include "error_recovery.hpp"
#include <cstdio>

extern "C" {
#include "main.h" // For HAL_GetTick()
}

namespace Config {

ErrorRecoveryManager::ErrorRecoveryManager() {
    // Initialize default strategies for all error codes
    for (size_t i = 0; i < strategies_.size(); ++i) {
        ErrorCode errorCode = static_cast<ErrorCode>(i);
        strategies_[i] = getDefaultStrategy(errorCode);
    }
}

void ErrorRecoveryManager::configureStrategy(ErrorCode errorCode, const RecoveryStrategy& strategy) {
    size_t index = static_cast<size_t>(errorCode);
    if (index < strategies_.size()) {
        strategies_[index] = strategy;
    }
}

Config::Result<void> ErrorRecoveryManager::handleError(ErrorCode errorCode, const char* context) {
    if (errorCode == ErrorCode::OK) {
        return Config::Result<void>();
    }

    // Check if we can attempt recovery
    if (!canAttemptRecovery(errorCode)) {
        logRecoveryAttempt(errorCode, RecoveryAction::NONE, context);
        return Config::Result<void>(ErrorCode::NOT_SUPPORTED);
    }

    size_t index = static_cast<size_t>(errorCode);
    const auto& strategy = strategies_[index];

    // Update recovery context
    updateContext(errorCode, strategy.primaryAction);

    // Check cooldown period
    uint32_t currentTime = HAL_GetTick();
    if (context_.inCooldown &&
        (currentTime - context_.lastRecoveryTime) < strategy.cooldownMs) {
        return Config::Result<void>(ErrorCode::TIMEOUT);
    }

    // Check retry limit
    if (context_.retryCount >= strategy.maxRetries) {
        // Try fallback action if available
        if (strategy.fallbackAction != RecoveryAction::NONE) {
            logRecoveryAttempt(errorCode, strategy.fallbackAction, context);
            return executeRecovery(errorCode, strategy.fallbackAction);
        } else {
            failedRecoveries_++;
            resetContext();
            return Config::Result<void>(ErrorCode::RESOURCE_EXHAUSTED);
        }
    }

    // Check if user confirmation is required
    if (strategy.requiresUserConfirmation && !context_.userConfirmationRequired) {
        context_.userConfirmationRequired = true;
        return Config::Result<void>(ErrorCode::INVALID_STATE);
    }

    // Execute primary recovery action
    logRecoveryAttempt(errorCode, strategy.primaryAction, context);
    return executeRecovery(errorCode, strategy.primaryAction);
}

Config::Result<void> ErrorRecoveryManager::executeRecovery(ErrorCode errorCode, RecoveryAction action) {
    context_.recoveryInProgress = true;
    context_.currentAction = action;
    totalRecoveries_++;

    Config::Result<void> result;

    switch (action) {
        case RecoveryAction::RETRY:
            result = performRetry(errorCode);
            break;
        case RecoveryAction::RESET:
            result = performReset(errorCode);
            break;
        case RecoveryAction::REINITIALIZE:
            result = performReinitialize(errorCode);
            break;
        case RecoveryAction::FALLBACK:
            result = performFallback(errorCode);
            break;
        case RecoveryAction::EMERGENCY_STOP:
            result = performEmergencyStop(errorCode);
            break;
        case RecoveryAction::SYSTEM_RESTART:
            result = performSystemRestart(errorCode);
            break;
        default:
            result = Config::Result<void>(ErrorCode::NOT_IMPLEMENTED);
            break;
    }

    // Update statistics
    if (result) {
        successfulRecoveries_++;
    } else {
        failedRecoveries_++;
    }

    // Update context
    context_.lastRecoveryTime = HAL_GetTick();
    context_.recoveryInProgress = false;

    return result;
}

void ErrorRecoveryManager::confirmUserAction() {
    context_.userConfirmationRequired = false;
}

void ErrorRecoveryManager::cancelRecovery() {
    resetContext();
}

void ErrorRecoveryManager::reportProgress(uint32_t currentStep, uint32_t totalSteps) {
    if (progressCallback_) {
        progressCallback_(context_.lastError, context_.currentAction, currentStep, totalSteps);
    }
}

RecoveryStrategy ErrorRecoveryManager::getDefaultStrategy(ErrorCode errorCode) {
    RecoveryStrategy strategy;
    strategy.errorCode = errorCode;
    strategy.maxRetries = 3;
    strategy.retryDelayMs = 100;
    strategy.cooldownMs = 1000;
    strategy.requiresUserConfirmation = false;

    switch (errorCode) {
        case ErrorCode::OK:
            strategy.primaryAction = RecoveryAction::NONE;
            strategy.fallbackAction = RecoveryAction::NONE;
            strategy.description = "No error";
            break;

        // System-level errors
        case ErrorCode::NOT_INITIALIZED:
            strategy.primaryAction = RecoveryAction::REINITIALIZE;
            strategy.fallbackAction = RecoveryAction::RESET;
            strategy.description = "Reinitialize component";
            break;

        case ErrorCode::INVALID_STATE:
            strategy.primaryAction = RecoveryAction::RESET;
            strategy.fallbackAction = RecoveryAction::REINITIALIZE;
            strategy.description = "Reset to valid state";
            break;

        // Hardware errors
        case ErrorCode::HARDWARE_ERROR:
            strategy.primaryAction = RecoveryAction::RETRY;
            strategy.fallbackAction = RecoveryAction::RESET;
            strategy.maxRetries = 5;
            strategy.description = "Retry hardware operation";
            break;

        case ErrorCode::HARDWARE_FAULT:
            strategy.primaryAction = RecoveryAction::FALLBACK;
            strategy.fallbackAction = RecoveryAction::EMERGENCY_STOP;
            strategy.requiresUserConfirmation = true;
            strategy.description = "Switch to fallback mode";
            break;

        // Communication errors
        case ErrorCode::TIMEOUT:
            strategy.primaryAction = RecoveryAction::RETRY;
            strategy.fallbackAction = RecoveryAction::RESET;
            strategy.maxRetries = 3;
            strategy.retryDelayMs = 500;
            strategy.description = "Retry with timeout";
            break;

        case ErrorCode::COMMUNICATION_ERROR:
            strategy.primaryAction = RecoveryAction::RETRY;
            strategy.fallbackAction = RecoveryAction::REINITIALIZE;
            strategy.description = "Retry communication";
            break;

        // Safety errors - immediate action required
        case ErrorCode::SAFETY_VIOLATION:
        case ErrorCode::EMERGENCY_STOP:
        case ErrorCode::OVER_TEMPERATURE:
        case ErrorCode::OVER_CURRENT:
            strategy.primaryAction = RecoveryAction::EMERGENCY_STOP;
            strategy.fallbackAction = RecoveryAction::SYSTEM_RESTART;
            strategy.maxRetries = 0; // No retries for safety issues
            strategy.requiresUserConfirmation = true;
            strategy.description = "Safety emergency stop";
            break;

        // Motor errors
        case ErrorCode::MOTOR_ERROR:
            strategy.primaryAction = RecoveryAction::RESET;
            strategy.fallbackAction = RecoveryAction::FALLBACK;
            strategy.description = "Reset motor controller";
            break;

        case ErrorCode::MOTOR_STALL:
            strategy.primaryAction = RecoveryAction::RETRY;
            strategy.fallbackAction = RecoveryAction::EMERGENCY_STOP;
            strategy.retryDelayMs = 1000;
            strategy.description = "Retry motor movement";
            break;

        // Configuration errors
        case ErrorCode::CONFIG_ERROR:
        case ErrorCode::OUT_OF_RANGE:
            strategy.primaryAction = RecoveryAction::FALLBACK;
            strategy.fallbackAction = RecoveryAction::REINITIALIZE;
            strategy.description = "Use fallback configuration";
            break;

        default:
            strategy.primaryAction = RecoveryAction::RETRY;
            strategy.fallbackAction = RecoveryAction::RESET;
            strategy.description = "Generic recovery";
            break;
    }

    return strategy;
}

const char* ErrorRecoveryManager::getActionName(RecoveryAction action) {
    switch (action) {
        case RecoveryAction::NONE: return "None";
        case RecoveryAction::RETRY: return "Retry";
        case RecoveryAction::RESET: return "Reset";
        case RecoveryAction::REINITIALIZE: return "Reinitialize";
        case RecoveryAction::FALLBACK: return "Fallback";
        case RecoveryAction::EMERGENCY_STOP: return "Emergency Stop";
        case RecoveryAction::SYSTEM_RESTART: return "System Restart";
        default: return "Unknown";
    }
}

ErrorRecoveryManager& ErrorRecoveryManager::getInstance() {
    static ErrorRecoveryManager instance;
    return instance;
}

bool ErrorRecoveryManager::canAttemptRecovery(ErrorCode errorCode) const {
    return ErrorUtils::isRecoverable(errorCode);
}

void ErrorRecoveryManager::updateContext(ErrorCode errorCode, RecoveryAction action) {
    if (context_.lastError != errorCode) {
        // New error, reset retry count
        context_.retryCount = 0;
        context_.lastError = errorCode;
    } else {
        // Same error, increment retry count
        context_.retryCount++;
    }

    context_.currentAction = action;
    context_.inCooldown = true;
}

void ErrorRecoveryManager::logRecoveryAttempt(ErrorCode errorCode, RecoveryAction action, const char* context) {
    if (logCallback_) {
        // Create log message
        char logMsg[256];
        snprintf(logMsg, sizeof(logMsg),
                "Recovery attempt: Error=%s, Action=%s, Context=%s, Retry=%lu",
                ErrorUtils::getDescription(errorCode),
                getActionName(action),
                context ? context : "None",
                static_cast<unsigned long>(context_.retryCount));
        logCallback_(logMsg);
    }
}

void ErrorRecoveryManager::resetContext() {
    context_ = RecoveryContext();
}

Config::Result<void> ErrorRecoveryManager::performRetry(ErrorCode errorCode) {
    if (recoveryCallback_) {
        return recoveryCallback_(errorCode, RecoveryAction::RETRY);
    }
    return Config::Result<void>();
}

Config::Result<void> ErrorRecoveryManager::performReset(ErrorCode errorCode) {
    if (recoveryCallback_) {
        return recoveryCallback_(errorCode, RecoveryAction::RESET);
    }
    return Config::Result<void>();
}

Config::Result<void> ErrorRecoveryManager::performReinitialize(ErrorCode errorCode) {
    if (recoveryCallback_) {
        return recoveryCallback_(errorCode, RecoveryAction::REINITIALIZE);
    }
    return Config::Result<void>();
}

Config::Result<void> ErrorRecoveryManager::performFallback(ErrorCode errorCode) {
    if (recoveryCallback_) {
        return recoveryCallback_(errorCode, RecoveryAction::FALLBACK);
    }
    return Config::Result<void>();
}

Config::Result<void> ErrorRecoveryManager::performEmergencyStop(ErrorCode errorCode) {
    if (recoveryCallback_) {
        return recoveryCallback_(errorCode, RecoveryAction::EMERGENCY_STOP);
    }
    return Config::Result<void>();
}

Config::Result<void> ErrorRecoveryManager::performSystemRestart(ErrorCode errorCode) {
    if (recoveryCallback_) {
        return recoveryCallback_(errorCode, RecoveryAction::SYSTEM_RESTART);
    }
    return Config::Result<void>();
}

// RecoveryCoordinator implementation
Config::Result<void> RecoveryCoordinator::registerComponent(IRecoverableComponent* component) {
    if (!component) {
        return Config::Result<void>(ErrorCode::INVALID_PARAMETER);
    }

    if (componentCount_ >= components_.size()) {
        return Config::Result<void>(ErrorCode::RESOURCE_EXHAUSTED);
    }

    // Check for duplicates
    for (size_t i = 0; i < componentCount_; ++i) {
        if (components_[i] == component) {
            return Config::Result<void>(ErrorCode::ALREADY_INITIALIZED);
        }
    }

    components_[componentCount_++] = component;
    return Config::Result<void>();
}

void RecoveryCoordinator::unregisterComponent(IRecoverableComponent* component) {
    for (size_t i = 0; i < componentCount_; ++i) {
        if (components_[i] == component) {
            // Shift remaining components
            for (size_t j = i; j < componentCount_ - 1; ++j) {
                components_[j] = components_[j + 1];
            }
            componentCount_--;
            break;
        }
    }
}

Config::Result<void> RecoveryCoordinator::performSystemRecovery(ErrorCode errorCode) {
    // Delegate to recovery manager
    return recoveryManager_.handleError(errorCode, "SystemRecovery");
}

Config::Result<void> RecoveryCoordinator::resetAllComponents() {
    Config::Result<void> overallResult;

    for (size_t i = 0; i < componentCount_; ++i) {
        auto result = components_[i]->reset();
        if (!result) {
            overallResult = result; // Store last error
        }
    }

    return overallResult;
}

Config::Result<void> RecoveryCoordinator::reinitializeAllComponents() {
    Config::Result<void> overallResult;

    for (size_t i = 0; i < componentCount_; ++i) {
        auto result = components_[i]->reinitialize();
        if (!result) {
            overallResult = result; // Store last error
        }
    }

    return overallResult;
}

bool RecoveryCoordinator::areAllComponentsHealthy() const {
    for (size_t i = 0; i < componentCount_; ++i) {
        if (components_[i]->isInFallbackMode()) {
            return false;
        }
    }
    return true;
}

// Global instances
ErrorRecoveryManager& recoveryManager = ErrorRecoveryManager::getInstance();
RecoveryCoordinator& recoveryCoordinator = *new RecoveryCoordinator();

} // namespace Config