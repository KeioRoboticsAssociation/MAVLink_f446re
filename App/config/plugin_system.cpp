/**
 * @file plugin_system.cpp
 * @brief Implements the plugin system for extending functionality.
 */

#include "plugin_system.hpp"
#include "../comm/message_dispatcher.hpp"
#include <cstring>

extern "C" {
#include "main.h" // For HAL_GetTick()
}

namespace Config {

/**
 * @brief Construct a new PluginRegistry object.
 */
PluginRegistry::PluginRegistry() {
    addSearchPath("/plugins");
    addSearchPath("/usr/local/lib/robot_plugins");
}

/**
 * @brief Loads a plugin from a given path.
 * @param pluginPath The path to the plugin to load.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::loadPlugin(const char* pluginPath) {
    if (!pluginPath) {
        return Config::Result<void>(ErrorCode::INVALID_PARAMETER);
    }
    if (isPluginLoaded(pluginPath)) {
        return Config::Result<void>(ErrorCode::ALREADY_INITIALIZED);
    }
    totalPluginsLoaded_++;
    return Config::Result<void>();
}

/**
 * @brief Unloads a plugin.
 * @param pluginName The name of the plugin to unload.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::unloadPlugin(const char* pluginName) {
    auto* entry = findPluginEntry(pluginName);
    if (!entry) {
        return Config::Result<void>(ErrorCode::NOT_INITIALIZED);
    }
    if (entry->active && entry->plugin) {
        auto result = entry->plugin->stop();
        if (!result) {
            return result;
        }
    }
    if (entry->plugin) {
        entry->plugin->shutdown();
        entry->plugin.reset();
    }
    entry->active = false;
    activePlugins_--;
    return Config::Result<void>();
}

/**
 * @brief Enables a loaded plugin.
 * @param pluginName The name of the plugin to enable.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::enablePlugin(const char* pluginName) {
    auto* entry = findPluginEntry(pluginName);
    if (!entry || !entry->plugin) {
        return Config::Result<void>(ErrorCode::NOT_INITIALIZED);
    }
    if (entry->active) {
        return Config::Result<void>();
    }
    auto result = entry->plugin->start();
    if (result) {
        entry->active = true;
        activePlugins_++;
    }
    return result;
}

/**
 * @brief Disables an active plugin.
 * @param pluginName The name of the plugin to disable.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::disablePlugin(const char* pluginName) {
    auto* entry = findPluginEntry(pluginName);
    if (!entry || !entry->plugin) {
        return Config::Result<void>(ErrorCode::NOT_INITIALIZED);
    }
    if (!entry->active) {
        return Config::Result<void>();
    }
    auto result = entry->plugin->stop();
    if (result) {
        entry->active = false;
        activePlugins_--;
    }
    return result;
}

/**
 * @brief Adds a search path for plugin discovery.
 * @param path The path to add.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::addSearchPath(const char* path) {
    if (!path || searchPathCount_ >= searchPaths_.size()) {
        return Config::Result<void>(ErrorCode::INVALID_PARAMETER);
    }
    searchPaths_[searchPathCount_++] = path;
    return Config::Result<void>();
}

/**
 * @brief Scans for available plugins.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::scanForPlugins() {
    return Config::Result<void>();
}

/**
 * @brief Automatically loads plugins based on configuration.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::autoLoadPlugins() {
    return Config::Result<void>();
}

/**
 * @brief Finds a loaded plugin by name.
 * @param name The name of the plugin to find.
 * @return A pointer to the plugin, or nullptr if not found.
 */
IPlugin* PluginRegistry::findPlugin(const char* name) const {
    auto* entry = const_cast<PluginRegistry*>(this)->findPluginEntry(name);
    return entry ? entry->plugin.get() : nullptr;
}

/**
 * @brief Enumerates all loaded plugins.
 * @param callback The function to call for each plugin.
 */
void PluginRegistry::enumeratePlugins(std::function<void(const PluginInfo&, bool active)> callback) const {
    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin) {
            callback(plugins_[i].info, plugins_[i].active);
        }
    }
}

/**
 * @brief Enumerates plugins of a specific type.
 * @param type The type of plugin to enumerate.
 * @param callback The function to call for each plugin of the specified type.
 */
void PluginRegistry::enumeratePluginsByType(PluginType type,
                                           std::function<void(const PluginInfo&, IPlugin*)> callback) const {
    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin && plugins_[i].info.type == type) {
            callback(plugins_[i].info, plugins_[i].plugin.get());
        }
    }
}

/**
 * @brief Checks if a plugin is loaded.
 * @param name The name of the plugin.
 * @return true if the plugin is loaded, false otherwise.
 */
bool PluginRegistry::isPluginLoaded(const char* name) const {
    return findPlugin(name) != nullptr;
}

/**
 * @brief Checks if a plugin is active.
 * @param name The name of the plugin.
 * @return true if the plugin is active, false otherwise.
 */
bool PluginRegistry::isPluginActive(const char* name) const {
    auto* entry = const_cast<PluginRegistry*>(this)->findPluginEntry(name);
    return entry && entry->active;
}

/**
 * @brief Validates a plugin.
 * @param plugin The plugin to validate.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::validatePlugin(IPlugin* plugin) const {
    if (!plugin) {
        return Config::Result<void>(ErrorCode::INVALID_PARAMETER);
    }
    const auto& info = plugin->getInfo();
    return validatePluginInfo(info);
}

/**
 * @brief Checks the dependencies of a plugin.
 * @param info The plugin info.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::checkDependencies(const PluginInfo& info) const {
    for (size_t i = 0; i < info.dependencyCount; ++i) {
        const auto& dep = info.dependencies[i];
        if (dep.required && !isPluginLoaded(dep.name)) {
            return Config::Result<void>(ErrorCode::MISSING_CONFIG);
        }
    }
    return Config::Result<void>();
}

/**
 * @brief Initializes all loaded plugins.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::initializeAllPlugins() {
    Config::Result<void> result;
    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin) {
            auto initResult = plugins_[i].plugin->initialize();
            if (!initResult) {
                result = initResult;
                plugins_[i].errorCount++;
            }
        }
    }
    return result;
}

/**
 * @brief Starts all initialized plugins.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::startAllPlugins() {
    Config::Result<void> result;
    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin && !plugins_[i].active) {
            auto startResult = plugins_[i].plugin->start();
            if (startResult) {
                plugins_[i].active = true;
                activePlugins_++;
            } else {
                result = startResult;
                plugins_[i].errorCount++;
            }
        }
    }
    return result;
}

/**
 * @brief Stops all active plugins.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::stopAllPlugins() {
    Config::Result<void> result;
    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin && plugins_[i].active) {
            auto stopResult = plugins_[i].plugin->stop();
            if (stopResult) {
                plugins_[i].active = false;
            } else {
                result = stopResult;
                plugins_[i].errorCount++;
            }
        }
    }
    activePlugins_ = 0;
    return result;
}

/**
 * @brief Shuts down all loaded plugins.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::shutdownAllPlugins() {
    Config::Result<void> result;
    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin) {
            auto shutdownResult = plugins_[i].plugin->shutdown();
            if (!shutdownResult) {
                result = shutdownResult;
            }
            plugins_[i].plugin.reset();
            plugins_[i].active = false;
        }
    }
    pluginCount_ = 0;
    activePlugins_ = 0;
    return result;
}

/**
 * @brief Updates all active plugins.
 * @param deltaTime The time since the last update.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::updateAllPlugins(float deltaTime) {
    Config::Result<void> result;
    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin && plugins_[i].active) {
            auto updateResult = plugins_[i].plugin->update(deltaTime);
            if (!updateResult) {
                result = updateResult;
                plugins_[i].errorCount++;
            }
        }
    }
    return result;
}

/**
 * @brief Handles an error that occurred in a plugin.
 * @param pluginName The name of the plugin that failed.
 * @param error The error code.
 * @return Result of the operation.
 */
Config::Result<void> PluginRegistry::handlePluginError(const char* pluginName, ErrorCode error) {
    auto* entry = findPluginEntry(pluginName);
    if (!entry) {
        return Config::Result<void>(ErrorCode::NOT_INITIALIZED);
    }
    entry->errorCount++;
    if (entry->plugin) {
        return entry->plugin->reset();
    }
    return Config::Result<void>(error);
}

/**
 * @brief Gets the singleton instance of the PluginRegistry.
 * @return The singleton instance.
 */
PluginRegistry& PluginRegistry::getInstance() {
    static PluginRegistry instance;
    return instance;
}

PluginRegistry::PluginEntry* PluginRegistry::findPluginEntry(const char* name) {
    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin && strcmp(plugins_[i].info.name, name) == 0) {
            return &plugins_[i];
        }
    }
    return nullptr;
}

Config::Result<void> PluginRegistry::registerPlugin(std::unique_ptr<IPlugin> plugin) {
    if (!plugin) {
        return Config::Result<void>(ErrorCode::INVALID_PARAMETER);
    }
    if (pluginCount_ >= plugins_.size()) {
        return Config::Result<void>(ErrorCode::RESOURCE_EXHAUSTED);
    }
    auto validationResult = validatePlugin(plugin.get());
    if (!validationResult) {
        return validationResult;
    }
    const auto& info = plugin->getInfo();
    auto depResult = checkDependencies(info);
    if (!depResult) {
        return depResult;
    }
    PluginEntry entry;
    entry.plugin = std::move(plugin);
    entry.info = info;
    entry.active = false;
    entry.loadTime = HAL_GetTick();
    entry.errorCount = 0;
    plugins_[pluginCount_++] = std::move(entry);
    return Config::Result<void>();
}

Config::Result<void> PluginRegistry::validatePluginInfo(const PluginInfo& info) const {
    if (!info.name || strlen(info.name) == 0) {
        return Config::Result<void>(ErrorCode::INVALID_PARAMETER);
    }
    if (!info.version || strlen(info.version) == 0) {
        return Config::Result<void>(ErrorCode::INVALID_PARAMETER);
    }
    if (info.type >= PluginType::MAX_PLUGIN_TYPES) {
        return Config::Result<void>(ErrorCode::OUT_OF_RANGE);
    }
    return Config::Result<void>();
}

void PluginRegistry::updateStatistics() {
    activePlugins_ = 0;
    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].active) {
            activePlugins_++;
        }
    }
}

/**
 * @brief Creates a new motor controller plugin.
 * @param motorType The type of motor the plugin supports.
 * @return A unique pointer to the created plugin.
 */
std::unique_ptr<IMotorControllerPlugin> MotorControllerPluginFactory::createPlugin(const char* motorType) const {
    for (size_t i = 0; i < creatorCount_; ++i) {
        if (strcmp(creators_[i].motorType, motorType) == 0) {
            return creators_[i].creator();
        }
    }
    return nullptr;
}

/**
 * @brief Checks if a motor type is supported by any plugin.
 * @param motorType The motor type to check.
 * @return true if the motor type is supported, false otherwise.
 */
bool MotorControllerPluginFactory::supportsMotorType(const char* motorType) const {
    for (size_t i = 0; i < creatorCount_; ++i) {
        if (strcmp(creators_[i].motorType, motorType) == 0) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Enumerates all supported motor types.
 * @param callback The function to call for each supported motor type.
 */
void MotorControllerPluginFactory::enumerateSupportedTypes(std::function<void(const char*)> callback) const {
    for (size_t i = 0; i < creatorCount_; ++i) {
        callback(creators_[i].motorType);
    }
}

/**
 * @brief Gets the singleton instance of the MotorControllerPluginFactory.
 * @return The singleton instance.
 */
MotorControllerPluginFactory& MotorControllerPluginFactory::getInstance() {
    static MotorControllerPluginFactory instance;
    return instance;
}

/**
 * @brief Construct a new PluginSystem object.
 */
PluginSystem::PluginSystem()
    : registry_(PluginRegistry::getInstance()),
      motorFactory_(MotorControllerPluginFactory::getInstance()) {}

/**
 * @brief Initializes the plugin system.
 * @param config The configuration for the plugin system.
 * @return Result of the operation.
 */
Config::Result<void> PluginSystem::initialize(const PluginSystemConfig& config) {
    if (initialized_) {
        return Config::Result<void>(ErrorCode::ALREADY_INITIALIZED);
    }
    config_ = config;
    auto coreResult = loadCorePlugins();
    if (!coreResult) {
        return coreResult;
    }
    if (config_.autoLoadEnabled) {
        auto autoLoadResult = registry_.autoLoadPlugins();
        if (!autoLoadResult) {
            return autoLoadResult;
        }
    }
    auto initResult = registry_.initializeAllPlugins();
    if (!initResult) {
        return initResult;
    }
    auto startResult = registry_.startAllPlugins();
    if (!startResult) {
        return startResult;
    }
    initialized_ = true;
    return Config::Result<void>();
}

/**
 * @brief Shuts down the plugin system.
 * @return Result of the operation.
 */
Config::Result<void> PluginSystem::shutdown() {
    if (!initialized_) {
        return Config::Result<void>();
    }
    auto result = registry_.shutdownAllPlugins();
    initialized_ = false;
    fallbackMode_ = false;
    return result;
}

/**
 * @brief Updates the plugin system.
 * @param deltaTime The time since the last update.
 * @return Result of the operation.
 */
Config::Result<void> PluginSystem::update(float deltaTime) {
    if (!initialized_) {
        return Config::Result<void>(ErrorCode::NOT_INITIALIZED);
    }
    return registry_.updateAllPlugins(deltaTime);
}

/**
 * @brief Retries a failed operation.
 * @return Result of the operation.
 */
Config::Result<void> PluginSystem::retry() {
    if (fallbackMode_) {
        fallbackMode_ = false;
        return registry_.startAllPlugins();
    }
    return Config::Result<void>();
}

/**
 * @brief Resets the plugin system.
 * @return Result of the operation.
 */
Config::Result<void> PluginSystem::reset() {
    auto stopResult = registry_.stopAllPlugins();
    if (!stopResult) {
        return stopResult;
    }
    return registry_.startAllPlugins();
}

/**
 * @brief Reinitializes the plugin system.
 * @return Result of the operation.
 */
Config::Result<void> PluginSystem::reinitialize() {
    auto shutdownResult = shutdown();
    if (!shutdownResult) {
        return shutdownResult;
    }
    return initialize(config_);
}

/**
 * @brief Enters a fallback mode with only essential plugins running.
 * @return Result of the operation.
 */
Config::Result<void> PluginSystem::enterFallbackMode() {
    registry_.stopAllPlugins();
    fallbackMode_ = true;
    return loadCorePlugins();
}

/**
 * @brief Checks if the plugin system is healthy.
 * @return true if the system is healthy, false otherwise.
 */
bool PluginSystem::isHealthy() const {
    if (!initialized_ || fallbackMode_) {
        return false;
    }
    bool healthy = true;
    registry_.enumeratePlugins([&healthy](const PluginInfo& info, bool active) {
        if (active) {
            // Health check logic would be here
        }
    });
    return healthy;
}

/**
 * @brief Gets the singleton instance of the PluginSystem.
 * @return The singleton instance.
 */
PluginSystem& PluginSystem::getInstance() {
    static PluginSystem instance;
    return instance;
}

Config::Result<void> PluginSystem::loadCorePlugins() {
    return Config::Result<void>();
}

Config::Result<void> PluginSystem::validateSystemHealth() {
    return Config::Result<void>();
}

PluginSystem& pluginSystem = PluginSystem::getInstance();

} // namespace Config