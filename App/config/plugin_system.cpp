#include "plugin_system.hpp"
#include <cstring>

extern "C" {
#include "main.h" // For HAL_GetTick()
}

namespace Config {

// PluginRegistry implementation
PluginRegistry::PluginRegistry() {
    // Initialize default search paths
    addSearchPath("/plugins");
    addSearchPath("/usr/local/lib/robot_plugins");
}

Config::Result<void> PluginRegistry::loadPlugin(const char* pluginPath) {
    if (!pluginPath) {
        return Config::Result<void>(ErrorCode::INVALID_PARAMETER);
    }

    // For embedded systems, we simulate dynamic loading
    // In a real implementation, this would load a shared library

    // Check if plugin is already loaded
    if (isPluginLoaded(pluginPath)) {
        return Config::Result<void>(ErrorCode::ALREADY_INITIALIZED);
    }

    // For now, return success - actual plugin loading would be implemented
    // based on the specific embedded platform capabilities
    totalPluginsLoaded_++;

    return Config::Result<void>();
}

Config::Result<void> PluginRegistry::unloadPlugin(const char* pluginName) {
    auto* entry = findPluginEntry(pluginName);
    if (!entry) {
        return Config::Result<void>(ErrorCode::NOT_INITIALIZED);
    }

    // Stop plugin if running
    if (entry->active && entry->plugin) {
        auto result = entry->plugin->stop();
        if (!result) {
            return result;
        }
    }

    // Shutdown plugin
    if (entry->plugin) {
        entry->plugin->shutdown();
        entry->plugin.reset();
    }

    // Remove from registry
    entry->active = false;
    activePlugins_--;

    return Config::Result<void>();
}

Config::Result<void> PluginRegistry::enablePlugin(const char* pluginName) {
    auto* entry = findPluginEntry(pluginName);
    if (!entry || !entry->plugin) {
        return Config::Result<void>(ErrorCode::NOT_INITIALIZED);
    }

    if (entry->active) {
        return Config::Result<void>(); // Already active
    }

    // Start the plugin
    auto result = entry->plugin->start();
    if (result) {
        entry->active = true;
        activePlugins_++;
    }

    return result;
}

Config::Result<void> PluginRegistry::disablePlugin(const char* pluginName) {
    auto* entry = findPluginEntry(pluginName);
    if (!entry || !entry->plugin) {
        return Config::Result<void>(ErrorCode::NOT_INITIALIZED);
    }

    if (!entry->active) {
        return Config::Result<void>(); // Already inactive
    }

    // Stop the plugin
    auto result = entry->plugin->stop();
    if (result) {
        entry->active = false;
        activePlugins_--;
    }

    return result;
}

Config::Result<void> PluginRegistry::addSearchPath(const char* path) {
    if (!path || searchPathCount_ >= searchPaths_.size()) {
        return Config::Result<void>(ErrorCode::INVALID_PARAMETER);
    }

    searchPaths_[searchPathCount_++] = path;
    return Config::Result<void>();
}

Config::Result<void> PluginRegistry::scanForPlugins() {
    // In embedded systems, plugin scanning is typically done at compile time
    // or through a manifest file. For now, we'll simulate success.
    return Config::Result<void>();
}

Config::Result<void> PluginRegistry::autoLoadPlugins() {
    // Auto-load core plugins
    // This would typically load plugins based on configuration
    return Config::Result<void>();
}

IPlugin* PluginRegistry::findPlugin(const char* name) const {
    auto* entry = const_cast<PluginRegistry*>(this)->findPluginEntry(name);
    return entry ? entry->plugin.get() : nullptr;
}

void PluginRegistry::enumeratePlugins(std::function<void(const PluginInfo&, bool active)> callback) const {
    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin) {
            callback(plugins_[i].info, plugins_[i].active);
        }
    }
}

void PluginRegistry::enumeratePluginsByType(PluginType type,
                                           std::function<void(const PluginInfo&, IPlugin*)> callback) const {
    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin && plugins_[i].info.type == type) {
            callback(plugins_[i].info, plugins_[i].plugin.get());
        }
    }
}

bool PluginRegistry::isPluginLoaded(const char* name) const {
    return findPlugin(name) != nullptr;
}

bool PluginRegistry::isPluginActive(const char* name) const {
    auto* entry = const_cast<PluginRegistry*>(this)->findPluginEntry(name);
    return entry && entry->active;
}

Config::Result<void> PluginRegistry::validatePlugin(IPlugin* plugin) const {
    if (!plugin) {
        return Config::Result<void>(ErrorCode::INVALID_PARAMETER);
    }

    const auto& info = plugin->getInfo();
    return validatePluginInfo(info);
}

Config::Result<void> PluginRegistry::checkDependencies(const PluginInfo& info) const {
    for (size_t i = 0; i < info.dependencyCount; ++i) {
        const auto& dep = info.dependencies[i];

        if (dep.required && !isPluginLoaded(dep.name)) {
            return Config::Result<void>(ErrorCode::MISSING_CONFIG);
        }
    }

    return Config::Result<void>();
}

Config::Result<void> PluginRegistry::initializeAllPlugins() {
    Config::Result<void> result;

    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin) {
            auto initResult = plugins_[i].plugin->initialize();
            if (!initResult) {
                result = initResult; // Store last error
                plugins_[i].errorCount++;
            }
        }
    }

    return result;
}

Config::Result<void> PluginRegistry::startAllPlugins() {
    Config::Result<void> result;

    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin && !plugins_[i].active) {
            auto startResult = plugins_[i].plugin->start();
            if (startResult) {
                plugins_[i].active = true;
                activePlugins_++;
            } else {
                result = startResult; // Store last error
                plugins_[i].errorCount++;
            }
        }
    }

    return result;
}

Config::Result<void> PluginRegistry::stopAllPlugins() {
    Config::Result<void> result;

    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin && plugins_[i].active) {
            auto stopResult = plugins_[i].plugin->stop();
            if (stopResult) {
                plugins_[i].active = false;
            } else {
                result = stopResult; // Store last error
                plugins_[i].errorCount++;
            }
        }
    }

    activePlugins_ = 0;
    return result;
}

Config::Result<void> PluginRegistry::shutdownAllPlugins() {
    Config::Result<void> result;

    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin) {
            auto shutdownResult = plugins_[i].plugin->shutdown();
            if (!shutdownResult) {
                result = shutdownResult; // Store last error
            }
            plugins_[i].plugin.reset();
            plugins_[i].active = false;
        }
    }

    pluginCount_ = 0;
    activePlugins_ = 0;
    return result;
}

Config::Result<void> PluginRegistry::updateAllPlugins(float deltaTime) {
    Config::Result<void> result;

    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin && plugins_[i].active) {
            auto updateResult = plugins_[i].plugin->update(deltaTime);
            if (!updateResult) {
                result = updateResult; // Store last error
                plugins_[i].errorCount++;
            }
        }
    }

    return result;
}

Config::Result<void> PluginRegistry::handlePluginError(const char* pluginName, ErrorCode error) {
    auto* entry = findPluginEntry(pluginName);
    if (!entry) {
        return Config::Result<void>(ErrorCode::NOT_INITIALIZED);
    }

    entry->errorCount++;

    // Attempt plugin recovery
    if (entry->plugin) {
        return entry->plugin->reset();
    }

    return Config::Result<void>(error);
}

PluginRegistry& PluginRegistry::getInstance() {
    static PluginRegistry instance;
    return instance;
}

PluginRegistry::PluginEntry* PluginRegistry::findPluginEntry(const char* name) {
    for (size_t i = 0; i < pluginCount_; ++i) {
        if (plugins_[i].plugin &&
            strcmp(plugins_[i].info.name, name) == 0) {
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

    // Validate plugin
    auto validationResult = validatePlugin(plugin.get());
    if (!validationResult) {
        return validationResult;
    }

    // Check dependencies
    const auto& info = plugin->getInfo();
    auto depResult = checkDependencies(info);
    if (!depResult) {
        return depResult;
    }

    // Add to registry
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

// MotorControllerPluginFactory implementation
std::unique_ptr<IMotorControllerPlugin> MotorControllerPluginFactory::createPlugin(const char* motorType) const {
    for (size_t i = 0; i < creatorCount_; ++i) {
        if (strcmp(creators_[i].motorType, motorType) == 0) {
            return creators_[i].creator();
        }
    }
    return nullptr;
}

bool MotorControllerPluginFactory::supportsMotorType(const char* motorType) const {
    for (size_t i = 0; i < creatorCount_; ++i) {
        if (strcmp(creators_[i].motorType, motorType) == 0) {
            return true;
        }
    }
    return false;
}

void MotorControllerPluginFactory::enumerateSupportedTypes(std::function<void(const char*)> callback) const {
    for (size_t i = 0; i < creatorCount_; ++i) {
        callback(creators_[i].motorType);
    }
}

MotorControllerPluginFactory& MotorControllerPluginFactory::getInstance() {
    static MotorControllerPluginFactory instance;
    return instance;
}

// PluginSystem implementation
PluginSystem::PluginSystem()
    : registry_(PluginRegistry::getInstance()),
      motorFactory_(MotorControllerPluginFactory::getInstance()) {
}

Config::Result<void> PluginSystem::initialize(const PluginSystemConfig& config) {
    if (initialized_) {
        return Config::Result<void>(ErrorCode::ALREADY_INITIALIZED);
    }

    config_ = config;

    // Load core plugins
    auto coreResult = loadCorePlugins();
    if (!coreResult) {
        return coreResult;
    }

    // Auto-load plugins if enabled
    if (config_.autoLoadEnabled) {
        auto autoLoadResult = registry_.autoLoadPlugins();
        if (!autoLoadResult) {
            return autoLoadResult;
        }
    }

    // Initialize all plugins
    auto initResult = registry_.initializeAllPlugins();
    if (!initResult) {
        return initResult;
    }

    // Start all plugins
    auto startResult = registry_.startAllPlugins();
    if (!startResult) {
        return startResult;
    }

    initialized_ = true;
    return Config::Result<void>();
}

Config::Result<void> PluginSystem::shutdown() {
    if (!initialized_) {
        return Config::Result<void>();
    }

    auto result = registry_.shutdownAllPlugins();
    initialized_ = false;
    fallbackMode_ = false;

    return result;
}

Config::Result<void> PluginSystem::update(float deltaTime) {
    if (!initialized_) {
        return Config::Result<void>(ErrorCode::NOT_INITIALIZED);
    }

    return registry_.updateAllPlugins(deltaTime);
}

Config::Result<void> PluginSystem::retry() {
    if (fallbackMode_) {
        fallbackMode_ = false;
        return registry_.startAllPlugins();
    }
    return Config::Result<void>();
}

Config::Result<void> PluginSystem::reset() {
    auto stopResult = registry_.stopAllPlugins();
    if (!stopResult) {
        return stopResult;
    }

    return registry_.startAllPlugins();
}

Config::Result<void> PluginSystem::reinitialize() {
    auto shutdownResult = shutdown();
    if (!shutdownResult) {
        return shutdownResult;
    }

    return initialize(config_);
}

Config::Result<void> PluginSystem::enterFallbackMode() {
    // Stop non-essential plugins
    registry_.stopAllPlugins();
    fallbackMode_ = true;

    // Load only core plugins would be implemented here
    return loadCorePlugins();
}

bool PluginSystem::isHealthy() const {
    if (!initialized_ || fallbackMode_) {
        return false;
    }

    // Check if all active plugins are healthy
    bool healthy = true;
    registry_.enumeratePlugins([&healthy](const PluginInfo& info, bool active) {
        if (active) {
            // Would check plugin health here
            // For now, assume healthy if active
        }
    });

    return healthy;
}

PluginSystem& PluginSystem::getInstance() {
    static PluginSystem instance;
    return instance;
}

Config::Result<void> PluginSystem::loadCorePlugins() {
    // Load essential plugins that are always needed
    // This would typically load built-in motor controllers, basic message handlers, etc.
    return Config::Result<void>();
}

Config::Result<void> PluginSystem::validateSystemHealth() {
    // Validate that all critical plugins are running
    return Config::Result<void>();
}

// Global instance
PluginSystem& pluginSystem = PluginSystem::getInstance();

} // namespace Config