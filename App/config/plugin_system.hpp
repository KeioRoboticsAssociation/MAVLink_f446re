#pragma once

#include "system_config.hpp"
#include "error_recovery.hpp"
#include "../motors/base/motor_interface.hpp"
#include "../comm/message_dispatcher.hpp"
#include <cstdint>
#include <functional>
#include <array>
#include <string>

namespace Config {

// Plugin types
enum class PluginType : uint8_t {
    MOTOR_CONTROLLER = 0,
    MESSAGE_HANDLER = 1,
    SENSOR_DRIVER = 2,
    ACTUATOR_DRIVER = 3,
    COMMUNICATION_PROTOCOL = 4,
    SAFETY_MONITOR = 5,
    TELEMETRY_PROVIDER = 6,
    MAX_PLUGIN_TYPES = 7
};

// Plugin metadata
struct PluginInfo {
    const char* name;
    const char* version;
    const char* author;
    const char* description;
    PluginType type;
    uint32_t apiVersion;
    uint32_t flags;

    // Dependency information
    struct Dependency {
        const char* name;
        const char* minVersion;
        bool required;
    };
    const Dependency* dependencies;
    size_t dependencyCount;
};

// Plugin interface base class
class IPlugin {
public:
    virtual ~IPlugin() = default;

    // Plugin lifecycle
    virtual Config::Result<void> initialize() = 0;
    virtual Config::Result<void> start() = 0;
    virtual Config::Result<void> stop() = 0;
    virtual Config::Result<void> shutdown() = 0;

    // Plugin information
    virtual const PluginInfo& getInfo() const = 0;
    virtual bool isCompatible(uint32_t systemApiVersion) const = 0;

    // Status
    virtual bool isInitialized() const = 0;
    virtual bool isRunning() const = 0;
    virtual bool isHealthy() const = 0;

    // Configuration
    virtual Config::Result<void> configure(const char* configData) = 0;
    virtual Config::Result<void> update(float deltaTime) = 0;

    // Error recovery support
    virtual Config::Result<void> reset() = 0;
    virtual Config::Result<void> diagnose() = 0;
};

// Motor controller plugin interface
class IMotorControllerPlugin : public IPlugin {
public:
    virtual std::unique_ptr<Motors::IMotorControllerBase> createController(uint8_t id) = 0;
    virtual bool supportsMotorType(const char* motorType) const = 0;
    virtual Config::Result<void> configureMotor(uint8_t id, const char* config) = 0;
};

// Message handler plugin interface
class IMessageHandlerPlugin : public IPlugin {
public:
    virtual std::unique_ptr<Communication::IMessageHandler> createHandler() = 0;
    virtual bool supportsMessageType(uint32_t messageId) const = 0;
    virtual uint8_t getPriority() const = 0;
};

// Plugin registry for managing loaded plugins
class PluginRegistry {
private:
    struct PluginEntry {
        std::unique_ptr<IPlugin> plugin;
        PluginInfo info;
        bool active;
        uint32_t loadTime;
        uint32_t errorCount;
    };

    std::array<PluginEntry, 32> plugins_;
    size_t pluginCount_ = 0;

    // Plugin discovery
    std::array<const char*, 16> searchPaths_;
    size_t searchPathCount_ = 0;

    // Statistics
    uint32_t totalPluginsLoaded_ = 0;
    uint32_t activePlugins_ = 0;

public:
    PluginRegistry();

    // Plugin management
    Config::Result<void> loadPlugin(const char* pluginPath);
    Config::Result<void> unloadPlugin(const char* pluginName);
    Config::Result<void> enablePlugin(const char* pluginName);
    Config::Result<void> disablePlugin(const char* pluginName);

    // Plugin discovery
    Config::Result<void> addSearchPath(const char* path);
    Config::Result<void> scanForPlugins();
    Config::Result<void> autoLoadPlugins();

    // Plugin access
    IPlugin* findPlugin(const char* name) const;
    template<typename T>
    T* findPlugin(const char* name) const {
        auto* plugin = findPlugin(name);
        return plugin ? dynamic_cast<T*>(plugin) : nullptr;
    }

    // Plugin enumeration
    void enumeratePlugins(std::function<void(const PluginInfo&, bool active)> callback) const;
    void enumeratePluginsByType(PluginType type,
                               std::function<void(const PluginInfo&, IPlugin*)> callback) const;

    // Status and statistics
    size_t getPluginCount() const { return pluginCount_; }
    size_t getActivePluginCount() const { return activePlugins_; }
    bool isPluginLoaded(const char* name) const;
    bool isPluginActive(const char* name) const;

    // Plugin validation
    Config::Result<void> validatePlugin(IPlugin* plugin) const;
    Config::Result<void> checkDependencies(const PluginInfo& info) const;

    // Lifecycle management
    Config::Result<void> initializeAllPlugins();
    Config::Result<void> startAllPlugins();
    Config::Result<void> stopAllPlugins();
    Config::Result<void> shutdownAllPlugins();
    Config::Result<void> updateAllPlugins(float deltaTime);

    // Error handling
    Config::Result<void> handlePluginError(const char* pluginName, ErrorCode error);

    // Singleton access
    static PluginRegistry& getInstance();

private:
    PluginEntry* findPluginEntry(const char* name);
    Config::Result<void> registerPlugin(std::unique_ptr<IPlugin> plugin);
    Config::Result<void> validatePluginInfo(const PluginInfo& info) const;
    void updateStatistics();
};

// Plugin factory for creating motor controller plugins
class MotorControllerPluginFactory {
private:
    struct PluginCreator {
        const char* motorType;
        std::function<std::unique_ptr<IMotorControllerPlugin>()> creator;
    };

    std::array<PluginCreator, 16> creators_;
    size_t creatorCount_ = 0;

public:
    // Registration
    template<typename T>
    Config::Result<void> registerPlugin(const char* motorType) {
        if (creatorCount_ >= creators_.size()) {
            return Config::Result<void>(ErrorCode::RESOURCE_EXHAUSTED);
        }

        creators_[creatorCount_++] = {
            motorType,
            []() -> std::unique_ptr<IMotorControllerPlugin> {
                return std::make_unique<T>();
            }
        };

        return Config::Result<void>();
    }

    // Plugin creation
    std::unique_ptr<IMotorControllerPlugin> createPlugin(const char* motorType) const;
    bool supportsMotorType(const char* motorType) const;

    // Enumeration
    void enumerateSupportedTypes(std::function<void(const char*)> callback) const;

    // Singleton access
    static MotorControllerPluginFactory& getInstance();
};

// Plugin system configuration
struct PluginSystemConfig {
    bool autoLoadEnabled = true;
    bool dependencyCheckEnabled = true;
    bool errorRecoveryEnabled = true;
    uint32_t maxPlugins = 32;
    uint32_t maxSearchPaths = 16;
    const char* defaultSearchPath = "/plugins";
    uint32_t pluginTimeoutMs = 5000;
};

// Main plugin system manager
class PluginSystem : public IRecoverableComponent {
private:
    PluginRegistry& registry_;
    MotorControllerPluginFactory& motorFactory_;
    PluginSystemConfig config_;
    bool initialized_ = false;
    bool fallbackMode_ = false;

public:
    PluginSystem();

    // System lifecycle
    Config::Result<void> initialize(const PluginSystemConfig& config = {});
    Config::Result<void> shutdown();
    Config::Result<void> update(float deltaTime);

    // Plugin management
    PluginRegistry& getRegistry() { return registry_; }
    MotorControllerPluginFactory& getMotorFactory() { return motorFactory_; }

    // Configuration
    void setConfig(const PluginSystemConfig& config) { config_ = config; }
    const PluginSystemConfig& getConfig() const { return config_; }

    // IRecoverableComponent implementation
    Config::Result<void> retry() override;
    Config::Result<void> reset() override;
    Config::Result<void> reinitialize() override;
    Config::Result<void> enterFallbackMode() override;
    bool isInFallbackMode() const override { return fallbackMode_; }
    const char* getComponentName() const override { return "PluginSystem"; }

    // Status
    bool isInitialized() const { return initialized_; }
    bool isHealthy() const;

    // Singleton access
    static PluginSystem& getInstance();

private:
    Config::Result<void> loadCorePlugins();
    Config::Result<void> validateSystemHealth();
};

// Global plugin system instance
extern PluginSystem& pluginSystem;

// Plugin registration macros for easier plugin development
#define REGISTER_MOTOR_PLUGIN(PluginClass, MotorType) \
    static bool registered_##PluginClass = []() { \
        auto& factory = Config::MotorControllerPluginFactory::getInstance(); \
        return factory.registerPlugin<PluginClass>(MotorType).isOk(); \
    }();

#define DECLARE_PLUGIN_INFO(Name, Version, Author, Description, Type) \
    static const Config::PluginInfo plugin_info = { \
        Name, Version, Author, Description, Type, \
        1, 0, nullptr, 0 \
    };

} // namespace Config