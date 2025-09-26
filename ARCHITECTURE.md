# STM32 MAVLink Robot Architecture Documentation

## Overview

This project implements a modern, scalable architecture for STM32-based robots with MAVLink communication. The architecture features compile-time configuration, plugin-based device management, and unified communication handling.

## Architecture Principles

### 1. Configuration-Driven Design
- **Compile-time configuration**: All device parameters defined at compile time
- **Type safety**: Configuration validation using C++ templates and static_assert
- **Zero runtime overhead**: No dynamic configuration loading or parsing
- **Template-based**: Easy robot customization through configuration templates

### 2. Plugin-Based Device Management
- **Unified interfaces**: Common motor interface for all device types
- **Factory pattern**: Configuration-driven device instantiation
- **Device registry**: Type-safe motor storage with unified control
- **Extensibility**: Easy addition of new motor types without core changes

### 3. Clean Architecture
- **Dependency injection**: SystemContext manages all subsystem dependencies
- **Error handling**: Comprehensive error propagation with Result<T> types
- **Hardware abstraction**: Clean separation between HAL and business logic
- **Single responsibility**: Each component has a clear, focused purpose

## Directory Structure

```
App/
├── config/                    # Configuration system
│   ├── robot_config.hpp      # Main robot configuration
│   ├── motor_config.hpp      # Motor instance mapping
│   ├── system_config.hpp     # System-wide settings
│   └── templates/            # Robot configuration templates
│       ├── basic_servo_robot.hpp
│       ├── precision_dc_robot.hpp
│       ├── hybrid_robot.hpp
│       └── template_generator.hpp
├── hal/                      # Hardware Abstraction Layer
│   ├── hardware_manager.hpp  # Unified hardware interface
│   └── hardware_manager.cpp
├── motors/                   # Motor subsystem
│   └── base/
│       ├── motor_interface.hpp    # Common motor interface
│       ├── motor_factory.hpp      # Motor factory implementation
│       ├── motor_factory.cpp
│       └── motor_registry.cpp
├── comm/                     # Communication subsystem
│   ├── unified_mavlink_handler.hpp  # Unified MAVLink manager
│   └── unified_mavlink_handler.cpp
├── safety/                   # Safety management
├── system_context.hpp        # Main system orchestrator
└── main_app.cpp             # System implementation
```

## Key Components

### SystemContext
The central orchestrator that manages all subsystems:

```cpp
class SystemContext {
public:
    struct Hardware { ... } hardware;     // HAL management
    struct Motors { ... } motors;         // Motor registry & factory
    struct Communication { ... } comm;    // MAVLink handling
    struct Safety { ... } safety;         // Safety monitoring

    Config::Result<ErrorCode> initialize();
    Config::Result<ErrorCode> update();
    void shutdown();
};
```

### Configuration System
Compile-time configuration using C++ templates:

```cpp
namespace Config {
    // Device-specific configurations
    constexpr std::array<ServoInstanceConfig, 4> SERVO_CONFIGS = {{ ... }};
    constexpr std::array<DCMotorInstanceConfig, 1> DC_MOTOR_CONFIGS = {{ ... }};

    // Hardware resource mapping
    constexpr std::array<MotorInstance, 6> MOTOR_INSTANCES = {{ ... }};

    // System settings
    namespace Robot {
        static constexpr const char* ROBOT_NAME = "MAVLink Test Robot";
        static constexpr uint8_t MAX_SERVOS = 4;
        // ...
    }
}
```

### Motor Interface
Unified interface for all motor types:

```cpp
template<typename TConfig>
class IMotorController {
public:
    virtual Config::Result<MotorStatus> initialize(const TConfig& config) = 0;
    virtual Config::Result<MotorStatus> update(float deltaTime) = 0;
    virtual Config::Result<MotorStatus> setCommand(const MotorCommand& cmd) = 0;
    virtual BaseMotorState getState() const = 0;
    // ...
};
```

### Unified MAVLink Handler
Single MAVLink parser with device routing:

```cpp
class UnifiedMAVLinkHandler {
private:
    std::array<IMAVLinkDevice*, MAX_MOTORS> devices_;
    RingBuffer<RING_BUFFER_SIZE> rxBuffer_, txBuffer_;

public:
    Config::Result<ErrorCode> registerDevice(IMAVLinkDevice* device);
    void processReceivedByte(uint8_t byte);
    void handleReceivedMessage(const mavlink_message_t& msg);
    Config::Result<ErrorCode> sendTelemetry();
};
```

## Creating a New Robot Configuration

### Method 1: Using Templates

1. Choose a template in `App/config/templates/`
2. Copy and modify for your robot:

```cpp
// my_robot_config.hpp
#include "templates/template_generator.hpp"

#define USE_BASIC_SERVO_ROBOT  // or other template
#include "templates/template_generator.hpp"

// Customize as needed
namespace MyRobot {
    static constexpr const char* ROBOT_NAME = "My Custom Robot";
    // Override specific configurations...
}
```

### Method 2: From Scratch

1. Define device configurations:

```cpp
constexpr std::array<Devices::ServoInstanceConfig, 2> MY_SERVO_CONFIGS = {{
    GENERATE_SERVO_CONFIG(1, "Pan", -90.0f, 90.0f, 1500, 120.0f, 240.0f),
    GENERATE_SERVO_CONFIG(2, "Tilt", -45.0f, 45.0f, 1500, 90.0f, 180.0f)
}};
```

2. Map to hardware resources:

```cpp
constexpr std::array<MotorInstance, 2> MY_MOTOR_INSTANCES = {{
    GENERATE_MOTOR_INSTANCE(1, SERVO, 2, 1),  // TIM2 CH1
    GENERATE_MOTOR_INSTANCE(2, SERVO, 2, 2)   // TIM2 CH2
}};
```

3. Update `robot_config.hpp` to use your configuration

## Configuration Validation

The system includes compile-time validation:

```cpp
// Automatic validation
static_assert(SERVO_CONFIGS.size() >= 1, "At least one servo required");
static_assert(MAX_SERVOS >= SERVO_CONFIGS.size(), "Servo limit exceeded");

// Runtime validation helpers
constexpr bool valid = validateServoConfigs(SERVO_CONFIGS);
```

## Available Robot Templates

### Basic Servo Robot
- **Purpose**: Simple 2-servo camera mount or pan-tilt system
- **Components**: 2 servos on TIM2
- **Features**: Basic position control with watchdog protection
- **Use case**: Camera gimbals, sensor pointing systems

### Precision DC Robot
- **Purpose**: High-precision positioning with encoder feedback
- **Components**: 2 DC motors + encoders, 200Hz control loop
- **Features**: Advanced PID control, high-resolution encoders
- **Use case**: CNC machines, robotic arms, precision actuators

### Hybrid Robot
- **Purpose**: Complex multi-motor system demonstration
- **Components**: 2 servos + 1 DC motor + 2 RoboMaster motors
- **Features**: Camera gimbal + lift mechanism + mobile base
- **Use case**: Mobile manipulation robots, inspection platforms

## Error Handling

The architecture uses a comprehensive Result<T> type system:

```cpp
Config::Result<MotorStatus> result = motor.initialize(config);
if (!result) {
    handleError(result.error());
    return result.error();
}
```

Error codes include:
- `OK`: Success
- `NOT_INITIALIZED`: Component not initialized
- `HARDWARE_ERROR`: Hardware failure
- `TIMEOUT`: Watchdog or communication timeout
- `CONFIG_ERROR`: Invalid configuration
- `SAFETY_VIOLATION`: Safety limit exceeded

## Safety Features

- **Watchdog timers**: Per-device timeout monitoring
- **Emergency stop**: System-wide emergency shutdown
- **Limit checking**: Position, velocity, and current limits
- **Graceful degradation**: Safe fallback behaviors
- **State monitoring**: Continuous health checking

## Performance Characteristics

- **Memory usage**: ~2.7KB RAM, ~30KB Flash
- **Loop frequency**: 100Hz main loop (configurable)
- **Latency**: <10ms command to response
- **Throughput**: 115200 baud MAVLink communication
- **Scalability**: Up to 8 motors supported

## Integration with Existing Code

The architecture is designed for gradual migration:

1. **Phase 1**: Replace global variables with SystemContext
2. **Phase 2**: Migrate to compile-time configuration
3. **Phase 3**: Adopt unified MAVLink and plugin architecture
4. **Phase 4**: Customize using templates for new robots

## Best Practices

1. **Configuration**: Always validate configurations at compile time
2. **Error handling**: Check all Result<T> return values
3. **Safety**: Implement appropriate watchdog timeouts
4. **Performance**: Use appropriate loop frequencies for your application
5. **Modularity**: Keep device-specific code in separate modules
6. **Documentation**: Document your robot configuration clearly

## Troubleshooting

### Common Issues

1. **Compilation errors with templates**: Check namespace scoping (use `::Motors::`)
2. **Motor not responding**: Verify hardware mapping in MOTOR_INSTANCES
3. **MAVLink communication issues**: Check baud rate and system/component IDs
4. **Safety violations**: Review limit settings in device configurations

### Debugging Tools

- Enable debug logging: `#define DEBUG` in CMakeLists.txt
- Use compile-time assertions for configuration validation
- Monitor system state through MAVLink telemetry
- Check error counts in SystemContext state

This architecture provides a robust foundation for STM32-based robots with room for customization and expansion while maintaining performance and safety.