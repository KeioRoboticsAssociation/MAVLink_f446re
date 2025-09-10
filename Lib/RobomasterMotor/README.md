# RoboMaster Motor Library

A comprehensive C++ library for controlling RoboMaster motors on STM32 platforms, designed following the ServoMotor library's architecture with individual motor control, safety features, and flexible configuration.

## Features

### Individual Motor Control
- Each motor is a separate object with independent configuration
- Support for motors 1-8 with any combination
- Individual PID tuning per motor
- Per-motor safety limits and watchdog

### Control Modes
- **Current Control**: Direct current/torque control
- **Velocity Control**: Speed control with PID regulation  
- **Position Control**: Multi-turn position control with cascade PID

### Safety Features
- Configurable watchdog timeout with fail-safe behaviors
- Thermal protection with temperature monitoring
- Current limiting and overcurrent detection
- Position limits (optional)
- Rate limiting for smooth acceleration

### Advanced Features
- Multi-turn position tracking with automatic wrap detection
- Configurable PID gains (Kp, Ki, Kd) for position and velocity loops
- Anti-windup protection for integral terms
- Statistical monitoring (saturation, timeouts, errors)
- Direction inversion and position offset support

## Architecture

The library consists of two main classes:

### `RoboMasterMotor`
Individual motor control class similar to `ServoMotor`:
- Motor-specific configuration and state
- Control loop implementation
- Safety monitoring
- CAN data processing

### `RoboMasterCANManager`
Centralized CAN communication manager:
- Handles CAN bus initialization and communication
- Manages motor registration and message routing  
- Efficient group messaging for multiple motors
- Error handling and diagnostics

## Basic Usage

```cpp
#include "RoboMasterMotor.hpp"
#include "RoboMasterCANManager.hpp"

// Global objects
RoboMasterCANManager can_manager;
RoboMasterMotor motor1;

void setup() {
    // Initialize CAN manager
    can_manager.init(&hcan1);
    can_manager.start();
    
    // Create and configure motor
    motor1.create(1, &can_manager);
    motor1.init();  // Use default config
    motor1.setEnabled(true);
}

void loop() {
    // Update CAN manager (handles all communication)
    can_manager.update();
    
    // Control motor
    motor1.setVelocityRPS(10.0f);  // 10 RPS target velocity
}
```

## Advanced Configuration

```cpp
void setup_advanced() {
    // Custom motor configuration
    RoboMasterConfig config;
    config.maxVelocityRPS = 50.0f;
    config.maxCurrent = 8000;
    config.positionKp = 20.0f;
    config.velocityKp = 40.0f;
    config.velocityKi = 0.3f;
    config.watchdogTimeoutMs = 500;
    config.failSafeBehavior = FailSafeBehavior::BRAKE;
    
    // Position limits (optional)
    config.positionLimitsEnabled = true;
    config.minPositionRad = -5.0f * M_PI;
    config.maxPositionRad = 5.0f * M_PI;
    
    motor1.create(1, &can_manager);
    motor1.init(config);
}
```

## Control Modes

### Velocity Control
```cpp
motor1.setControlMode(RoboMasterControlMode::VELOCITY);
motor1.setVelocityRPS(25.0f);  // 25 RPS target
```

### Position Control  
```cpp
motor1.setControlMode(RoboMasterControlMode::POSITION);
motor1.setPositionRad(2.0f * M_PI);  // 2 full rotations
```

### Current Control
```cpp
motor1.setControlMode(RoboMasterControlMode::CURRENT);
motor1.setCurrent(5000);  // 5000mA torque current
```

## Status Monitoring

```cpp
void monitor_motor() {
    // Get current measurements
    float position = motor1.getCurrentPosition();
    float velocity = motor1.getCurrentVelocity();
    int16_t current = motor1.getCurrentMilliamps();
    uint8_t temperature = motor1.getTemperature();
    
    // Check status
    RoboMasterStatus status = motor1.getStatus();
    if (status != RoboMasterStatus::OK) {
        switch (status) {
            case RoboMasterStatus::TIMEOUT:
                motor1.resetWatchdog();
                break;
            case RoboMasterStatus::OVERHEAT:
                motor1.setEnabled(false);
                break;
            // Handle other errors...
        }
    }
    
    // Get statistics
    const RoboMasterState& state = motor1.getState();
    printf("Saturations: %u, Timeouts: %u\n", 
           state.saturationCount, state.timeoutCount);
}
```

## CAN Integration

The library requires CAN interrupt handling:

```cpp
// In your CAN interrupt handler
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        can_manager.handleCANReceive();
    }
}
```

## Migration from Legacy API

The old `RoboMasterController` class is deprecated. To migrate:

### Old API:
```cpp
RoboMasterController controller(&hcan1);
controller.start();
controller.setTargetSpeed(0, 10.0f);  // Motor 0 (ID 1)
controller.timer_callback();
```

### New API:
```cpp
RoboMasterCANManager can_manager;
RoboMasterMotor motor1;

can_manager.init(&hcan1);
can_manager.start();
motor1.create(1, &can_manager);
motor1.init();
motor1.setVelocityRPS(10.0f);
can_manager.update();
```

## Configuration Options

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `maxVelocityRPS` | Maximum velocity | 100.0 | 0.1 - 1000 |
| `maxAccelerationRPS2` | Max acceleration | 200.0 | 1.0 - 10000 |
| `maxCurrent` | Current limit (mA) | 16000 | 100 - 20000 |
| `positionKp` | Position P gain | 10.0 | 0.0 - 1000 |
| `velocityKp` | Velocity P gain | 35.0 | 0.0 - 1000 |
| `velocityKi` | Velocity I gain | 0.15 | 0.0 - 100 |
| `watchdogTimeoutMs` | Watchdog timeout | 1000 | 50 - 5000 |
| `maxTemperature` | Temperature limit | 80 | 40 - 100 |

## Error Handling

The library provides comprehensive error detection:

- `RoboMasterStatus::OK` - Normal operation
- `RoboMasterStatus::TIMEOUT` - Communication timeout
- `RoboMasterStatus::OVERHEAT` - Temperature too high
- `RoboMasterStatus::OVERCURRENT` - Current limit exceeded  
- `RoboMasterStatus::CONFIG_ERROR` - Invalid configuration
- `RoboMasterStatus::CAN_ERROR` - CAN communication error

## Performance Notes

- The CAN manager efficiently groups motor commands to minimize bus traffic
- Update rate should be 100Hz or higher for smooth control
- Position control uses cascade PID (position → velocity → current)
- Built-in rate limiting prevents excessive acceleration
- Multi-turn position tracking handles unlimited rotation

## Hardware Requirements

- STM32F4xx or compatible microcontroller
- CAN peripheral configured for 1Mbps
- RoboMaster motors (M2006, M3508, etc.)
- Proper CAN bus termination and wiring

## Thread Safety

The library is designed for single-threaded operation. If using multiple threads:
- Call `can_manager.update()` from one thread only
- Motor control commands are thread-safe
- CAN interrupt handler is independent

See `Examples/example_usage.cpp` for complete working examples.