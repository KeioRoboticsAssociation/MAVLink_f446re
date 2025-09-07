# ServoMotor Library

A comprehensive STM32 HAL-based servo motor control library designed for embedded applications requiring precise PWM servo control with external configuration support.

## Features

- **STM32 HAL Integration**: Uses standard STM32 HAL timer and PWM functions
- **Multi-Instance Support**: Control multiple servos simultaneously with unique IDs
- **External Configuration**: JSON-based configuration with per-servo-ID settings
- **Safety Features**: Watchdog timeout, fail-safe behaviors, soft limits
- **Rate Limiting**: Configurable velocity and acceleration limits
- **Flexible Control**: Angle (degrees/radians) or direct pulse width control
- **Diagnostics**: Error tracking, saturation monitoring, state reporting

## Quick Start

### 1. Basic Setup

```cpp
#include "ServoMotor.hpp"

ServoMotor servo;
TIM_HandleTypeDef htim1; // Your configured timer

// Create and initialize servo
servo.create(1, &htim1, TIM_CHANNEL_1);
servo.init(); // Uses default configuration
```

### 2. With JSON Configuration

```cpp
// Load configuration from file for servo ID 1
servo.loadConfigFromFileForId("servo_config.json");

// Or load from JSON string
const char* config = R"({
  "1": {
    "initial_offset": 10.0,
    "min_angle": 0.0,
    "max_angle": 90.0
  }
})";
servo.loadConfigFromJson(config);
```

### 3. Control Servo

```cpp
// Angle control
servo.setAngleDeg(45.0f);          // Set to 45 degrees
servo.setAngleRad(M_PI/4);         // Set to π/4 radians

// Direct pulse control
servo.setPulseUs(1500);            // Set to 1.5ms pulse

// Enable/disable
servo.setEnabled(true);
servo.setEnabled(false);

// Update loop (call periodically)
servo.update(); // Handles rate limiting and safety
```

## Configuration

### JSON Configuration Format

The library supports per-servo-ID configuration:

```json
{
  "1": {
    "initial_offset": 10.0,
    "min_angle": 0.0,
    "max_angle": 30.0,
    "pulseMinUs": 1000,
    "pulseMaxUs": 2000,
    "pulseNeutralUs": 1500,
    "directionInverted": false,
    "maxVelocityDegPerS": 180.0,
    "maxAccelerationDegPerS2": 360.0,
    "watchdogTimeoutMs": 500,
    "failSafeBehavior": "NEUTRAL_POSITION",
    "startupAngleDeg": 10.0,
    "startDisabled": false
  },
  "2": {
    "initial_offset": 90.0,
    "min_angle": 45.0,
    "max_angle": 135.0
  }
}
```

### Configuration Parameters

| Parameter | Type | Description | Default |
|-----------|------|-------------|---------|
| `initial_offset` / `offsetDeg` | float | Mechanical offset in degrees | 0.0 |
| `min_angle` / `angleMinDeg` | float | Minimum angle limit (degrees) | -90.0 |
| `max_angle` / `angleMaxDeg` | float | Maximum angle limit (degrees) | 90.0 |
| `pulseMinUs` | uint16_t | Minimum pulse width (μs) | 1000 |
| `pulseMaxUs` | uint16_t | Maximum pulse width (μs) | 2000 |
| `pulseNeutralUs` | uint16_t | Neutral pulse width (μs) | 1500 |
| `directionInverted` | bool | Invert servo direction | false |
| `maxVelocityDegPerS` | float | Max angular velocity (°/s) | 180.0 |
| `maxAccelerationDegPerS2` | float | Max acceleration (°/s²) | 360.0 |
| `watchdogTimeoutMs` | uint32_t | Watchdog timeout (ms) | 500 |
| `failSafeBehavior` | string | Fail-safe action | "NEUTRAL_POSITION" |
| `startupAngleDeg` | float | Initial angle on startup | 0.0 |
| `startDisabled` | bool | Start with output disabled | false |

### Fail-Safe Behaviors

- `"HOLD_POSITION"`: Maintain last commanded position
- `"NEUTRAL_POSITION"`: Move to neutral position (pulseNeutralUs)
- `"DISABLE_OUTPUT"`: Disable PWM output (CCR = 0)

## API Reference

### Core Methods

```cpp
// Initialization
ServoStatus create(uint8_t id, TIM_HandleTypeDef* htim, uint32_t channel);
ServoStatus init();
ServoStatus init(const ServoConfig& config);

// Control
ServoStatus setAngleDeg(float angleDeg);
ServoStatus setAngleRad(float angleRad);
ServoStatus setPulseUs(uint16_t pulseUs);
ServoStatus setEnabled(bool enabled);

// Configuration
ServoStatus loadConfigFromFileForId(const char* filePath);
ServoStatus loadConfigFromJson(const char* jsonString);
ServoStatus saveConfigToFile(const char* filePath) const;

// State Query
float getAngleCmd() const;
float getCurrentAngle() const;
uint16_t getPulseUs() const;
ServoLimits getLimits() const;
ServoStatus getStatus() const;
const ServoState& getState() const;

// Maintenance
void update();           // Call periodically for rate limiting/safety
void resetWatchdog();    // Reset watchdog timer
```

### Status Codes

```cpp
enum class ServoStatus {
    OK = 0,              // Operation successful
    NOT_INITIALIZED,     // Servo not initialized
    TIMER_ERROR,         // Timer configuration error
    OUT_OF_RANGE,        // Command out of valid range
    TIMEOUT,             // Watchdog timeout occurred
    CONFIG_ERROR         // Configuration validation failed
};
```

## Timer Configuration

The library automatically retrieves timer parameters from the HAL timer handle:

```cpp
// Timer configuration is read from:
timerPeriod_ = htim_->Init.Period;      // ARR register
timerPrescaler_ = htim_->Init.Prescaler; // PSC register
```

For 50Hz servo control (20ms period), typical timer setup:
- **Period (ARR)**: 20000 - 1
- **Prescaler (PSC)**: 84 - 1 (for 84MHz clock → 1MHz timer)
- **Resolution**: 1μs per tick

## Safety Features

### Watchdog Protection
- Automatically applies fail-safe if no commands received within timeout
- Configurable timeout period and fail-safe behavior
- Automatic recovery when communication resumes

### Soft Limits
- Commands automatically clipped to configured angle/pulse ranges
- Saturation events counted for diagnostics
- No risk of mechanical damage from out-of-range commands

### Rate Limiting
- Smooth servo movement with velocity and acceleration limits
- Reduces mechanical stress and power consumption
- Prevents sudden jerky movements

## Multi-Servo Example

```cpp
ServoMotor servos[4];
TIM_HandleTypeDef htim1, htim2;

// Initialize multiple servos
servos[0].create(1, &htim1, TIM_CHANNEL_1);
servos[1].create(2, &htim1, TIM_CHANNEL_2);
servos[2].create(3, &htim2, TIM_CHANNEL_1);
servos[3].create(4, &htim2, TIM_CHANNEL_2);

// Load configurations
for (int i = 0; i < 4; i++) {
    servos[i].loadConfigFromFileForId("servo_config.json");
    servos[i].init();
}

// Control loop
while (1) {
    // Update all servos
    for (int i = 0; i < 4; i++) {
        servos[i].update();
    }
    
    // Set positions
    servos[0].setAngleDeg(45.0f);
    servos[1].setAngleDeg(-30.0f);
    
    HAL_Delay(10); // 100Hz update rate
}
```

## File Structure

```
ServoMotor/
├── Inc/
│   ├── ServoMotor.hpp      # Main servo class
│   ├── ServoConfig.hpp     # Configuration parser
│   └── servo_config.json   # Example configuration
├── Src/
│   ├── ServoMotor.cpp      # Servo implementation
│   └── ServoConfig.cpp     # Configuration parser implementation
└── README.md               # This file
```

## Requirements

- STM32 HAL library
- C++20 compiler support
- Timer configured for PWM output
- Sufficient timer resolution for servo control (typically 1μs)

## Performance

- **Response Delay**: <5ms from command to CCR update
- **PWM Jitter**: <50μs (target <20μs)
- **Angle Resolution**: ≤0.3% of range (depends on timer ARR)
- **Memory Usage**: ~200 bytes per servo instance

## License

This library is provided as-is for embedded servo control applications.