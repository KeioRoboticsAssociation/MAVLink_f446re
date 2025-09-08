# Encoder Library

This library provides encoder support for STM32F446RETx with MAVLink communication protocol integration. It supports both TIM encoder mode and external counter configurations for A/B/Z signal acquisition.

## Features

- **Configurable Encoder Parameters**: CPR (Counts Per Revolution), signal polarity inversion (invertA, invertB), Z signal usage
- **Multiple Interface Modes**: TIM encoder mode or external counter
- **JSON Configuration**: Persistent configuration storage and loading
- **MAVLink Integration**: Remote configuration and status monitoring via MAVLink protocol
- **Position Tracking**: Real-time position, angle (rad/deg), and revolution counting
- **Multi-instance Support**: Support for multiple encoders identified by motor ID
- **Watchdog Protection**: Timeout detection and error handling

## Architecture

### Core Classes

- **`Encoder`**: Main encoder class handling position tracking and configuration
- **`EncoderConfig`**: Configuration structure with JSON parsing support
- **`EncoderConfigParser`**: JSON configuration file parser and serializer
- **`MAVLinkEncoderController`**: MAVLink protocol handler for remote communication

### Configuration Structure

```cpp
struct EncoderConfig {
    uint16_t cpr = 1024;                    // Counts Per Revolution
    bool invertA = false;                   // Invert A signal polarity
    bool invertB = false;                   // Invert B signal polarity
    bool useZ = true;                       // Use Z (index) signal
    EncoderMode mode = TIM_ENCODER_MODE;    // Encoder interface mode
    uint32_t watchdogTimeoutMs = 500;       // Watchdog timeout
    int32_t offsetCounts = 0;               // Position offset
    bool wrapAround = true;                 // Enable position wrap-around
};
```

## Usage Example

### Basic Encoder Setup

```cpp
#include "Encoder.hpp"

extern TIM_HandleTypeDef htim3;  // Timer configured in encoder mode

// Create encoder instance
Encoder encoder1;

void setup() {
    // Initialize encoder with ID 1 on TIM3
    encoder1.create(1, &htim3);
    encoder1.init();  // Use default configuration
    
    // Or initialize with custom configuration
    EncoderConfig config;
    config.cpr = 2048;
    config.invertA = true;
    config.useZ = false;
    encoder1.init(config);
}

void loop() {
    // Update encoder (call regularly)
    encoder1.update();
    
    // Read current position and angle
    int32_t position = encoder1.getPosition();
    float angleDeg = encoder1.getAngleDeg();
    float angleRad = encoder1.getAngleRad();
    uint32_t revolutions = encoder1.getRevolutions();
    
    HAL_Delay(10);
}
```

### MAVLink Integration

```cpp
#include "Encoder.hpp"
#include "MAVLinkEncoderController.hpp"

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3, htim4;

MAVLinkEncoderController mavlink_encoder_controller;
Encoder encoder1, encoder2;
uint8_t rx_buffer[1];

void setup() {
    // Initialize encoders
    encoder1.create(1, &htim3);
    encoder1.init();
    
    encoder2.create(2, &htim4);
    encoder2.init();
    
    // Initialize MAVLink controller
    mavlink_encoder_controller.init(&huart2, 1);
    mavlink_encoder_controller.addEncoder(&encoder1);
    mavlink_encoder_controller.addEncoder(&encoder2);
    
    // Enable UART receive interrupt
    HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
}

void loop() {
    // Update MAVLink controller (handles status transmission and encoder updates)
    mavlink_encoder_controller.update();
    HAL_Delay(10);
}

// UART interrupt handler
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        mavlink_encoder_controller.processReceivedByte(rx_buffer[0]);
        HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
    }
}
```

### JSON Configuration

The library supports JSON configuration files for persistent settings:

```json
{
  "encoder": {
    "cpr": 1024,
    "invertA": false,
    "invertB": false,
    "useZ": true,
    "mode": "TIM_ENCODER_MODE",
    "watchdogTimeoutMs": 500,
    "offsetCounts": 0,
    "wrapAround": true
  }
}
```

Load and save configuration:

```cpp
// Load configuration from file
encoder1.loadConfigFromFile("/config/encoder1.json");

// Save current configuration
encoder1.saveConfigToFile("/config/encoder1.json");

// Load configuration for specific encoder ID from multi-encoder config file
encoder1.loadConfigFromFileForId("/config/encoders.json");
```

## MAVLink Protocol Interface

The library implements the following MAVLink commands for remote configuration:

### Configuration Commands

- **MOTOR_CONFIG_SET** (Command ID: 31000)
  - `param1`: Motor/Encoder ID
  - `param2`: Configuration type (1=CPR, 2=invertA, 3=invertB, 4=useZ)
  - `param3`: Value

- **MOTOR_CONFIG_GET** (Command ID: 31001)
  - `param1`: Motor/Encoder ID
  - Response: PARAM_VALUE messages with encoder configuration

- **MOTOR_CONFIG_SAVE** (Command ID: 31002)
  - `param1`: Motor/Encoder ID
  - Saves current configuration to persistent storage

### Status Messages

- **ATTITUDE**: Encoder position and angle data
  - `roll`: Current angle (radians)
  - `pitch`: Normalized position (0.0-1.0)
  - `yaw`: Revolution count

## Hardware Configuration

### STM32 Timer Setup (STM32CubeMX)

1. **Timer Configuration**:
   - Set timer to "Encoder Mode"
   - Configure channels for A/B signals
   - Set appropriate prescaler and period
   - Enable encoder interface

2. **GPIO Configuration**:
   - Configure encoder input pins as alternate function
   - Set appropriate alternate function mapping
   - Configure Z signal as external interrupt (if used)

### Example Timer Configuration

```c
// In main.c (STM32CubeMX generated)
static void MX_TIM3_Init(void) {
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    
    TIM_Encoder_InitTypeDef sEncoderConfig = {0};
    sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC1Filter = 0;
    sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sEncoderConfig.IC2Filter = 0;
    
    HAL_TIM_Encoder_Init(&htim3, &sEncoderConfig);
}
```

## API Reference

### Encoder Class Methods

- `EncoderStatus create(uint8_t id, TIM_HandleTypeDef* htim)`: Initialize encoder with ID and timer
- `EncoderStatus init()`: Initialize with default configuration
- `EncoderStatus init(const EncoderConfig& config)`: Initialize with custom configuration
- `int32_t getPosition()`: Get current position in counts
- `float getAngleRad()`: Get current angle in radians
- `float getAngleDeg()`: Get current angle in degrees
- `uint32_t getRevolutions()`: Get revolution count
- `EncoderStatus reset()`: Reset encoder position to offset
- `EncoderStatus setZeroPosition()`: Set current position as zero
- `void update()`: Update encoder state (call regularly)

## Error Handling

The library provides comprehensive error handling through the `EncoderStatus` enum:

- `OK`: Operation successful
- `NOT_INITIALIZED`: Encoder not initialized
- `TIMER_ERROR`: Timer configuration error
- `OUT_OF_RANGE`: Parameter out of valid range
- `TIMEOUT`: Watchdog timeout occurred
- `CONFIG_ERROR`: Configuration validation failed

## Integration with Existing Project

This library is designed to work alongside the existing ServoMotor library and follows the same architectural patterns. It can be integrated into existing MAVLink-based systems with minimal changes to the main application code.

The library automatically builds as part of the CMake system when placed in the `Lib/Encoder/` directory structure.