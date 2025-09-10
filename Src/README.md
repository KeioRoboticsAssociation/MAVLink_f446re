# STM32 MAVLink Interface Documentation

This document provides comprehensive information about the STM32F446RETx MAVLink implementation for creating a corresponding ROS interface package.

## System Overview

### Hardware Platform
- **Microcontroller**: STM32F446RETx
- **Clock Speed**: 84MHz (HSI PLL configuration)
- **Communication**: UART2 at 115200 baud
- **Pins**: PA2/PA3 (UART2 TX/RX)

### MAVLink Configuration
- **System ID**: 1
- **Component ID**: MAV_COMP_ID_AUTOPILOT1
- **Protocol Version**: MAVLink v2
- **Library**: c_library_v2/common

## Communication Protocol

### Message Types Supported

#### Outgoing Messages (STM32 → ROS)
1. **HEARTBEAT** (`MAVLINK_MSG_ID_HEARTBEAT`)
   - Frequency: 1Hz
   - Type: MAV_TYPE_QUADROTOR
   - Autopilot: MAV_AUTOPILOT_GENERIC
   - Base mode: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
   - System status: MAV_STATE_ACTIVE

2. **SERVO_OUTPUT_RAW** (`MAVLINK_MSG_ID_SERVO_OUTPUT_RAW`)
   - Frequency: 10Hz
   - Ports 1-3: Servo PWM values (1000-2000 microseconds)
   - Timestamp: HAL_GetTick()

3. **AUTOPILOT_VERSION** (`MAVLINK_MSG_ID_AUTOPILOT_VERSION`)
   - Sent on request
   - Flight SW version: 0x01000000
   - Board version: 0x01000000

#### Incoming Messages (ROS → STM32)
1. **MANUAL_CONTROL** (`MAVLINK_MSG_ID_MANUAL_CONTROL`)
   - Controls servo positions via joystick-like interface
   - X, Y, Z, R values mapped to servo angles
   - Target system: Must match system ID

2. **RC_CHANNELS_OVERRIDE** (`MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE`)
   - Direct PWM control of servos
   - Channels 1-3 mapped to servo motors
   - Values: 1000-2000 microseconds (65535 = ignore)

## Servo Control System

### Hardware Configuration
- **Servo 1**: Timer 2, Channel 1
- **Servo 2**: Timer 2, Channel 2  
- **Servo 3**: Timer 12, Channel 1

### Servo Parameters
```cpp
struct ServoConfig {
    float angleMinDeg = -90.0f;         // Minimum angle
    float angleMaxDeg = 90.0f;          // Maximum angle
    uint16_t pulseMinUs = 1000;         // Minimum pulse width
    uint16_t pulseMaxUs = 2000;         // Maximum pulse width
    uint16_t pulseNeutralUs = 1500;     // Neutral position pulse
    bool directionInverted = false;     // Direction inversion
    float offsetDeg = 0.0f;             // Angle offset
    float maxVelocityDegPerS = 180.0f;  // Maximum velocity
    float maxAccelerationDegPerS2 = 360.0f; // Maximum acceleration
    uint32_t watchdogTimeoutMs = 500;   // Watchdog timeout
    FailSafeBehavior failSafeBehavior = FailSafeBehavior::NEUTRAL_POSITION;
    float startupAngleDeg = 0.0f;       // Startup position
    bool startDisabled = false;         // Start in disabled state
};
```

### Servo State Information
```cpp
struct ServoState {
    float currentAngleDeg;      // Current angle position
    float targetAngleDeg;       // Target angle position
    uint16_t currentPulseUs;    // Current PWM pulse width
    bool enabled;               // Servo enabled state
    ServoStatus status;         // Status (OK, ERROR, etc.)
    uint32_t lastCommandTime;   // Last command timestamp
    uint32_t saturationCount;   // Saturation event counter
    uint32_t timeoutCount;      // Timeout event counter
    uint32_t errorCount;        // Error event counter
};
```

## Data Flow Architecture

### STM32 Main Loop (10ms cycle)
1. **Update MAVLink Controller**
   - Process incoming bytes from UART
   - Parse MAVLink messages
   - Handle servo commands
   - Send telemetry data

2. **Servo Updates**
   - Apply rate limiting and acceleration constraints
   - Update PWM outputs
   - Monitor watchdog timeouts
   - Handle fail-safe conditions

### UART Interrupt Handler
```cpp
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        mavlink_controller.processReceivedByte(rx_buffer[0]);
        HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
    }
}
```

## ROS Interface Requirements

### Recommended ROS Topics

#### Publishers (STM32 → ROS)
- `/stm32_mavlink/heartbeat` (mavros_msgs/Heartbeat)
- `/stm32_mavlink/servo_output_raw` (mavros_msgs/ServoOutputRaw)
- `/stm32_mavlink/servo_states` (custom message with ServoState array)
- `/stm32_mavlink/system_status` (diagnostic_msgs/DiagnosticArray)

#### Subscribers (ROS → STM32)
- `/stm32_mavlink/manual_control` (mavros_msgs/ManualControl)
- `/stm32_mavlink/rc_override` (mavros_msgs/RCOverride)
- `/stm32_mavlink/servo_commands` (custom message for direct servo control)

#### Services
- `/stm32_mavlink/set_servo_config` (configure servo parameters)
- `/stm32_mavlink/get_servo_status` (retrieve servo diagnostics)
- `/stm32_mavlink/enable_servo` (enable/disable individual servos)

### Custom ROS Messages

#### ServoState.msg
```
uint8 servo_id
float32 current_angle_deg
float32 target_angle_deg
uint16 current_pulse_us
bool enabled
uint8 status
uint32 last_command_time
uint32 saturation_count
uint32 timeout_count
uint32 error_count
```

#### ServoCommand.msg
```
uint8 servo_id
float32 angle_deg
uint16 pulse_us
bool use_angle    # true = use angle_deg, false = use pulse_us
```

### MAVLink Bridge Configuration
- **Connection**: Serial port (e.g., /dev/ttyUSB0, /dev/ttyACM0)
- **Baud Rate**: 115200
- **Frame ID**: "stm32_mavlink"
- **System ID**: 1 (must match STM32)
- **Component ID**: Should use MAV_COMP_ID_ONBOARD_COMPUTER

## Error Handling and Diagnostics

### Watchdog System
- Each servo has individual watchdog timer (500ms default)
- On timeout: Apply fail-safe behavior (hold/neutral/disable)
- Timeout events logged in servo state

### Status Codes
```cpp
enum class ServoStatus {
    OK = 0,
    NOT_INITIALIZED,
    TIMER_ERROR,
    OUT_OF_RANGE,
    TIMEOUT,
    CONFIG_ERROR
};
```

### Diagnostic Information
- Communication link status
- Servo health and error counts
- MAVLink message statistics
- System uptime and reset counters

## Build and Flash Information

### CMake Build Commands
```bash
cmake --preset Debug
cmake --build --preset Debug
```

### Flash Command (STM32CubeProgrammer)
```bash
STM32_Programmer_CLI --connect port=swd --download build/Debug/mavlink_test --hardRst --rst --start
```

### VSCode Tasks
- `CMake: clean rebuild`
- `CubeProg: Flash project (SWD)`
- `Build + Flash`

## Integration Notes

1. **Coordinate Frame**: STM32 uses degrees for angles, consider ROS standard (radians)
2. **Timing**: STM32 timestamps use HAL_GetTick() (milliseconds since boot)
3. **Safety**: Implement emergency stop and fail-safe mechanisms in ROS layer
4. **Scaling**: PWM values are 1000-2000μs, standard servo range
5. **Latency**: UART communication introduces ~1-2ms latency at 115200 baud

## Testing and Validation

### STM32 Side Verification
- Monitor UART output with serial terminal
- Verify servo PWM signals with oscilloscope
- Test fail-safe behavior by disconnecting communication

### ROS Side Testing
- Use `rostopic echo` to monitor telemetry
- Publish test commands to verify servo response
- Implement automated test sequences for validation

This documentation should provide sufficient detail for generating a comprehensive ROS package that interfaces with this STM32 MAVLink implementation.