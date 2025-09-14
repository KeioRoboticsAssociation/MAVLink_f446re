# RoboMaster Motor Control Library

STM32 C++ library for controlling RoboMaster motors (M2006, M3508, etc.) via CAN bus with comprehensive safety features and flexible control modes.

## Quick Start

```cpp
#include "RoboMasterMotor.hpp"
#include "RoboMasterCANManager.hpp"

RoboMasterCANManager can_manager;
RoboMasterMotor motor1;

void setup() {
    can_manager.init(&hcan1);
    can_manager.start();

    motor1.create(1, &can_manager);
    motor1.init();
    motor1.setEnabled(true);
}

void loop() {
    can_manager.update();
    motor1.setVelocityRPS(10.0f);  // 10 RPS target
}
```

## Key Features

- **Individual Motor Control**: Each motor is an independent object with separate configuration
- **Multiple Control Modes**: Current, velocity, and position control with PID regulation
- **Safety Systems**: Watchdog timeout, thermal protection, current limiting, position limits
- **Advanced Control**: Multi-turn tracking, cascade PID, anti-windup, rate limiting
- **Monitoring**: Real-time statistics, error detection, temperature monitoring

## Architecture

### `RoboMasterMotor`
Individual motor control class with motor-specific state, control loops, and safety monitoring.

### `RoboMasterCANManager`
Centralized CAN communication manager handling bus initialization, message routing, and error diagnostics.

## Control Modes

### Velocity Control
```cpp
motor1.setControlMode(RoboMasterControlMode::VELOCITY);
motor1.setVelocityRPS(25.0f);
```

### Position Control
```cpp
motor1.setControlMode(RoboMasterControlMode::POSITION);
motor1.setPositionRad(2.0f * M_PI);  // 2 full rotations
```

### Current Control
```cpp
motor1.setControlMode(RoboMasterControlMode::CURRENT);
motor1.setCurrent(5000);  // 5000mA torque
```

## Advanced Configuration

```cpp
RoboMasterConfig config;
config.maxVelocityRPS = 50.0f;
config.maxCurrent = 8000;
config.positionKp = 20.0f;
config.velocityKp = 40.0f;
config.velocityKi = 0.3f;
config.watchdogTimeoutMs = 500;
config.failSafeBehavior = FailSafeBehavior::BRAKE;

// Optional position limits
config.positionLimitsEnabled = true;
config.minPositionRad = -5.0f * M_PI;
config.maxPositionRad = 5.0f * M_PI;

motor1.create(1, &can_manager);
motor1.init(config);
```

## Status Monitoring

```cpp
// Real-time measurements
float position = motor1.getCurrentPosition();
float velocity = motor1.getCurrentVelocity();
int16_t current = motor1.getCurrentMilliamps();
uint8_t temperature = motor1.getTemperature();

// Error handling
RoboMasterStatus status = motor1.getStatus();
if (status == RoboMasterStatus::TIMEOUT) {
    motor1.resetWatchdog();
} else if (status == RoboMasterStatus::OVERHEAT) {
    motor1.setEnabled(false);
}

// Statistics
const RoboMasterState& state = motor1.getState();
printf("Saturations: %u, Timeouts: %u\n",
       state.saturationCount, state.timeoutCount);
```

## CAN Integration

Required CAN interrupt handler:

```cpp
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1) {
        can_manager.handleCANReceive();
    }
}
```

## Configuration Parameters

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

## Error States

- `RoboMasterStatus::OK` - Normal operation
- `RoboMasterStatus::TIMEOUT` - Communication timeout
- `RoboMasterStatus::OVERHEAT` - Temperature exceeded
- `RoboMasterStatus::OVERCURRENT` - Current limit exceeded
- `RoboMasterStatus::CONFIG_ERROR` - Invalid configuration
- `RoboMasterStatus::CAN_ERROR` - CAN communication failure

## Hardware Requirements

- STM32F4xx or compatible with CAN peripheral
- CAN bus configured for 1Mbps
- RoboMaster motors with proper termination
- Update rate ≥100Hz recommended

## Migration from Legacy API

**Old:**
```cpp
RoboMasterController controller(&hcan1);
controller.setTargetSpeed(0, 10.0f);
controller.timer_callback();
```

**New:**
```cpp
RoboMasterCANManager can_manager;
RoboMasterMotor motor1;
can_manager.init(&hcan1);
motor1.create(1, &can_manager);
motor1.setVelocityRPS(10.0f);
can_manager.update();
```

## CAN Protocol Reference

### Command IDs

| ID | Description | Motors |
|----|-------------|---------|
| `0x200` | Current command for motors 1-4 | M1, M2, M3, M4 |
| `0x1FF` | Current command for motors 5-8 | M5, M6, M7, M8 |
| `0x1FE` | Extended command for motors 1-4 | M1, M2, M3, M4 |
| `0x1FD` | Extended command for motors 5-8 | M5, M6, M7, M8 |

### Feedback IDs

| Motor | Feedback ID | Description |
|-------|-------------|-------------|
| Motor 1 | `0x201` | Position, velocity, current, temperature |
| Motor 2 | `0x202` | Position, velocity, current, temperature |
| Motor 3 | `0x203` | Position, velocity, current, temperature |
| Motor 4 | `0x204` | Position, velocity, current, temperature |
| Motor 5 | `0x205` | Position, velocity, current, temperature |
| Motor 6 | `0x206` | Position, velocity, current, temperature |
| Motor 7 | `0x207` | Position, velocity, current, temperature |
| Motor 8 | `0x208` | Position, velocity, current, temperature |

### Command Frame Format

**Current Command Frame (8 bytes):**
```
Byte 0-1: Motor 1 current (big-endian, signed 16-bit)
Byte 2-3: Motor 2 current (big-endian, signed 16-bit)
Byte 4-5: Motor 3 current (big-endian, signed 16-bit)
Byte 6-7: Motor 4 current (big-endian, signed 16-bit)
```

**Manual Command Example:**
```cpp
// Send 5000mA to motor 1, 0mA to others on ID 0x200
uint8_t cmd_data[8] = {
    0x13, 0x88,  // Motor 1: 5000mA (0x1388)
    0x00, 0x00,  // Motor 2: 0mA
    0x00, 0x00,  // Motor 3: 0mA
    0x00, 0x00   // Motor 4: 0mA
};
```

### Feedback Frame Format

**Motor Feedback Frame (8 bytes):**
```
Byte 0-1: Position (big-endian, 0-8191 encoder counts, 13-bit)
Byte 2-3: Velocity (big-endian, signed 16-bit RPM)
Byte 4-5: Current (big-endian, signed 16-bit mA)
Byte 6:   Temperature (unsigned 8-bit Celsius)
Byte 7:   Reserved/unused
```

**Manual Parsing Example:**
```cpp
void parseMotorFeedback(const uint8_t* data) {
    int16_t raw_position = (data[0] << 8) | data[1];  // 0-8191
    int16_t raw_velocity = (data[2] << 8) | data[3];  // RPM
    int16_t current_ma   = (data[4] << 8) | data[5];  // mA
    uint8_t temperature  = data[6];                   // °C

    // Convert to engineering units
    float position_rad = (float)raw_position * (2.0f * M_PI / 8192.0f);
    float velocity_rps = (float)raw_velocity / 60.0f;
}
```

### Raw CAN Integration

For direct CAN usage without the library:

```cpp
// Send current commands
CAN_TxHeaderTypeDef tx_header;
tx_header.StdId = 0x200;  // Motors 1-4
tx_header.IDE = CAN_ID_STD;
tx_header.RTR = CAN_RTR_DATA;
tx_header.DLC = 8;

uint8_t cmd_data[8] = {0x13, 0x88, 0, 0, 0, 0, 0, 0};  // 5A to M1
uint32_t tx_mailbox;
HAL_CAN_AddTxMessage(&hcan1, &tx_header, cmd_data, &tx_mailbox);

// Receive feedback in CAN interrupt
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x208) {
            uint8_t motor_id = rx_header.StdId - 0x200;  // 1-8
            // Process feedback data...
        }
    }
}
```

## ROS/MAVLink Integration

The library supports ROS integration via MAVLink protocol for parameter management and motor control.

### MAVLink Message IDs for ROS Commands

#### Standard MAVLink Messages

| Message ID | Message Type | Purpose |
|------------|--------------|---------|
| `20` | `PARAM_REQUEST_READ` | Read specific parameter |
| `21` | `PARAM_REQUEST_LIST` | Request all parameters |
| `23` | `PARAM_SET` | Set parameter value |
| `69` | `MANUAL_CONTROL` | Direct motor control |
| `76` | `COMMAND_LONG` | System commands (save, emergency stop) |
| `66` | `REQUEST_DATA_STREAM` | Enable/disable telemetry |

#### Custom RoboMaster Messages

| Message ID | Message Type | Purpose |
|------------|--------------|---------|
| `180` | `ROBOMASTER_MOTOR_CONTROL` | Advanced motor control with mode selection |
| `181` | `ROBOMASTER_MOTOR_STATUS` | Individual motor status and diagnostics |
| `182` | `ROBOMASTER_MOTOR_CONFIG` | Motor configuration read/write |
| `183` | `ROBOMASTER_TELEMETRY` | High-rate telemetry data |

### Motor Parameter Names

Parameters use format `M{ID}_{PARAM}` where `{ID}` is motor ID (1-8):

| Parameter | Description | Range | Example |
|-----------|-------------|-------|---------|
| `M1_POS_KP` | Position P gain | 0-1000 | `M1_POS_KP`, `M2_POS_KP` |
| `M1_POS_KI` | Position I gain | 0-100 | `M1_POS_KI`, `M3_POS_KI` |
| `M1_POS_KD` | Position D gain | 0-100 | `M1_POS_KD`, `M4_POS_KD` |
| `M1_VEL_KP` | Velocity P gain | 0-1000 | `M1_VEL_KP`, `M2_VEL_KP` |
| `M1_VEL_KI` | Velocity I gain | 0-100 | `M1_VEL_KI`, `M3_VEL_KI` |
| `M1_VEL_KD` | Velocity D gain | 0-100 | `M1_VEL_KD`, `M4_VEL_KD` |
| `M1_MAX_VEL` | Max velocity (RPS) | 1-200 | `M1_MAX_VEL`, `M2_MAX_VEL` |
| `M1_MAX_CUR` | Max current (mA) | 100-20000 | `M1_MAX_CUR`, `M3_MAX_CUR` |
| `M1_MAX_TEMP` | Max temperature (°C) | 40-100 | `M1_MAX_TEMP`, `M4_MAX_TEMP` |
| `M1_TIMEOUT` | Watchdog timeout (ms) | 50-5000 | `M1_TIMEOUT`, `M2_TIMEOUT` |

### ROS Command Examples

#### 1. Read Parameter
```bash
# Read Motor 1 velocity P gain
rostopic pub /mavlink/to mavros_msgs/Mavlink '{
  header: {stamp: now, frame_id: ""},
  msgid: 20,
  payload64: [1, 0, "M1_VEL_KP", 0]
}'
```

#### 2. Set Parameter
```bash
# Set Motor 1 velocity P gain to 50.0
rostopic pub /mavlink/to mavros_msgs/Mavlink '{
  header: {stamp: now, frame_id: ""},
  msgid: 23,
  payload64: [1, 0, "M1_VEL_KP", 50.0]
}'
```

#### 3. Motor Control (Manual Control)
```bash
# Control motors 1-4 via MANUAL_CONTROL
# x: Motor 1 command (-1000 to 1000)
# y: Motor 2 command (-1000 to 1000)
# z: Motor 3 command (-1000 to 1000)
# r: Motor 4 command (-1000 to 1000)
rostopic pub /mavlink/to mavros_msgs/Mavlink '{
  header: {stamp: now, frame_id: ""},
  msgid: 69,
  payload64: [500, -200, 0, 800, 0, 1]
}'
```

#### 4. Emergency Stop
```bash
# Emergency stop all motors
rostopic pub /mavlink/to mavros_msgs/Mavlink '{
  header: {stamp: now, frame_id: ""},
  msgid: 76,
  payload64: [400, 0, 0, 0, 0, 0, 0, 0, 1, 0]
}'
```

#### 5. Save Parameters to Flash
```bash
# Save current parameters to non-volatile storage
rostopic pub /mavlink/to mavros_msgs/Mavlink '{
  header: {stamp: now, frame_id: ""},
  msgid: 76,
  payload64: [245, 1, 0, 0, 0, 0, 0, 0, 1, 0]
}'
```

#### 6. Request All Parameters
```bash
# Get list of all available parameters
rostopic pub /mavlink/to mavros_msgs/Mavlink '{
  header: {stamp: now, frame_id: ""},
  msgid: 21,
  payload64: [1, 0, 0, 0]
}'
```

### Custom Message Examples

#### 7. Advanced Motor Control (Message ID 180)
```bash
# Control motor with specific mode and value
# Format: [motor_id, control_mode, value, enable]
# control_mode: 0=Current, 1=Velocity, 2=Position
rostopic pub /mavlink/to mavros_msgs/Mavlink '{
  header: {stamp: now, frame_id: ""},
  msgid: 180,
  payload64: [1, 2, 3.14159, 1]  # Motor 1, Position mode, π radians, Enable
}'
```

#### 8. Request Motor Status (Message ID 181)
```bash
# Request detailed status for specific motor
rostopic pub /mavlink/to mavros_msgs/Mavlink '{
  header: {stamp: now, frame_id: ""},
  msgid: 181,
  payload64: [1, 0, 0, 0]  # Motor ID 1
}'
```

#### 9. Motor Configuration (Message ID 182)
```bash
# Read motor configuration
rostopic pub /mavlink/to mavros_msgs/Mavlink '{
  header: {stamp: now, frame_id: ""},
  msgid: 182,
  payload64: [1, 0, 0, 0]  # Motor ID 1, Read mode
}'

# Write motor configuration
rostopic pub /mavlink/to mavros_msgs/Mavlink '{
  header: {stamp: now, frame_id: ""},
  msgid: 182,
  payload64: [1, 1, 50.0, 0.5]  # Motor ID 1, Write mode, Max velocity, PID gain
}'
```

#### 10. Enable High-Rate Telemetry (Message ID 183)
```bash
# Enable 50Hz telemetry stream
rostopic pub /mavlink/to mavros_msgs/Mavlink '{
  header: {stamp: now, frame_id: ""},
  msgid: 66,
  payload64: [183, 50, 1, 1]  # Stream ID 183, 50Hz rate, Enable, Target system 1
}'
```

### Python ROS Integration Example

```python
#!/usr/bin/env python3
import rospy
from mavros_msgs.srv import ParamGet, ParamSet
from mavros_msgs.msg import Mavlink
import numpy as np

class RoboMasterController:
    def __init__(self):
        rospy.init_node('robomaster_controller')
        self.param_get = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        self.param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        self.mavlink_pub = rospy.Publisher('/mavlink/to', Mavlink, queue_size=10)

    def read_motor_params(self, motor_id):
        """Read all parameters for a specific motor"""
        params = {}
        param_names = ['POS_KP', 'POS_KI', 'VEL_KP', 'VEL_KI', 'MAX_VEL', 'MAX_CUR']

        for param in param_names:
            param_name = f'M{motor_id}_{param}'
            try:
                response = self.param_get(param_name)
                params[param] = response.value.real
                rospy.loginfo(f"{param_name}: {response.value.real}")
            except Exception as e:
                rospy.logwarn(f"Failed to read {param_name}: {e}")

        return params

    def set_motor_pid(self, motor_id, pos_kp=None, pos_ki=None, vel_kp=None, vel_ki=None):
        """Set PID gains for a motor"""
        if pos_kp is not None:
            self.param_set(f'M{motor_id}_POS_KP', pos_kp)
            rospy.loginfo(f"Set Motor {motor_id} Position Kp: {pos_kp}")

        if pos_ki is not None:
            self.param_set(f'M{motor_id}_POS_KI', pos_ki)
            rospy.loginfo(f"Set Motor {motor_id} Position Ki: {pos_ki}")

        if vel_kp is not None:
            self.param_set(f'M{motor_id}_VEL_KP', vel_kp)
            rospy.loginfo(f"Set Motor {motor_id} Velocity Kp: {vel_kp}")

        if vel_ki is not None:
            self.param_set(f'M{motor_id}_VEL_KI', vel_ki)
            rospy.loginfo(f"Set Motor {motor_id} Velocity Ki: {vel_ki}")

    def control_motor(self, motor1_cmd=0, motor2_cmd=0, motor3_cmd=0, motor4_cmd=0):
        """Send motor control commands (-1000 to 1000)"""
        msg = Mavlink()
        msg.msgid = 69  # MANUAL_CONTROL
        msg.payload64 = [motor1_cmd, motor2_cmd, motor3_cmd, motor4_cmd, 0, 1]
        self.mavlink_pub.publish(msg)

    def emergency_stop(self):
        """Emergency stop all motors"""
        msg = Mavlink()
        msg.msgid = 76  # COMMAND_LONG
        msg.payload64 = [400, 0, 0, 0, 0, 0, 0, 0, 1, 0]  # MAV_CMD_COMPONENT_ARM_DISARM
        self.mavlink_pub.publish(msg)
        rospy.logwarn("Emergency stop commanded!")

if __name__ == '__main__':
    controller = RoboMasterController()

    # Example usage
    rospy.sleep(1)  # Wait for connections

    # Read current parameters
    motor1_params = controller.read_motor_params(1)

    # Adjust PID gains
    controller.set_motor_pid(1, pos_kp=25.0, vel_kp=45.0)

    # Control motors
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Send sinusoidal commands
        t = rospy.Time.now().to_sec()
        cmd1 = int(500 * np.sin(t))
        cmd2 = int(300 * np.cos(t))

        controller.control_motor(cmd1, cmd2, 0, 0)
        rate.sleep()
```

### System ID and Component Configuration

- **System ID**: `1` (configurable via `mavlink_controller.init()`)
- **Component ID**: `MAV_COMP_ID_AUTOPILOT1` (1)
- **Target System**: Use `1` in all ROS commands
- **UART**: Typically UART2 for MAVLink communication
- **Baud Rate**: 115200 (matches MAVLink standard)

### Telemetry Data

The system automatically sends telemetry at 10Hz containing:
- Motor positions, velocities, currents, temperatures
- System status and error states
- Parameter change acknowledgments
- Command execution results

See `Examples/ros_integration_example.cpp` for complete implementation.