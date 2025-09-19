# RIO Mini Firmware

ESP32 Arduino firmware for the RIO Mini desktop companion robot with micro-ROS integration.

## Hardware Requirements

- ESP32 development board
- VL53L0X Time-of-Flight sensor
- NeoPixel LED (WS2812B)
- Servo motor (MG996 or similar)
- MX1508 dual motor driver
- 2x DC motors for differential drive

## Pin Configuration

```cpp
// Motor Control
#define LEFT_MOTOR_IN1  25  // ESP32 GPIO25 -> MX1508 IN1
#define LEFT_MOTOR_IN2  26  // ESP32 GPIO26 -> MX1508 IN2
#define RIGHT_MOTOR_IN3 27  // ESP32 GPIO27 -> MX1508 IN3
#define RIGHT_MOTOR_IN4 13  // ESP32 GPIO13 -> MX1508 IN4

// Sensors and Actuators
#define NEOPIXEL_PIN    16  // ESP32 GPIO16 for NeoPixel data
#define SERVO_PIN       17  // ESP32 GPIO17 for servo control
```

## ROS2 Topics

### Subscriptions

- `/cmd_vel` (geometry_msgs/TwistStamped): Robot movement control
- `/head_pitch` (std_msgs/Float32): Servo angle control (0-180°)
- `/led` (std_msgs/ColorRGBA): LED color and brightness control
- `/pwm_scale` (std_msgs/Float32): Motor PWM scaling (0.0-1.0)

### Publishers

- `/tof` (sensor_msgs/Range): Time-of-Flight sensor data

## Configuration

Update the following variables in the code:

```cpp
// WiFi Configuration
char ssid[] = "<Your WiFi Network Name>";
char psk[] = "<Your WiFi Password>";

// ROS Agent Configuration
char agent_ip[] = "<Your Agent IP>";
size_t agent_port = 8888;
```

## Key Features

### Auto-Reconnection

The firmware implements automatic reconnection to the ROS2 agent:

- **WAITING_AGENT**: Pings agent every 2 seconds
- **AGENT_AVAILABLE**: Normal operation with periodic health checks
- **AGENT_DISCONNECTED**: Clean shutdown and reconnection attempt

### Smooth Servo Movement

- 3-second smooth movement to target positions
- Non-blocking parallel execution
- Configurable movement duration via `SERVO_MOVEMENT_DURATION_MS`

### Edge Detection

- Automatic edge detection using ToF sensor
- LED turns red when edge is detected
- Prevents forward movement when edge is detected

## FreeRTOS Task Architecture

- **Main Task** (Core 1): ROS executor and connection management
- **ToF Task** (Core 0): Sensor data publishing at 20Hz
- **Servo Task** (Core 1): Smooth servo movement control

## Dependencies

- Arduino ESP32 Core
- Adafruit VL53L0X Library
- Adafruit NeoPixel Library
- ESP32Servo Library
- micro-ROS Arduino Library

## Installation

1. Install Arduino IDE with ESP32 support
2. Install required libraries via Library Manager
3. Install micro-ROS Arduino library
4. Configure WiFi and ROS agent settings
5. Upload to ESP32

## Reference Links

- [RIO Mini 3D Model](https://github.com/botforge-robotics/rio_mini_3d_model) - Hardware design and 3D models
- [RIO Mini ROS2](https://github.com/botforge-robotics/rio_mini_ros2) - ROS2 software stack and AI integration

## License

This project is part of the RIO Mini robot ecosystem. See the main ROS2 repository for licensing information.

