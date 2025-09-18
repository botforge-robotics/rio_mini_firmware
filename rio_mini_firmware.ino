#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <std_msgs/msg/color_rgba.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/range.h>
#include <rosidl_runtime_c/string_functions.h>
#include <cstring>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*==========================================
=               ROS Configuration          =
==========================================*/
// WiFi Configuration
char ssid[] = "<Your WiFi Network Name>"; // Your WiFi Network Name
char psk[] = "<Your WiFi Password>";      // Your WiFi Password

// Static IP Configuration
char agent_ip[] = "<Your Agent IP>"; // Desired Static IP as string
size_t agent_port = 8888;            // Communication Port

// ROS Node and Executor
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

// Subscriptions
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__TwistStamped twist_msg;

rcl_subscription_t head_pitch_sub;
std_msgs__msg__Float32 head_pitch_msg;

rcl_subscription_t led_sub;
std_msgs__msg__ColorRGBA led_msg;

rcl_subscription_t pwm_scale_sub;
std_msgs__msg__Float32 pwm_scale_msg;

// Publishers
rcl_publisher_t tof_pub;
sensor_msgs__msg__Range tof_msg;

// FreeRTOS Task handles
TaskHandle_t mainTaskHandle = NULL;
TaskHandle_t tofTaskHandle = NULL;

// Robot parameters for differential drive
const double wheel_distance = 0.08; // Distance between wheels in meters
const double wheel_radius = 0.019;  // Wheel radius in meters

// PWM scaling parameter (0.0 to any higher value)
float scale_motor_pwm = 1.0; // Default to full scale

// ToF sensor last valid reading
float last_valid_distance = 0.0; // Last valid distance reading in meters

// Time synchronization status
bool time_sync_status = false;

// Edge detection variables
bool edge_detection_enabled = true;    // Global flag to enable/disable edge detection
bool edge_detected = false;            // Flag indicating if edge is currently detected
float edge_detection_threshold = 60.0; // Threshold distance in mm for edge detection
bool edge_led_active = false;          // Flag indicating if LED is currently controlled by edge detection

// Motor driver pin definitions
#define LEFT_MOTOR_IN1 25  // ESP32 GPIO25 -> MX1508 IN1
#define LEFT_MOTOR_IN2 26  // ESP32 GPIO26 -> MX1508 IN2
#define RIGHT_MOTOR_IN3 27 // ESP32 GPIO27 -> MX1508 IN3
#define RIGHT_MOTOR_IN4 13 // ESP32 GPIO13 -> MX1508 IN4

// ToF sensor object
Adafruit_VL53L0X tof = Adafruit_VL53L0X();

// NeoPixel LED setup
#define NEOPIXEL_PIN 16 // ESP32 GPIO2 for NeoPixel data
#define NUM_PIXELS 1    // Number of LEDs (just 1)
Adafruit_NeoPixel pixels(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Servo setup
#define SERVO_PIN 17 // ESP32 GPIO17 for servo control
Servo servo;

// Function declarations
void setupMotors();
void setLeftMotor(int power);
void setRightMotor(int power);
void setBothMotors(int leftPower, int rightPower);
void stopMotors();
void setupToF();
float getDistance();
void setupNeoPixel();
void setPixelColor(uint8_t r, uint8_t g, uint8_t b);
void setupServo();
void setServoAngle(int angle);

// ROS Callback functions
void cmd_vel_callback(const void *msgin);
void head_pitch_callback(const void *msgin);
void led_callback(const void *msgin);
void pwm_scale_callback(const void *msgin);

// ROS Helper functions
void setupROS();
void publishToFData();
bool synchronize_time();

// FreeRTOS Task functions
void mainTask(void *pvParameters);
void tofTask(void *pvParameters);

// Error handling macros
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      delay(1000);               \
      ESP.restart();             \
    }                            \
  }

#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

// ROS Callback functions
void cmd_vel_callback(const void *msgin);
void head_pitch_callback(const void *msgin);
void led_callback(const void *msgin);
void pwm_scale_callback(const void *msgin);

// ROS Helper functions
void setupROS();
void publishToFData();
bool synchronize_time();

// FreeRTOS Task functions
void mainTask(void *pvParameters);
void tofTask(void *pvParameters);

// Error handling macros
#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      delay(1000);               \
      ESP.restart();             \
    }                            \
  }

#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

void setup()
{
  // Serial.begin(115200);
  // Serial.println("ESP32 Rio Mini Starting...");
  // Serial.begin(115200);
  // Serial.println("ESP32 Rio Mini Starting...");

  setupMotors();
  setupToF();
  setupNeoPixel();
  setupServo();

  // Initialize ROS
  setupROS();

  // Serial.println("Setup complete. Starting main loop...");
  // Initialize ROS
  setupROS();

  // Serial.println("Setup complete. Starting main loop...");
  delay(2000);
}

void loop()
{
  // Main loop is now handled by FreeRTOS task
  // This loop can be empty or used for other purposes
  vTaskDelay(1000); // Delay 1 second
  // Main loop is now handled by FreeRTOS task
  // This loop can be empty or used for other purposes
  vTaskDelay(1000); // Delay 1 second
}

// Initialize motor pins with 8-bit PWM
// Initialize motor pins with 8-bit PWM
void setupMotors()
{
  // Initialize all pins to 0 (motors stopped)
  analogWrite(LEFT_MOTOR_IN1, 0);
  analogWrite(LEFT_MOTOR_IN2, 0);
  analogWrite(RIGHT_MOTOR_IN3, 0);
  analogWrite(RIGHT_MOTOR_IN4, 0);
  // Initialize all pins to 0 (motors stopped)
  analogWrite(LEFT_MOTOR_IN1, 0);
  analogWrite(LEFT_MOTOR_IN2, 0);
  analogWrite(RIGHT_MOTOR_IN3, 0);
  analogWrite(RIGHT_MOTOR_IN4, 0);

  // Serial.println("Motor pins initialized with 8-bit PWM (5kHz)");
  // Serial.println("Motor pins initialized with 8-bit PWM (5kHz)");
}

// Control Left Motor with power input (-255 to 255) - 8-bit PWM
// Control Left Motor with power input (-255 to 255) - 8-bit PWM
void setLeftMotor(int power)
{
  // Apply dead band: -10 to 10 becomes 0 (4% of 8-bit range)
  if (power >= -10 && power <= 10)
  {
    power = 0;
  }

  // Apply dead band: -10 to 10 becomes 0 (4% of 8-bit range)
  if (power >= -10 && power <= 10)
  {
    power = 0;
  }

  // Constrain power to valid range
  power = constrain(power, -255, 255);

  if (power > 0)
  {
    // Forward direction - both pins use analogWrite for symmetric response
    analogWrite(LEFT_MOTOR_IN1, 0);
    // Forward direction - both pins use analogWrite for symmetric response
    analogWrite(LEFT_MOTOR_IN1, 0);
    analogWrite(LEFT_MOTOR_IN2, power);
    // Serial.print("Left Motor: Forward, Power: ");
    // Serial.println(power);
    // Serial.print("Left Motor: Forward, Power: ");
    // Serial.println(power);
  }
  else if (power < 0)
  {
    // Reverse direction - both pins use analogWrite for symmetric response
    // Reverse direction - both pins use analogWrite for symmetric response
    analogWrite(LEFT_MOTOR_IN1, abs(power));
    analogWrite(LEFT_MOTOR_IN2, 0);
    // Serial.print("Left Motor: Reverse, Power: ");
    // Serial.println(abs(power));
    analogWrite(LEFT_MOTOR_IN2, 0);
    // Serial.print("Left Motor: Reverse, Power: ");
    // Serial.println(abs(power));
  }
  else
  {
    // Stop motor - both pins to 0
    analogWrite(LEFT_MOTOR_IN1, 0);
    analogWrite(LEFT_MOTOR_IN2, 0);
    // Serial.println("Left Motor: Stopped");
    // Stop motor - both pins to 0
    analogWrite(LEFT_MOTOR_IN1, 0);
    analogWrite(LEFT_MOTOR_IN2, 0);
    // Serial.println("Left Motor: Stopped");
  }
}

// Control Right Motor with power input (-255 to 255) - 8-bit PWM
// Control Right Motor with power input (-255 to 255) - 8-bit PWM
void setRightMotor(int power)
{
  // Apply dead band: -10 to 10 becomes 0 (4% of 8-bit range)
  if (power >= -10 && power <= 10)
  {
    power = 0;
  }

  // Apply dead band: -10 to 10 becomes 0 (4% of 8-bit range)
  if (power >= -10 && power <= 10)
  {
    power = 0;
  }

  // Constrain power to valid range
  power = constrain(power, -255, 255);

  if (power > 0)
  {
    // Forward direction - both pins use analogWrite for symmetric response
    analogWrite(RIGHT_MOTOR_IN3, 0);
    // Forward direction - both pins use analogWrite for symmetric response
    analogWrite(RIGHT_MOTOR_IN3, 0);
    analogWrite(RIGHT_MOTOR_IN4, power);
    // Serial.print("Right Motor: Forward, Power: ");
    // Serial.println(power);
  }
  else if (power < 0)
  {
    // Reverse direction - both pins use analogWrite for symmetric response
    analogWrite(RIGHT_MOTOR_IN3, abs(power));
    analogWrite(RIGHT_MOTOR_IN4, 0);
    // Serial.print("Right Motor: Reverse, Power: ");
    // Serial.println(abs(power));
  }
  else
  {
    // Stop motor - both pins to 0
    analogWrite(RIGHT_MOTOR_IN3, 0);
    analogWrite(RIGHT_MOTOR_IN4, 0);
    // Serial.println("Right Motor: Stopped");
  }
}

// Control both motors simultaneously
void setBothMotors(int leftPower, int rightPower)
{
  // Edge detection logic: if edge is detected and enabled, only allow backward movement
  if (edge_detection_enabled && edge_detected)
  {
    // Only allow negative PWM values (backward movement) when edge is detected
    if (leftPower < 0 && rightPower < 0)
    {
      // Both motors moving backward - allow it
      setLeftMotor(leftPower);
      setRightMotor(rightPower);
    }
    else
    {
      // Any forward movement or angular movement - stop the robot
      stopMotors();
    }
    return;
  }

  // Normal operation when edge detection is disabled or no edge detected
  setLeftMotor(leftPower);
  setRightMotor(rightPower);
}

// Stop all motors
void stopMotors()
{
  analogWrite(LEFT_MOTOR_IN1, 0);
  analogWrite(LEFT_MOTOR_IN2, 0);
  analogWrite(RIGHT_MOTOR_IN3, 0);
  analogWrite(RIGHT_MOTOR_IN4, 0);
  // Serial.println("All motors stopped");
}

// Initialize ToF sensor
void setupToF()
{
  Wire.begin();

  if (!tof.begin())
  {
    // Serial.println("Failed to find VL53L0X sensor");
    while (1)
      ;
  }

  // Serial.println("VL53L0X ToF sensor initialized");
}

// Get distance from ToF sensor
float getDistance()
{
  VL53L0X_RangingMeasurementData_t measure;

  tof.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4)
  { // phase failures have incorrect data
    return measure.RangeMilliMeter;
  }
  else
  {
    return -1.0; // Error reading
  }
}

// Initialize NeoPixel LED
void setupNeoPixel()
{
  pixels.begin();
  pixels.clear();
  pixels.show();
  // Serial.println("NeoPixel LED initialized");
}

// Set LED color with RGB values (0-255 each)
void setPixelColor(uint8_t r, uint8_t g, uint8_t b)
{
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
  // Serial.print("LED Color set to RGB(");
  // Serial.print(r);
  // Serial.print(", ");
  // Serial.print(g);
  // Serial.print(", ");
  // Serial.print(b);
  // Serial.println(")");
}

// Initialize servo
void setupServo()
{
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 1000, 2500);
  setServoAngle(180); // Set servo to center position (90 degrees)
}

// Set servo angle (0-180 degrees)
void setServoAngle(int angle)
{
  // Constrain angle to valid range
  angle = constrain(angle, 0, 180);

  servo.write(angle);
  // Serial.print("Servo angle set to: ");
  // Serial.print(angle);
  // Serial.println(" degrees");
}

/*==========================================
=               ROS Implementation         =
==========================================*/

// Setup ROS node, subscriptions, and publishers
void setupROS()
{
  // Set up micro-ROS WiFi transport
  // Serial.println("Setting up WiFi transport...");
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create init_options
  // Serial.println("Initializing ROS support...");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  // Serial.println("Creating ROS node...");
  RCCHECK(rclc_node_init_default(&node, "rio_mini", "", &support));

  // Create subscriptions
  // Serial.println("Creating subscriptions...");

  // Serial.println("Creating cmd_vel subscription...");
  RCCHECK(rclc_subscription_init_default(
      &cmd_vel_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
      "cmd_vel"));

  // Serial.println("Creating head_pitch subscription...");
  RCCHECK(rclc_subscription_init_default(
      &head_pitch_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "head_pitch"));

  // Serial.println("Creating led subscription...");
  RCCHECK(rclc_subscription_init_default(
      &led_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
      "led"));

  // Serial.println("Creating pwm_scale subscription...");
  RCCHECK(rclc_subscription_init_default(
      &pwm_scale_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "pwm_scale"));

  // Create publisher
  // Serial.println("Creating publisher...");
  RCCHECK(rclc_publisher_init_best_effort(
      &tof_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "tof"));

  // Serial.println("Initializing executor...");
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

  // Add subscriptions to executor
  // Serial.println("Adding subscriptions to executor...");
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &head_pitch_sub, &head_pitch_msg, &head_pitch_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &led_sub, &led_msg, &led_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &pwm_scale_sub, &pwm_scale_msg, &pwm_scale_callback, ON_NEW_DATA));

  // Create FreeRTOS tasks
  // Serial.println("Creating FreeRTOS tasks...");

  // Create main task on core 1
  BaseType_t mainTaskResult = xTaskCreatePinnedToCore(
      mainTask,        // Task function
      "Main Task",     // Name of the task
      8192,            // Stack size
      NULL,            // Task input parameter
      1,               // Priority
      &mainTaskHandle, // Task handle
      1                // Core 1 - avoid Core 0 for WiFi communication
  );

  if (mainTaskResult != pdPASS)
  {
    // Serial.println("ERROR: Failed to create Main task!");
    return;
  }

  // Create TOF publisher task on core 0
  BaseType_t tofTaskResult = xTaskCreatePinnedToCore(
      tofTask,        // Task function
      "TOF Task",     // Name of the task
      4096,           // Stack size
      NULL,           // Task input parameter
      2,              // Priority (higher than main task)
      &tofTaskHandle, // Task handle
      0               // Core 0 - dedicated TOF publishing
  );

  if (tofTaskResult != pdPASS)
  {
    // Serial.println("ERROR: Failed to create TOF task!");
    return;
  }

  // Initialize ToF message
  rosidl_runtime_c__String__init(&tof_msg.header.frame_id);
  rosidl_runtime_c__String__assign(&tof_msg.header.frame_id, "tof_link");
  tof_msg.min_range = 0.03; // 3cm minimum range
  tof_msg.max_range = 2.0;  // 2m maximum range

  // Synchronize time
  // Serial.println("Synchronizing time...");
  if (!synchronize_time())
  {
    // Serial.println("Time sync failed!");
  }

  // Initialize memory management
  // Serial.println("Initializing memory management...");

  // Serial.println("ROS setup complete - All systems ready!");
}

// Callback for cmd_vel topic - controls differential drive
void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__TwistStamped *msg = (const geometry_msgs__msg__TwistStamped *)msgin;

  double linear_x = msg->twist.linear.x;
  double angular_z = msg->twist.angular.z;

  // Check if both linear and angular velocities are zero - stop motors
  if (linear_x == 0.0 && angular_z == 0.0)
  {
    stopMotors();
    // Serial.println("Cmd_vel: Both velocities zero - motors stopped");
    return;
  }

  // Convert twist to wheel velocities using differential drive kinematics
  // v_left = v - (L/2) * ω
  // v_right = v + (L/2) * ω
  double left_wheel_speed = linear_x - (wheel_distance / 2.0) * angular_z;
  double right_wheel_speed = linear_x + (wheel_distance / 2.0) * angular_z;

  // Convert wheel speeds to PWM values (-1 to 1 mapped to -255 to 255)
  // Clamp wheel speeds to -1 to 1 range first
  left_wheel_speed = constrain(left_wheel_speed, -1.0, 1.0);
  right_wheel_speed = constrain(right_wheel_speed, -1.0, 1.0);

  // Map from -1 to 1 range to -255 to 255 PWM range (8-bit)
  int left_pwm = (int)(left_wheel_speed * 255.0);
  int right_pwm = (int)(right_wheel_speed * 255.0);

  // Apply PWM scaling factor
  left_pwm = (int)(left_pwm * scale_motor_pwm);
  right_pwm = (int)(right_pwm * scale_motor_pwm);

  // Control motors (dead band applied in setLeftMotor/setRightMotor)
  setBothMotors(left_pwm, right_pwm);

  // Serial.print("Cmd_vel: linear=");
  // Serial.print(linear_x);
  // Serial.print(", angular=");
  // Serial.print(angular_z);
  // Serial.print(" -> Left PWM=");
  // Serial.print(left_pwm);
  // Serial.print(", Right PWM=");
  // Serial.println(right_pwm);
}

// Callback for head_pitch topic - controls servo angle
void head_pitch_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

  float pitch_angle = msg->data;

  setServoAngle(pitch_angle);

  // Serial.print("Head pitch: ");
  // Serial.print(pitch_angle);
  // Serial.print(" -> Servo angle: ");
  // Serial.println(servo_angle);
}

// Callback for LED topic - controls LED color
void led_callback(const void *msgin)
{
  const std_msgs__msg__ColorRGBA *msg = (const std_msgs__msg__ColorRGBA *)msgin;

  // Only allow LED control if edge detection is not controlling the LED
  if (edge_led_active)
  {
    // Serial.println("LED control blocked - edge detection is active");
    return;
  }

  uint8_t r = (uint8_t)constrain(msg->r * 255, 0, 255);
  uint8_t g = (uint8_t)constrain(msg->g * 255, 0, 255);
  uint8_t b = (uint8_t)constrain(msg->b * 255, 0, 255);
  uint8_t brightness = (uint8_t)constrain(msg->a * 255, 0, 255);

  // Set LED color and brightness
  pixels.setBrightness(brightness);
  setPixelColor(r, g, b);

  // Serial.print("LED: R=");
  // Serial.print(r);
  // Serial.print(", G=");
  // Serial.print(g);
  // Serial.print(", B=");
  // Serial.print(b);
  // Serial.print(", Brightness=");
  // Serial.println(brightness);
}

// Publish ToF sensor data with proper error handling
void publishToFData()
{

  float distance = getDistance();

  // Edge detection logic
  if (edge_detection_enabled)
  {
    if (distance > 0) // Valid reading
    {
      // Check if distance exceeds threshold (edge detected)
      if (distance > edge_detection_threshold)
      {
        if (!edge_detected) // Only act when state changes
        {
          // Serial.print("Edge detected! Distance: ");
          // Serial.print(distance);
          // Serial.print(" mm (threshold: ");
          // Serial.print(edge_detection_threshold);
          // Serial.println(" mm)");

          // Turn LED red when edge is detected
          setPixelColor(255, 0, 0); // Red color
          edge_led_active = true;
        }
        edge_detected = true;
      }
      else
      {
        if (edge_detected) // Only act when state changes
        {
          // Serial.print("Edge cleared! Distance: ");
          // Serial.print(distance);
          // Serial.println(" mm");

          // Turn off LED when edge is no longer detected
          setPixelColor(0, 0, 0); // Turn off LED
          edge_led_active = false;
        }
        // Edge no longer detected, reset flag
        edge_detected = false;
      }
    }
    // If invalid reading, keep current edge_detected state
  }

  // Always publish, even for invalid readings
  if (distance > 0) // Valid reading
  {
    last_valid_distance = distance / 1000.0; // Convert mm to meters and store
    tof_msg.range = last_valid_distance;
  }
  else
  {
    // Invalid reading - use last valid reading
    tof_msg.range = last_valid_distance;
  }

  // Set timestamp
  struct timespec tv = {0};
  clock_gettime(0, &tv);
  tof_msg.header.stamp.nanosec = tv.tv_nsec;
  tof_msg.header.stamp.sec = tv.tv_sec;

  // Publish with error handling
  rcl_ret_t ret = rcl_publish(&tof_pub, &tof_msg, NULL);
  if (ret != RCL_RET_OK)
  {
    // Serial.print("ToF publish failed: ");
    // Serial.println(ret);
    return; // Exit early on failure
  }

  // Serial.print("ToF distance: ");
  // Serial.print(distance);
  // Serial.print(" mm, Published: ");
  // Serial.print(tof_msg.range * 1000.0);
  // Serial.println(" mm");
}

// Synchronize time with ROS agent
bool synchronize_time()
{
  static bool first_sync = true;
  if (first_sync)
  {
    // Serial.println("Synchronizing time with agent...");
    first_sync = false;
  }

  rmw_ret_t ret = rmw_uros_sync_session(1000); // Increase timeout to 1000ms
  if (ret != RMW_RET_OK)
  {
    time_sync_status = false;
    return false;
  }

  time_sync_status = true;
  return true;
}

// Callback for pwm_scale topic - controls PWM scaling
void pwm_scale_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;

  // Constrain the value to be >= 0.0 (allow any higher value)
  scale_motor_pwm = constrain(msg->data, 0.0, 100.0);

  // Serial.print("PWM scale updated to: ");
  // Serial.println(scale_motor_pwm);
}

// FreeRTOS Main Task - handles ROS executor only
void mainTask(void *pvParameters)
{
  // Serial.println("Main Task started");

  static unsigned long last_sync_time = 0;
  const unsigned long sync_interval = 10000; // Sync every 10 seconds

  while (true)
  {
    // Periodic time synchronization
    if (millis() - last_sync_time > sync_interval)
    {
      synchronize_time();
      last_sync_time = millis();
    }

    // Only proceed with ROS operations if time is synchronized
    if (time_sync_status)
    {
      // Handle ROS executor
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }

    vTaskDelay(1); // Small delay like in example
  }
}

// FreeRTOS TOF Task - dedicated ToF sensor publishing on core 0
void tofTask(void *pvParameters)
{
  // Serial.println("TOF Task started on Core 0");

  static unsigned long last_tof_publish = 0;
  const unsigned long tof_publish_interval = 50; // 20Hz publishing rate

  while (true)
  {
    // Only publish if time is synchronized
    if (time_sync_status)
    {
      // Publish ToF data at 20Hz using millis() timing
      if (millis() - last_tof_publish > tof_publish_interval)
      {
        publishToFData();
        last_tof_publish = millis();
      }
    }

    vTaskDelay(1); // Small delay
  }
}

