#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>

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

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 Motor Test Starting...");

  setupMotors();
  setupToF();
  setupNeoPixel();
  setupServo();

  Serial.println("Setup complete. Starting main loop...");
  delay(2000);
}

void loop()
{
}

// Initialize motor pins
void setupMotors()
{
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);

  // Initialize all pins to LOW (motors stopped)
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);

  Serial.println("Motor pins initialized");
}

// Control Left Motor with power input (-255 to 255)
void setLeftMotor(int power)
{
  // Constrain power to valid range
  power = constrain(power, -255, 255);

  if (power > 0)
  {
    // Forward direction (inverted)
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    analogWrite(LEFT_MOTOR_IN2, power);
    Serial.print("Left Motor: Forward, Power: ");
    Serial.println(power);
  }
  else if (power < 0)
  {
    // Reverse direction (inverted)
    analogWrite(LEFT_MOTOR_IN1, abs(power));
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    Serial.print("Left Motor: Reverse, Power: ");
    Serial.println(abs(power));
  }
  else
  {
    // Stop motor
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    Serial.println("Left Motor: Stopped");
  }
}

// Control Right Motor with power input (-255 to 255)
void setRightMotor(int power)
{
  // Constrain power to valid range
  power = constrain(power, -255, 255);

  if (power > 0)
  {
    // Forward direction (inverted)
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    analogWrite(RIGHT_MOTOR_IN4, power);
    Serial.print("Right Motor: Forward, Power: ");
    Serial.println(power);
  }
  else if (power < 0)
  {
    // Reverse direction (inverted)
    analogWrite(RIGHT_MOTOR_IN3, abs(power));
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
    Serial.print("Right Motor: Reverse, Power: ");
    Serial.println(abs(power));
  }
  else
  {
    // Stop motor
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
    Serial.println("Right Motor: Stopped");
  }
}

// Control both motors simultaneously
void setBothMotors(int leftPower, int rightPower)
{
  setLeftMotor(leftPower);
  setRightMotor(rightPower);
}

// Stop all motors
void stopMotors()
{
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  Serial.println("All motors stopped");
}

// Initialize ToF sensor
void setupToF()
{
  Wire.begin();

  if (!tof.begin())
  {
    Serial.println("Failed to find VL53L0X sensor");
    while (1)
      ;
  }

  Serial.println("VL53L0X ToF sensor initialized");
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
  Serial.println("NeoPixel LED initialized");
}

// Set LED color with RGB values (0-255 each)
void setPixelColor(uint8_t r, uint8_t g, uint8_t b)
{
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
  Serial.print("LED Color set to RGB(");
  Serial.print(r);
  Serial.print(", ");
  Serial.print(g);
  Serial.print(", ");
  Serial.print(b);
  Serial.println(")");
}

// Initialize servo
void setupServo()
{
  servo.attach(SERVO_PIN);
  servo.write(90); // Start at center position
  Serial.println("Servo initialized at center position (90 degrees)");
}

// Set servo angle (0-180 degrees)
void setServoAngle(int angle)
{
  // Constrain angle to valid range
  angle = constrain(angle, 0, 180);

  servo.write(angle);
  Serial.print("Servo angle set to: ");
  Serial.print(angle);
  Serial.println(" degrees");
}
