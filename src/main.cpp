#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Motor driver pin definitions
#define LEFT_MOTOR_IN1 25  // ESP32 GPIO25 -> MX1508 IN1
#define LEFT_MOTOR_IN2 26  // ESP32 GPIO26 -> MX1508 IN2
#define RIGHT_MOTOR_IN3 27 // ESP32 GPIO27 -> MX1508 IN3
#define RIGHT_MOTOR_IN4 13 // ESP32 GPIO13 -> MX1508 IN4

// ToF sensor object
Adafruit_VL53L0X tof = Adafruit_VL53L0X();

// Function declarations
void setupMotors();
void setLeftMotor(int power);
void setRightMotor(int power);
void setBothMotors(int leftPower, int rightPower);
void stopMotors();
void setupToF();
float getDistance();

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 Motor Test Starting...");

  setupMotors();
  setupToF();

  Serial.println("Setup complete. Starting main loop...");
  delay(2000);
}

void loop()
{

  delay(100); // Read distance every second
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
    // Forward direction
    analogWrite(LEFT_MOTOR_IN1, power);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    Serial.print("Left Motor: Forward, Power: ");
    Serial.println(power);
  }
  else if (power < 0)
  {
    // Reverse direction
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    analogWrite(LEFT_MOTOR_IN2, abs(power));
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
    // Forward direction
    analogWrite(RIGHT_MOTOR_IN3, power);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
    Serial.print("Right Motor: Forward, Power: ");
    Serial.println(power);
  }
  else if (power < 0)
  {
    // Reverse direction
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    analogWrite(RIGHT_MOTOR_IN4, abs(power));
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
