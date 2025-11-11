#ifndef CAR_H
#define CAR_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <limits.h>

// SPI pin definitions
#define CS_STEER 41
#define CS_RIGHT 39
#define CS_LEFT 40
#define MOSI_PIN 11
#define MISO_PIN 13
#define SCK_PIN 12
#define EN_PIN 4

// Stepper and driver parameters
#define R_SENSE 0.075f
#define MOTOR_STEPS 200
#define MICROSTEPS 16

// Steering sensor parameters
#define STEERING_SENSOR_PIN 18
#define STEERING_SENSOR_MAX_VALUE 4095.0f
#define DEGREES_PER_REVOLUTION 360.0f

// Encoder parameters (removed - running open-loop)

// Steering control parameters
#define STEERING_DEADBAND 0.5f
#define STEERING_KP 400.0f
#define STEERING_MAX_SPEED 12000.0f
#define STEERING_ERROR_THRESHOLD 5.0f
#define STEERING_CORRECTION_INTERVAL 50000  // 50 ms
#define STEERING_GEAR_RATIO (55.0f / 12.0f) // ≈ 4.5833
#define STEERING_MAX_ALLOWED_ERROR 2.0f
#define STALL_DETECTION_COUNT 10      // ~500 ms
#define SMALL_MOVEMENT_THRESHOLD 0.5f // degrees

// Drive motor control parameters
#define DRIVE_ERROR_GAIN 1.0f
#define DRIVE_STALL_THRESHOLD 0.5f
#define DRIVE_STALL_REDUCTION 250
#define DRIVE_MAX_STALL_COUNT 5
#define MAX_STEP_ACCEL 200.0f
// Test max RPM used by setPercent (open-loop test). Adjust if you need higher speed.
#define TEST_MAX_RPM 120.0f

// (Removed DRIVE_TEST_MAX_STEP_RATE) Use TEST_MAX_RPM to map percent -> rpm -> steps/sec

class Motor
{
public:
  // Constructor and lifecycle
  Motor(int cs);
  void begin();

  // Simplified public API (open-loop)
  void setSpeed(float rpm);        // For drive motors: rpm
  void setPercent(float percent);  // Open-loop percent -100..100 (no encoder)
  void updateControlloops();       // Run control loop updates (open-loop ramping)

  friend class Car; // allow Car to read internal status (keeps Motor API minimal)

private:
  // underlying driver and pins
  TMC5160Stepper driver{0, R_SENSE};
  int cs_pin = -1;

  // Drive-specific internal state (open-loop - no encoder)
  uint32_t step_rate_cmd = 0;         // current applied steps/sec
  uint32_t target_steps_per_sec = 0;  // desired steps/sec
  float current_rpm = 0.0f;
  float target_rpm = 0.0f;
  float percent = 0.0f;               // open-loop percent command (-100..100)
  uint32_t last_time = 0;             // micros() timestamp for ramping
};

class Car
{
public:
  float speed = 0.0f;
  Motor motor;  // primary motor
  Motor motor2; // secondary motor
  Motor motor3; // tertiary motor

  Car(int motorCS, int motor2CS, int motor3CS);
  void updateControlLoops();
  void begin();
  void setSpeed(float rpm);
  void setMotor2Speed(float rpm);
  void setMotor3Speed(float rpm);
  float getMotorRPM();
  float getMotor2RPM();
  float getMotor3RPM();

private:
  SemaphoreHandle_t carMutex;
  void lock();
  void unlock();
};

#endif // CAR_H