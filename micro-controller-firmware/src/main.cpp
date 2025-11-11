#include <Arduino.h>
#include "PestolinkAgent.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>

// Include car control logic
#include "car.h"

#define EXECUTE_EVERY_N_MS_MS_VAR_NAME_CONCAT(inner) inner

// Motor/vehicle configuration
float g_wheelbase = 0.185f;
float g_track_width = 0.15f;

// Desmos link: https://www.desmos.com/calculator/hzw38ukeor
float g_steering_scaling_factor = (1.0f / 0.87f); // Scaling factor for encoder angle to wheel steering angle
float o_speed_scaling_factor = 4.0f;              // Factor to scale the speed command

// Steering control constants
const float MAX_STEERING_ANGLE = 45.0f; // degrees

// Car control instance (two motors)
Car car(CS_RIGHT, CS_LEFT, CS_STEER);

// Car initialization flags
bool car_initialized = false;

// PestoLink BLE agent (constructed in setup)
PestoLinkAgent *pestoAgent = NULL;

// Telemetry timing
uint32_t lastTelemetryMs = 0;

// (Removed micro-ROS callbacks and message/subscription declarations)

void initializeCar()
{
  if (!car_initialized)
  {
    // Initialize SPI for motor drivers
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
    delay(100);

    // Initialize car control
    car.begin();
    delay(100);

    car.setSpeed(0.0f); // Start with zero speed
    car.setMotor2Speed(0.0f);

    car_initialized = true;
  }
}

// No micro-ROS entities anymore; PestoLinkAgent will provide commands via BLE

void setup()
{
  // Initialize the car control system
  initializeCar();
  // Construct PestoLinkAgent (BLE remote control)
  pestoAgent = new PestoLinkAgent("PestoCar");
}

float angle = 45.00f;
uint32_t lastApplyMicros = micros();

// Test helper removed (not used)

void loop()
{
  if (pestoAgent && car_initialized)
  {
    car.setSpeed(10000.0f);
    car.setMotor2Speed(100000.0f);
    car.setMotor3Speed(100000.0f);
    // Buttons control the single motor; axes are ignored in this mode

    // Decide motor command from PestoLink buttons
    const float wheel_rad_per_sec = 2.0f;
    const float rpm_cmd = wheel_rad_per_sec * 60.0f / (2.0f * M_PI); // rad/s -> rpm

    // Do not use USB Serial; use BLE only. Connection state can be checked if needed.
    if (pestoAgent->get_button(3))
    {
      car.setSpeed(rpm_cmd);
      car.setMotor2Speed(rpm_cmd);
    }
    else if (pestoAgent->get_button(1))
    {
      car.setSpeed(-rpm_cmd);
      car.setMotor2Speed(-rpm_cmd);
    }
    else
    {
      car.setSpeed(0.0f);
      car.setMotor2Speed(0.0f);
    }

    // Update control loops
    car.updateControlLoops();

    // Periodic telemetry over BLE (every 200 ms)
    uint32_t now = millis();
    if (now - lastTelemetryMs >= 200)
    {
      float motor_rpm = car.getMotorRPM();
      char buf[32];
      snprintf(buf, sizeof(buf), "RPM:%d", (int)motor_rpm);
      pestoAgent->telemetryPrint(String(buf), "00FF00");
      lastTelemetryMs = now;
    }
  }
}