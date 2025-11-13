#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>

// Include car control logic
#include "car.h"

// Include PestoLink BLE remote control
#include "PestoLink-Receive.h"

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

// Telemetry timing
uint32_t lastTelemetryMs = 0;
// Use the global PestoLink instance declared in PestoLink-Receive.h
// (do not create a second local instance named `pesto`)

// (Removed micro-ROS callbacks and message/subscription declarations)

void initializeCar()
{
  static bool serialStarted = false;
  if (!serialStarted)
  {
    Serial.begin(115200);
    delay(10);
    serialStarted = true;
  }
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
  PestoLink.begin("Frontloader");
  // Initialize the car control system
  initializeCar();
}

float angle = 45.00f;
uint32_t lastApplyMicros = micros();

// Test helper removed (not used)

void loop()
{
  PestoLink.update();
  if (PestoLink.isConnected())
  {
    char telemetryBuf[32];
    // Convert float to string (dtostrf: value, min width, precision, buffer)
    dtostrf(PestoLink.getAxis(0), 6, 3, telemetryBuf);
    PestoLink.printTelemetry(telemetryBuf);
    // Read battery voltage and send as telemetry.
    // Adjust BATT_PIN, ADC_MAX, VREF and VOLTAGE_DIVIDER_RATIO for your hardware.
    const int BATT_PIN = A0;
    const float ADC_MAX = 4095.0f;            // use 1023.0f for 10-bit ADC (Arduino UNO), 4095 for 12-bit (ESP32)
    const float VREF = 3.3f;                  // ADC reference voltage
    const float VOLTAGE_DIVIDER_RATIO = 2.0f; // set according to your resistor divider

    int raw = analogRead(BATT_PIN);
    float batteryVoltage = (raw / ADC_MAX) * VREF * VOLTAGE_DIVIDER_RATIO;

    char battBuf[16];
    dtostrf(batteryVoltage, 5, 2, battBuf); // width 5, 2 decimal places
    PestoLink.printTelemetry(battBuf);
  }
  if (car_initialized)
  {
    if (PestoLink.isConnected())
    {
      float steeringInput = PestoLink.getAxis(1); // Assume axis 1 is steering
      float speedInput = PestoLink.getAxis(0);    // Assume axis 0 is speed

      // Map steering input (-1 to 1) to steering angle
      float steeringAngle = 100.0 * steeringInput;
      // Map speed input (-1 to 1) to speed in rpm
      float speedRpm = speedInput * 100.0f * o_speed_scaling_factor; // Max 100 rpm scaled
      float left = speedRpm + (steeringAngle);
      float right = speedRpm - (steeringAngle);
      float max = fmaxf(fabsf(left), fabsf(right));
      if (max > 100.0f)
      {
        left = (left / max) * 100.0f;
        right = (right / max) * 100.0f;
      }

      car.setSpeed(right);
      car.setMotor2Speed(left);
      // For steering, we can use motor3 as the steering motor
    }
    else
    {
      Serial.println("Disconnected");
      const float speed = 67.0f;
      car.setSpeed(speed);
      car.setMotor2Speed(speed);
      car.setMotor3Speed(speed);
      car.updateControlLoops();
    }
    delay(10);
  }
}