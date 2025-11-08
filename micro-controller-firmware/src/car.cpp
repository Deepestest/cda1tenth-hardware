#include "car.h"
#include <limits.h>
#include <math.h>

// Unified Motor implementation: supports both steering and drive behavior
Motor::Motor(int cs, bool steering) : driver(cs, R_SENSE), cs_pin(cs), isSteering(steering) {}

void Motor::begin()
{
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
  driver.begin();

  if (isSteering)
  {
    // Steering configuration (positioning)
    driver.rms_current(300);
    driver.ihold(5);
    driver.irun(50);
    driver.iholddelay(5);
    driver.microsteps(MICROSTEPS);
    driver.en_pwm_mode(false);
    driver.pwm_autoscale(true);
    driver.toff(3);
    driver.blank_time(24);

    driver.a1(500);
    driver.v1(500);
    driver.AMAX(5000);
    driver.DMAX(5000);
    driver.VMAX(8000);
    driver.d1(500);
    driver.VSTOP(10);

    driver.RAMPMODE(0); // Positioning mode
    lastCorrectionMicros = micros();
  }
  else
  {
    // Drive configuration (velocity)
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);
    driver.shaft(true);
    driver.rms_current(1000);
    driver.microsteps(MICROSTEPS);
    driver.en_pwm_mode(true);
    driver.pwm_autoscale(true);
    driver.TCOOLTHRS(0xFFFFF);
    driver.THIGH(0);
    driver.semin(5);
    driver.semax(2);
    driver.sedn(0b01);
    driver.toff(3);
    driver.blank_time(24);
    driver.ihold(10);
    driver.irun(31);
    driver.iholddelay(5);
    driver.VDCMIN(0);
    driver.a1(1000);
    driver.v1(1000);
    driver.AMAX(1000);
    driver.DMAX(1000);
    driver.d1(1000);
    driver.VSTOP(10);
    driver.RAMPMODE(2);
    driver.X_ENC(0);
  }
}

float Motor::normalizeAngle(float angle)
{
  while (angle > 180.0f)
    angle -= DEGREES_PER_REVOLUTION;
  while (angle < -180.0f)
    angle += DEGREES_PER_REVOLUTION;
  return angle;
}

float Motor::getSteeringSensorAngle()
{
  int raw = analogRead(STEERING_SENSOR_PIN);
  float angle = ((float)raw / STEERING_SENSOR_MAX_VALUE) * DEGREES_PER_REVOLUTION;
  return angle;
}

void Motor::setPosition(float radians)
{
  // public API requested: setPosition accepts radians (position)
  if (!isSteering)
    return;

  float degrees = radians * 180.0f / M_PI;
  float stepsPerRev = MOTOR_STEPS * MICROSTEPS;
  float currentAngle = normalizeAngle(getSteeringSensorAngle() - angleOffset);
  targetAngle = normalizeAngle(-degrees);
  int32_t actualSteps = (int32_t)((currentAngle / DEGREES_PER_REVOLUTION) * stepsPerRev * STEERING_GEAR_RATIO);
  driver.XACTUAL(actualSteps);
  float targetSteps = (targetAngle / DEGREES_PER_REVOLUTION) * stepsPerRev * STEERING_GEAR_RATIO;
  driver.XTARGET((int32_t)targetSteps);
}

bool tmc5160_recover(TMC5160Stepper &drv, int ENN_PIN)
{
  // Shared recovery for drive faults
  digitalWrite(ENN_PIN, HIGH);
  drv.toff(0);
  drv.GSTAT(0b111);
  delay(3);
  drv.irun(10);
  drv.toff(3);
  digitalWrite(ENN_PIN, LOW);
  delay(2);
  uint8_t gstat = drv.GSTAT();
  drv.irun(31);
  drv.toff(10);
  return (gstat & (1 << 2)) == 0;
}

void Motor::setSpeed(float rpm)
{
  if (isSteering)
  {
    // no-op for steering motors (speed controlled by position)
    return;
  }
  target_steps_per_sec = (abs(rpm) / 60.0f) * MOTOR_STEPS * MICROSTEPS;
  target_rpm = rpm;

  // Apply immediately to driver for more responsive behavior during testing.
  // This forces VMAX to requested value so the motor should start moving toward
  // the requested speed without waiting for the PID-like ramping logic.
  step_rate_cmd = target_steps_per_sec;
  driver.shaft(target_rpm < 0);
}

void Motor::updateControlloops()
{
  if (isSteering)
  {
    // steering update (position correction)
    uint32_t now = micros();
    if (now - lastCorrectionMicros < STEERING_CORRECTION_INTERVAL)
      return;
    lastCorrectionMicros = now;
    float currentAngle = normalizeAngle(getSteeringSensorAngle() - angleOffset);
    float error = normalizeAngle(targetAngle - currentAngle);

    if (fabsf(currentAngle - lastExternalAngle) < SMALL_MOVEMENT_THRESHOLD)
    {
      stallCounterSteer++;
    }
    else
    {
      stallCounterSteer = 0;
    }
    lastExternalAngle = currentAngle;

    float stepsPerRev = MOTOR_STEPS * MICROSTEPS;
    int32_t actualSteps = (int32_t)((currentAngle / 360.0f) * stepsPerRev * STEERING_GEAR_RATIO);

    if (stallCounterSteer > STALL_DETECTION_COUNT)
    {
      driver.XACTUAL(actualSteps);
      driver.XTARGET(actualSteps);
      stallCounterSteer = 0;
      return;
    }

    if (fabsf(error) > STEERING_MAX_ALLOWED_ERROR)
    {
      driver.XACTUAL(actualSteps);
    }
  }
  else
  {
    // drive update (velocity control)
    if (driver.GSTAT() & (1 << 2))
    {
      if (!tmc5160_recover(driver, EN_PIN))
      {
        return;
      }
    }
    int32_t current_enc = driver.X_ENC();
    uint32_t now = micros();
    uint32_t dt = now - last_time;
    int32_t delta_enc = current_enc - last_enc;

    if (dt > 0)
    {
      float measured_ticks_per_sec = (float)delta_enc * 1e6f / dt;
      float measured_steps_per_sec = measured_ticks_per_sec * (MOTOR_STEPS * MICROSTEPS / ENCODER_TICKS_PER_REVOLUTION);
      measured_steps_per_sec = abs(measured_steps_per_sec);

      current_rpm = (measured_steps_per_sec / (MOTOR_STEPS * MICROSTEPS)) * 60.0f;

      float error = target_steps_per_sec - measured_steps_per_sec;
      int32_t adjustment = (int32_t)(error * DRIVE_ERROR_GAIN);

      if (measured_steps_per_sec < (DRIVE_STALL_THRESHOLD * step_rate_cmd))
      {
        stall_counter_drive++;
      }
      else
      {
        stall_counter_drive = 0;
      }

      if (stall_counter_drive > DRIVE_MAX_STALL_COUNT)
      {
        step_rate_cmd -= DRIVE_STALL_REDUCTION * stall_counter_drive;
        if ((int32_t)step_rate_cmd < 0)
          step_rate_cmd = 0;
      }
      else
      {
        step_rate_cmd += adjustment;
        if ((int32_t)step_rate_cmd < 0)
          step_rate_cmd = 0;
      }

      float max_step_change = MAX_STEP_ACCEL * (dt / 1e6f);
      if (target_steps_per_sec > step_rate_cmd + max_step_change)
      {
        step_rate_cmd += max_step_change;
      }
      else if (target_steps_per_sec < step_rate_cmd - max_step_change)
      {
        step_rate_cmd -= max_step_change;
      }
      else
      {
        step_rate_cmd = target_steps_per_sec;
      }

      if (step_rate_cmd < 0.0f)
        step_rate_cmd = 0.0f;

      driver.VMAX(step_rate_cmd);
      driver.shaft(target_rpm < 0);
    }

    last_enc = current_enc;
    last_time = now;
  }
}

Car::Car(int motorCS)
    : motor(motorCS, false)
{
  carMutex = xSemaphoreCreateMutex();
}

void Car::lock()
{
  xSemaphoreTake(carMutex, portMAX_DELAY);
}

void Car::unlock()
{
  xSemaphoreGive(carMutex);
}

void Car::updateControlLoops()
{
  lock();
  motor.updateControlloops();
  unlock();
}

void Car::begin()
{
  motor.begin();
}
void Car::setSpeed(float rpm)
{
  lock();
  speed = rpm;
  motor.setSpeed(rpm);
  unlock();
}

float Car::getMotorRPM()
{
  lock();
  float rpm = motor.current_rpm;
  unlock();
  return rpm;
}