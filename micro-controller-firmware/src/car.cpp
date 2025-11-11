#include "car.h"
#include <limits.h>
#include <math.h>

// Simple open-loop Motor implementation (no encoder, no steering position)
// Motor maps rpm -> steps/sec and directly programs the driver VMAX.

#define OPEN_LOOP_VMAX_MULTIPLIER 1.0f

Motor::Motor(int cs)
    : driver(cs, R_SENSE), cs_pin(cs), step_rate_cmd(0), target_steps_per_sec(0),
      current_rpm(0.0f), target_rpm(0.0f), percent(0.0f), last_time(0)
{
}

void Motor::begin()
{
  pinMode(cs_pin, OUTPUT);
  digitalWrite(cs_pin, HIGH);
  driver.begin();

  // Basic drive configuration (velocity mode)
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

  // start with stopped motor
  step_rate_cmd = 0;
  target_steps_per_sec = 0;
  current_rpm = 0.0f;
  target_rpm = 0.0f;
  last_time = micros();
}

bool tmc5160_recover(TMC5160Stepper &drv, int ENN_PIN)
{
  // Attempt a simple recovery sequence for UV/CP faults
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
  rpm = rpm * 4;
  // rpm can be positive or negative; we convert magnitude to steps/sec
  target_rpm = rpm;
  float mag_rpm = fabsf(rpm);
  target_steps_per_sec = (uint32_t)((mag_rpm / 60.0f) * MOTOR_STEPS * MICROSTEPS);
  // ensure percent reflects current target for debug
  if (TEST_MAX_RPM > 0.0f)
  {
    percent = (mag_rpm / TEST_MAX_RPM) * 100.0f;
    if (percent > 100.0f)
      percent = 100.0f;
  }
  // Immediately program the driver for open-loop operation.
  uint32_t vmax_value = (uint32_t)(target_steps_per_sec * OPEN_LOOP_VMAX_MULTIPLIER);
  step_rate_cmd = vmax_value;
  driver.VMAX(vmax_value);
  driver.shaft(target_rpm < 0);
  // reflect commanded rpm as current in open-loop
  current_rpm = target_rpm;
}

void Motor::setPercent(float p)
{
  if (p > 100.0f)
    p = 100.0f;
  if (p < -100.0f)
    p = -100.0f;
  percent = p;
  // map percent to rpm using TEST_MAX_RPM
  float rpm = (p / 100.0f) * TEST_MAX_RPM;
  setSpeed(rpm);
}

void Motor::updateControlloops()
{
  // No control loop: ensure driver is set to the commanded value.
  uint32_t vmax_value = (uint32_t)(target_steps_per_sec * OPEN_LOOP_VMAX_MULTIPLIER);
  step_rate_cmd = vmax_value;
  driver.VMAX(vmax_value);
  driver.shaft(target_rpm < 0);
  // In purely open-loop mode we report the commanded rpm as the current rpm.
  current_rpm = target_rpm;
  last_time = micros();
}

// ----- Car implementation -----
Car::Car(int motorCS, int motor2CS, int motor3CS)
    : motor(motorCS), motor2(motor2CS), motor3(motor3CS)
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
  motor2.updateControlloops();
  motor3.updateControlloops();
  unlock();
}

void Car::begin()
{
  motor.begin();
  motor2.begin();
  motor3.begin();
}

void Car::setSpeed(float rpm)
{
  lock();
  speed = rpm;
  motor.setSpeed(rpm);
  unlock();
}

void Car::setMotor2Speed(float rpm)
{
  lock();
  motor2.setSpeed(rpm);
  unlock();
}

void Car::setMotor3Speed(float rpm)
{
  lock();
  motor3.setSpeed(rpm);
  unlock();
}

float Car::getMotorRPM()
{
  lock();
  float rpm = motor.current_rpm;
  unlock();
  return rpm;
}

float Car::getMotor2RPM()
{
  lock();
  float rpm = motor2.current_rpm;
  unlock();
  return rpm;
}

float Car::getMotor3RPM()
{
  lock();
  float rpm = motor3.current_rpm;
  unlock();
  return rpm;
}