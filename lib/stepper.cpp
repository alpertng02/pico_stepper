/**
 *@file stepper.cpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief Linear stepper motor driver for Raspberry Pi Pico
 * @version 0.1
 * @date 2024-07-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "stepper.hpp"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/platform.h"
#include "pico/time.h"
#include <array>
#include <cmath>

#ifdef STEPPER_DEBUG_LOG
#include <cstdio>
#endif


// How many steppers are created.
static volatile int stpCount{};
// Buffer to hold created slices.
static volatile uint stpSlice[8]{};

// All indexes in arrays below represent the slice number of the corresponding
// stepper. Unfortunately, PWM interrupt can only be used with global variables
// so in order to protect the encapsulation of the class, all global variables
// are static.

static volatile int32_t stpDir[8]{};
static volatile int32_t stpPos[8]{};
static volatile int32_t stpTargetPos[8]{};

// Fp means the value is scaled by factor of 10^3. This is to prevent floating
// point arithmatic for better performance since RP2040 does not have a FPU
// while still having some precision on arithmatic operations.

static volatile int64_t stpSpeedFp[8]{};
static volatile bool stpIsMoving[8]{};

inline static void turnOffStepper(uint slice) {
  pwm_set_enabled(slice, false);
  stpSpeedFp[slice] = 0;
  stpIsMoving[slice] = false;
}


/**
 * @brief PWM wrap interrupt to keep track of steppers' position and
 * automatically stop them once the position has been reached.
 * Due to the hardware limitations on RP2040, all PWM slices share the same
 * callback so we need to handle each stepper in this single function.
 *
 */
static void stepperPwmCallback(void) {
  const uint32_t irq = pwm_get_irq_status_mask();
  for (int i = 0; i < 8; i++) {
    const uint slice = stpSlice[i];
    if (irq & (1 << slice)) {
      // Clear the interrupt flag so the interrupt does not trigger again.
      pwm_clear_irq(slice);
      // Update the position.
      stpPos[slice] += stpDir[slice];
      stpIsMoving[slice] = true;
      // If position has been reached, update the variables needed and disable
      // pwm for the slice.
      if (stpPos[slice] == stpTargetPos[slice]) {
        turnOffStepper(slice);
      }
    }
  }
}

Stepper::Stepper(const uint pulPin, const uint dirPin,
                 const uint32_t stepsPerRev)
    : mPul(pulPin), mDir(dirPin), mSlice(pwm_gpio_to_slice_num(pulPin)),
      mStepsPerRev(stepsPerRev) {
  // Set the global variables to default values.
  stpSlice[stpCount] = mSlice;
  stpPos[mSlice] = 0;
  stpDir[mSlice] = true;
  stpCount++;

  // Initialize the direction pin.
  gpio_init(dirPin);
  gpio_set_dir(dirPin, true);
  gpio_put(dirPin, true);

  initPwm();
}

void Stepper::setStepsPerRev(const uint32_t steps) { mStepsPerRev = steps; }

void Stepper::setPos(const int32_t currentSteps) {
  stpPos[mSlice] = currentSteps;
}

void Stepper::setPos(const float currentRads) {
  setPos(radsToSteps(currentRads));
}

int32_t Stepper::getPos() { return stpPos[mSlice]; }

float Stepper::getPosRads() {
  return static_cast<float>(getPos()) * 2.0f * mPi / mStepsPerRev;
}

void Stepper::setTargetPos(const int32_t targetSteps) {
  stpTargetPos[mSlice] = targetSteps;
}

void Stepper::setTargetPos(const float targetRads) {
  setTargetPos(radsToSteps(targetRads));
}

void Stepper::setSpeed(const float rad) { setSpeed(radsToSteps(rad)); }

void Stepper::setSpeed(int32_t steps) {
  uint32_t speed = static_cast<uint32_t>(steps > 0 ? steps : 1);
  // Calculate the wrap value of the pwm counter.
  uint32_t wrap = mClockHz / speed;

  // If wrap number overflows 16 bits then increase the clock division amount to
  // achieve desired pwm frequency. If the wrap value is lower than 2^10, lower
  // the clock div to increase the resolution of the counter wrap.
  while (wrap < (0x0001 << 10) || wrap > UINT16_MAX) {
    if (wrap <= (0x0001 << 10)) {
      if (mClockDiv <= 1.0f) {
        wrap = mClockHz / speed;
        wrap = wrap > UINT16_MAX ? UINT16_MAX : wrap;
        break;
      } else {
        mClockDiv /= 2.0f;
        mClockDiv = mClockDiv <= 1.0f ? 1.0f : mClockDiv;
      }
    } else if (wrap > UINT16_MAX) {
      if (mClockDiv >= 255.9f) {
        wrap = UINT16_MAX;
        break;
      } else {
        mClockDiv *= 2.0f;
        mClockDiv = mClockDiv >= 255.92f ? 255.92f : mClockDiv;
      }
    }
    mClockHz = static_cast<uint32_t>(mSysClockHz / mClockDiv);
    pwm_set_clkdiv(mSlice, mClockDiv);
    wrap = static_cast<uint32_t>(mClockHz / static_cast<float>(speed));
  }

  mWrap = wrap;

#ifdef STEPPER_DEBUG_LOG
  printf("setSpeedFp() => wrap %lu, speedFp %lld, clkDiv %.2f\n", wrap, stepFp,
         mClockDiv);
#endif

  // Set the dutycycle to 50% to create equal rectangle waves.
  pwm_set_wrap(mSlice, mWrap);
  pwm_set_gpio_level(mPul, mWrap / 2);
}


int32_t Stepper::getActualSpeed() {
  if (isMoving()) {
    return static_cast<int32_t>(mClockHz / static_cast<float>(mWrap));
  } else {
    return 0;
  }
}

float Stepper::getActualSpeedRads() { return stepsToRads(getActualSpeed()); }

void Stepper::setDir(const bool dir) {
  stpDir[mSlice] = dir ? 1 : -1;
  gpio_put(mDir, dir);
}

int Stepper::getDir() { return stpDir[mSlice]; }

void Stepper::enable(const bool en) {
  if (!en && (stpTargetPos[mSlice] == stpPos[mSlice])) {
    turnOffStepper(mSlice);
  } else {
    if (!isMoving()) {
      stpSpeedFp[mSlice] = 0;
      stpIsMoving[mSlice] = true;
    }
    pwm_set_enabled(mSlice, en);
  }
}

bool Stepper::isMoving() { return stpIsMoving[mSlice]; }

Stepper::~Stepper() {
  pwm_set_irq_enabled(mSlice, false);
  turnOffStepper(mSlice);
  gpio_deinit(mPul);
  gpio_deinit(mDir);
}

void Stepper::initPwm() {
  gpio_set_function(mPul, GPIO_FUNC_PWM);

  pwm_config config = pwm_get_default_config();

  mSysClockHz = clock_get_hz(clk_sys);

  mClockDiv = static_cast<float>(mSysClockHz) / (0x0001 << 19);
  while (mClockDiv > (255.0f + 15.0f / 16.0f)) {
    mClockDiv /= 2.0f;
  }
  const uint16_t wrap{UINT16_MAX};

  pwm_config_set_clkdiv(&config, mClockDiv);
  pwm_config_set_wrap(&config, wrap);

  mClockHz = static_cast<uint32_t>(mSysClockHz / mClockDiv);

  pwm_init(mSlice, &config, false);
  pwm_set_irq_enabled(mSlice, true);
  pwm_set_enabled(mSlice, false);

  if (irq_get_exclusive_handler(PWM_IRQ_WRAP) == nullptr) {
    irq_set_exclusive_handler(PWM_IRQ_WRAP, stepperPwmCallback);
    irq_set_priority(PWM_IRQ_WRAP, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(PWM_IRQ_WRAP, true);
  }

  enable(false);

#ifdef STEPPER_DEBUG_LOG
  printf("initPwm() => Pwm %u => SysClockSpeed: %lu, PwmClockSpeed: %lu, "
         "ClockDiv: %.2f\n",
         mPul, mSysClockHz, mClockHz, mClockDiv);
#endif
}

int32_t Stepper::radsToSteps(const float rads) {
  return static_cast<int32_t>((rads / mPi) * (mStepsPerRev / 2.0f));
}

float Stepper::stepsToRads(const int32_t steps) {
  return static_cast<float>((steps * 2.0f * mPi) / mStepsPerRev);
}