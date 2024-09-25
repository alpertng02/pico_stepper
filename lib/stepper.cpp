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
#include <cmath>

// #define DEBUG_LOG

#ifdef DEBUG_LOG
#include <cstdio>
#endif

// Steppers created on core 1 will use this alarm pool for interrupts.
static alarm_pool* alarmPoolForCore1 = nullptr;

// How many steppers are created.
static volatile int stpCount[2]{};
// Buffer to hold created slices.
static volatile uint stpSlice[2][8]{};

// All indexes in arrays below represent the slice number of the corresponding
// stepper. Unfortunately, PWM interrupt can only be used with global variables
// so in order to protect the encapsulation of the class, all global variables
// are static.

static volatile int32_t stpDir[8]{};
static volatile int32_t stpPos[8]{};
static volatile int32_t stpTargetPos[8]{};
static volatile bool stpPosSet[8]{};

// Fp means the value is scaled by factor of 10^3. This is to prevent floating
// point arithmatic for better performance since RP2040 does not have a FPU
// while still having some precision on arithmatic operations.

static volatile int64_t stpSpeedFp[8]{};
static volatile int64_t stpTargetSpeedFp[8]{};
static volatile int64_t stpAccelFp[8]{};
static volatile int32_t stpDeaccelSteps[8]{};
static volatile int64_t stpStartingSpeedFp[8]{};
static volatile int64_t stpStoppingSpeedFp[8]{};
static volatile uint32_t stpMotionTimeMs[8]{};
static volatile bool stpAccelSet[8]{};
static volatile bool stpIsMoving[8]{};
static repeating_timer stpTimer[8]{};

/**
 * @brief Repeating timer callback for controlling the velocity of the stepper.
 * This is where majority of the processing is done for controlling the motor.
 * This timer interrupt will only be used when stepper is used with
 * startMotion() or setAccel() function.
 *
 * @tparam slice
 * @param rt
 * @return true
 * @return false
 */
template <uint slice> static bool stepperTimerCallback(repeating_timer* rt) {
    // Get the class instance that created the timer callback.
    Stepper* stepper = static_cast<Stepper*>(rt->user_data);

    const int64_t speedFp = stpSpeedFp[slice];
    const int32_t remainingSteps = stpTargetPos[slice] - stpPos[slice];
    int64_t targetSpeedFp = stpTargetSpeedFp[slice];
    // Calculate the amount of the acceleration by using timer interrupt's period
    // as reference.
    int64_t accelAmountFp = (stpAccelFp[slice] * llabs(rt->delay_us)) / 1000000;
    // If stepper is not moving in correct direction...
    if (remainingSteps * stpDir[slice] < 0) {
        stpMotionTimeMs[slice] += static_cast<uint32_t>(llabs(rt->delay_us) / 1000);
        // If stepper is in valid speed range to stop, change the direction.
        if (Stepper::IsInBounds(speedFp, stpStoppingSpeedFp[slice] - accelAmountFp,
                stpStartingSpeedFp[slice] + accelAmountFp)) {
            stepper->setDir(stpDir[slice] > 0 ? false : true);
            return stepper->startMotion(
                stpTargetPos[slice], stpAccelFp[slice] / 1000, stpMotionTimeMs[slice]);
        } else {
            targetSpeedFp = stpStoppingSpeedFp[slice];
        }
    } else if (remainingSteps * stpDir[slice] <= stpDeaccelSteps[slice]) {
        // If stepper reached deacceleration steps, deaccelerate the stepper.
        targetSpeedFp = stpStoppingSpeedFp[slice];
    }
    // If target speed is lower than current speed, acceleration is negative.
    accelAmountFp *= targetSpeedFp >= speedFp ? 1 : -1;
    int64_t changedSpeedFp = speedFp + accelAmountFp;
    changedSpeedFp = Stepper::IsInBounds(changedSpeedFp, targetSpeedFp - llabs(accelAmountFp),
                         targetSpeedFp + llabs(accelAmountFp))
                         ? targetSpeedFp
                         : changedSpeedFp;
    // Set the new speed of the stepper if the new speed calculated is different.
    // This is to prevent numerous divison operations to save time in interrupt
    // since for the most of the trajectory, speed will be constant.
    if (changedSpeedFp != speedFp) {
        stepper->setSpeedFp(changedSpeedFp);
    }
#ifdef DEBUG_LOG
    printf("Slice %u, Pos %ld, Speed %lld, Target Speed %lld, Accel Amount %lld\n", slice,
        stpPos[slice], speedFp, targetSpeedFp, accelAmountFp);
#endif
    return true;
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
    const uint coreNum = get_core_num();
    for (int i = 0; i < stpCount[coreNum]; i++) {
        const uint slice = stpSlice[coreNum][i];
        if (irq & (1 << slice)) {
            // printf("stepperPwmCallback slice %u on core%u\n", slice, get_core_num());
            // Clear the interrupt flag so the interrupt does not trigger again.
            pwm_clear_irq(slice);
            // Update the position.
            stpPos[slice] += stpDir[slice];
            stpIsMoving[slice] = true;
            // If position has been reached, update the variables needed and disable
            // pwm for the slice.
            if (stpPosSet[slice] && stpPos[slice] == stpTargetPos[slice]) {
                stpSpeedFp[slice] = stpStartingSpeedFp[slice];
                stpPosSet[slice] = false;
                stpAccelSet[slice] = false;
                stpIsMoving[slice] = false;
                cancel_repeating_timer(&stpTimer[slice]);
                pwm_set_enabled(slice, false);
                continue;
            }
        }
    }
}

Stepper::Stepper(
    const uint pulPin, const uint dirPin, const uint32_t stepsPerRev, const uint32_t periodMs)
    : mPul(pulPin), mDir(dirPin), mSlice(pwm_gpio_to_slice_num(pulPin)), mStepsPerRev(stepsPerRev),
      mPeriodMs(periodMs) {
    // Set the global variables to default values.
    auto coreNum = get_core_num();
    stpSlice[coreNum][stpCount[coreNum]] = mSlice;
    stpPos[mSlice] = 0;
    stpPosSet[mSlice] = false;
    stpAccelSet[mSlice] = false;
    stpDir[mSlice] = true;
    stpCount[coreNum]++;

    // Initialize the direction pin.
    gpio_init(dirPin);
    gpio_set_dir(dirPin, true);
    gpio_put(dirPin, true);

    initPwm();

    // If the class is created from core 1, create alarm pool for core 1 timer
    // interrupts.
    if (coreNum && alarmPoolForCore1 == nullptr) {
        alarmPoolForCore1 = alarm_pool_create_with_unused_hardware_alarm(8);
    }

    setStartingSpeed(static_cast<int32_t>(stepsPerRev / 10));
    // TODO add stopping speed factor to calculateTargetSpeed() equation.
    setStoppingSpeed(static_cast<int32_t>(0));
}

void Stepper::setStepsPerRev(const uint32_t steps) {
    mStepsPerRev = steps;
}


void Stepper::setTimerPeriod(const uint32_t periodMs) {
    mPeriodMs = periodMs;
    stpTimer[mSlice].delay_us = -mPeriodMs * 1000;
}

void Stepper::setAccel(const int32_t accelSteps) { setAccelFp(accelSteps * 1000); }

void Stepper::setAccel(const float accelRads) { setAccel(radsToSteps(accelRads)); }

void Stepper::setAccelFp(const int64_t accelFp) {
    stpAccelSet[mSlice] = true;
    stpAccelFp[mSlice] = accelFp;
}

void Stepper::setDeaccelSteps(const int32_t deaccelSteps) {
    stpDeaccelSteps[mSlice] = deaccelSteps;
}

void Stepper::setPos(const int32_t currentSteps) { stpPos[mSlice] = currentSteps; }

void Stepper::setPos(const float currentRads) { setPos(radsToSteps(currentRads)); }

int32_t Stepper::getPos() { return stpPos[mSlice]; }

float Stepper::getPosRads() { return static_cast<float>(getPos()) * 2.0f * mPi / mStepsPerRev; }

void Stepper::setTargetPos(const int32_t targetSteps) {
    stpTargetPos[mSlice] = targetSteps;
    stpPosSet[mSlice] = true;
}

void Stepper::setTargetPos(const float targetRads) { setTargetPos(radsToSteps(targetRads)); }

void Stepper::setSpeed(const int32_t step) { setSpeedFp(step * 1000); }

void Stepper::setSpeed(const float rad) { setSpeed(radsToSteps(rad)); }

void Stepper::setSpeedFp(const int64_t stepFp) {
    // Calculate the wrap value of the pwm counter.
    const int32_t steps = static_cast<int32_t>(stepFp / 1000);
    uint32_t wrap = mClockHz / static_cast<uint32_t>(((steps > 0) ? steps : 1));

    // If wrap number overflows 16 bits then increase the clock division amount to
    // achieve desired pwm frequency. If the wrap value is lower than 2^10, lower
    // the clock div to increase the precision of the counter wrap.
    while (wrap < (0x0001 << 10) || wrap > UINT16_MAX) {
        if (wrap <= (0x0001 << 10)) {
            if (mClockDiv <= 1.0f) {
                wrap = mClockHz / static_cast<uint32_t>(((steps > 0) ? steps : 1));
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
        wrap = mClockHz / static_cast<uint32_t>(steps);
    }

    mWrap = wrap;
    stpSpeedFp[mSlice] = stepFp;

#ifdef DEBUG_LOG
    printf("setSpeedFp => wrap %lu, speedFp %lld, clkDiv %.2f\n", wrap, stepFp, mClockDiv);
#endif

    // Set the dutycycle to 50% to create equal rectangle waves.
    pwm_set_wrap(mSlice, mWrap);
    pwm_set_gpio_level(mPul, mWrap / 2);
}

void Stepper::setTargetSpeed(const int32_t targetSpeed) { setTargetSpeedFp(targetSpeed * 1000); }

void Stepper::setTargetSpeed(const float targetSpeed) { setTargetSpeed(radsToSteps(targetSpeed)); }

void Stepper::setTargetSpeedFp(const int64_t targetSpeedFp) {
    stpTargetSpeedFp[mSlice] = targetSpeedFp;
}

void Stepper::setStartingSpeed(const int32_t steps) { stpStartingSpeedFp[mSlice] = steps * 1000; }

void Stepper::setStartingSpeed(const float rads) { setStartingSpeed(radsToSteps(rads)); }

void Stepper::setStoppingSpeed(const int32_t steps) { stpStoppingSpeedFp[mSlice] = steps * 1000; }

void Stepper::setStoppingSpeed(const float rads) { setStoppingSpeed(radsToSteps(rads)); }

int32_t Stepper::getActualSpeed() { return mClockHz / mWrap; }

float Stepper::getActualSpeedRads() { return stepsToRads(getActualSpeed()); }

void Stepper::setDir(const bool dir) {
    stpDir[mSlice] = dir ? 1 : -1;
    gpio_put(mDir, dir);
}

int Stepper::getDir() { return stpDir[mSlice]; }

void Stepper::enable(const bool en) {
    if (!en || (stpPosSet[mSlice] && (stpTargetPos[mSlice] == stpPos[mSlice]))) {
        pwm_set_enabled(mSlice, false);
        cancel_repeating_timer(&stpTimer[mSlice]);
        stpIsMoving[mSlice] = false;
        stpPosSet[mSlice] = false;
        stpAccelSet[mSlice] = false;
    } else {
        pwm_set_enabled(mSlice, true);
        if (!isMoving() && stpAccelSet[mSlice]) {
            stpSpeedFp[mSlice] = stpStartingSpeedFp[mSlice];
            if (get_core_num() == 1) {
                alarm_pool_add_repeating_timer_ms(
                    alarmPoolForCore1, -mPeriodMs, getTimerCallback(), this, &stpTimer[mSlice]);
            } else {
                add_repeating_timer_ms(-mPeriodMs, getTimerCallback(), this, &stpTimer[mSlice]);
            }
        }
    }
}

bool Stepper::startMotion(const int32_t targetPosSteps, const int32_t accelSteps,
    const uint32_t timeMs, const bool start) {
    const int32_t deltaSteps = targetPosSteps - stpPos[mSlice];
    const int32_t initialSpeed =
        isMoving() ? stpSpeedFp[mSlice] / 1000 : stpStartingSpeedFp[mSlice] / 1000;
    const int32_t targetSpeed =
        abs(calculateTargetSpeed(deltaSteps, initialSpeed, accelSteps, timeMs));

    if (targetSpeed == 0) {
#ifdef DEBUG_LOG
        printf("StartMotion =>  Slice %u, DeltaSteps %ld, InitialSpeed %ld, Not "
               "Possible!!!\n",
            mSlice, deltaSteps, initialSpeed);
#endif
        return false;
    }

    if (!isMoving() && deltaSteps * stpDir[mSlice] < 0) {
        setDir(stpDir[mSlice] == 1 ? false : true);
    }

    stpMotionTimeMs[mSlice] = timeMs;
    setTargetPos(targetPosSteps);
    setTargetSpeed(targetSpeed);
    setAccel(accelSteps);
    setDeaccelSteps(abs(getStepIncrease(
        targetSpeed, -abs(accelSteps), static_cast<uint32_t>((targetSpeed * 1000) / accelSteps))));
#ifdef DEBUG_LOG
    printf("StartMotion =>  Slice %u, TargetPos %ld, targetSpeed %ld, "
           "deaccelSteps %ld\n",
        mSlice, targetPosSteps, targetSpeed, stpDeaccelSteps[mSlice]);
#endif
    enable(start);
    return true;
}

bool Stepper::startMotion(
    const float targetPosRads, const float accelRads, const float sec, const bool start) {
    return startMotion(radsToSteps(targetPosRads), radsToSteps(accelRads),
        static_cast<uint32_t>(sec * 1000), start);
}

bool Stepper::isMoving() { return stpIsMoving[mSlice]; }

Stepper::~Stepper() {
    pwm_set_irq_enabled(mSlice, false);
    pwm_set_enabled(mSlice, false);
    cancel_repeating_timer(&stpTimer[mSlice]);
    gpio_deinit(mPul);
    gpio_deinit(mDir);
}

void Stepper::initPwm() {
    gpio_set_function(mPul, GPIO_FUNC_PWM);

    pwm_config config = pwm_get_default_config();

    mSysClockHz = clock_get_hz(clk_sys);

    mClockDiv = static_cast<float>(mSysClockHz / (0x0001 << 19));
    while (mClockDiv > (255.0f + 15.0f / 16.0f)) {
        mClockDiv /= 2.0f;
    }
    const uint16_t wrap{ UINT16_MAX };

    pwm_config_set_clkdiv(&config, mClockDiv);
    pwm_config_set_wrap(&config, wrap);

    mClockHz = static_cast<uint32_t>(mSysClockHz / mClockDiv);

    pwm_init(mSlice, &config, false);
    pwm_set_irq_enabled(mSlice, true);
    pwm_set_enabled(mSlice, false);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, stepperPwmCallback);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    enable(false);

#ifdef DEBUG_LOG
    printf("Pwm %u => SysClockSpeed: %lu, PwmClockSpeed: %lu, ClockDiv: %.2f\n", mPul, mSysClockHz,
        mClockHz, mClockDiv);
#endif
}

int32_t Stepper::radsToSteps(const float rads) {
    return static_cast<int32_t>((rads / mPi) * (mStepsPerRev / 2));
}

float Stepper::stepsToRads(const int32_t steps) {
    return static_cast<float>((steps * 2.0f * mPi) / mStepsPerRev);
}

int32_t Stepper::calculateTargetSpeed(
    int32_t deltaSteps, int32_t initialSpeed, int32_t accel, uint32_t timeMs) {
    deltaSteps = labs(deltaSteps);
    const float deltaT = timeMs / 1000.0f;
    const float a = 1;
    const float b = -(initialSpeed + deltaT * accel);
    const float initialSpeedSqrd =
        static_cast<float>(initialSpeed) * static_cast<float>(initialSpeed);
    const float cRightSide = static_cast<float>(accel) * static_cast<float>(-deltaSteps);
    const float c = (initialSpeedSqrd / 2.0f + cRightSide);

    const float disc = b * b - 4.0f * a * c;

#ifdef DEBUG_LOG
    printf("calculateTargetSpeed() => deltaT %.2f, a %.2f, b %.2f, Vi^2 %.2f, "
           "CRight %.2f, c %.2f, disc "
           "%.2f\n",
        deltaT, a, b, initialSpeedSqrd, cRightSide, c, disc);
#endif
    int32_t result = 0;
    if (disc > 0) {
        const float discSqrt = sqrtf(disc);
        float vf1 = (-b - discSqrt) / (2.0f * a);
        if (vf1 / accel > deltaT / 2.0f) {
            vf1 = (-b + discSqrt) / (2.0f * a);
#ifdef DEBUG_LOG
            printf("Alternate root vf1 = %.2f\n", vf1);
#endif
        }
#ifdef DEBUG_LOG
        else {
            printf("First root vf1 = %.2f\n", vf1);
        }
#endif
        if (vf1 < initialSpeed) {
            result =
                static_cast<int32_t>((deltaSteps - initialSpeed * initialSpeed / (2.0f * accel)) /
                                     (deltaT - initialSpeed / static_cast<float>(accel)));
        } else {
            result = static_cast<int32_t>(vf1);
        }
    }
#ifdef DEBUG_LOG
    printf("Result %ld\n", result);
#endif
    return result;
}

int32_t Stepper::getStepIncrease(
    const int32_t currentSpeed, const int32_t currentAccel, const uint32_t timeMs) {
    const int64_t accelPart{ static_cast<int64_t>(currentAccel) * timeMs * timeMs };
    const int64_t speedPart{ static_cast<int64_t>(currentSpeed) * timeMs };
    const int32_t pos = static_cast<int32_t>((accelPart / (1000 * 1000 * 2)) + (speedPart / 1000));
    return pos;
}

int32_t Stepper::getSpeedIncrease(const int32_t currentAccel, const uint32_t timeMs) {
    const int32_t accelPart{ static_cast<int32_t>(((currentAccel * timeMs) / 1000)) };
    return accelPart;
}

repeating_timer_callback_t Stepper::getTimerCallback() {
    switch (mSlice) {
        case 0:
            return stepperTimerCallback<0>;
        case 1:
            return stepperTimerCallback<1>;
        case 2:
            return stepperTimerCallback<2>;
        case 3:
            return stepperTimerCallback<3>;
        case 4:
            return stepperTimerCallback<4>;
        case 5:
            return stepperTimerCallback<5>;
        case 6:
            return stepperTimerCallback<6>;
        case 7:
            return stepperTimerCallback<7>;
        default:
            return nullptr;
    }
}
