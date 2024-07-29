/**
 * @file stepper.cpp
 * @brief Implementation of the Stepper class for controlling a stepper motor using Raspberry Pi Pico.
 */
#include "stepper.hpp"

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico/time.h"
#include "pico/platform.h"
#include "pico/float.h"
#include <cmath>
#include <cstdio>

static alarm_pool* alarmPoolForCore1 = nullptr;

static volatile int stpCount = 0;

static volatile uint stpSlice[8] {};

static volatile int32_t stpDir[8] {};
static volatile int32_t stpPos[8] {};
static volatile int32_t stpTargetPos[8] {};
static volatile bool stpPosSet[8] {};

static repeating_timer stpTimer[8] {};


template <uint slice>
static bool stepper_timer_callback(repeating_timer* rt) {
    Stepper* stepper = static_cast<Stepper*>(rt->user_data);

    const int64_t speedFp = stepper->mSpeedFp;
    const int32_t remainingSteps = stpTargetPos[slice] - stpPos[slice];
    int32_t targetSpeedFp = stepper->mTargetSpeedFp;
    int64_t accelAmountFp = (stepper->mAccelFp * llabs(rt->delay_us)) / 1000000;

    stepper->getTimerCallback();
    if (remainingSteps * stpDir[slice] < 0) {
        if (Stepper::IsInBounds(speedFp, -accelAmountFp, accelAmountFp)) {
            stepper->setDir(stpDir[slice] > 0 ? false : true);
            stepper->startMotion(stpTargetPos[slice], stepper->mAccelFp / 1000, 1000);
            return true;
        } else {
            targetSpeedFp = 0;
        }
    } else if (remainingSteps * stpDir[slice] < stepper->mDeaccelSteps) {
        targetSpeedFp = 0;
    }

    accelAmountFp *= targetSpeedFp >= speedFp ? 1 : -1;
    int64_t changedSpeedFp = speedFp + accelAmountFp;
    changedSpeedFp = Stepper::IsInBounds(changedSpeedFp, targetSpeedFp - llabs(accelAmountFp), targetSpeedFp + llabs(accelAmountFp)) ? targetSpeedFp : changedSpeedFp;
    printf("Slice %u, Pos %ld, Speed %lld, Accel %lld\n", slice, stpPos[slice], speedFp, accelAmountFp);
    stepper->setSpeedFp(changedSpeedFp);
    return true;
}

void stepper_pwm_callback(void) {
    uint32_t irq = pwm_get_irq_status_mask();

    for (int i = 0; i < stpCount; i++) {
        const uint slice = stpSlice[i];

        if (irq & (1 << slice)) {
            // Clear the interrupt flag so the interrupt does not trigger again. 
            pwm_clear_irq(slice);

            stpPos[slice] += stpDir[slice];

            // printf("Slice %u, Pos %ld\n", slice, stpPos[slice]);
            if (stpPosSet[slice] && stpPos[slice] == stpTargetPos[slice]) {
                stpPosSet[slice] = false;
                cancel_repeating_timer(&stpTimer[slice]);
                pwm_set_enabled(slice, false);
                continue;;
            }
        }
    }
}

Stepper::Stepper(const uint pulPin, const uint dirPin, const uint32_t stepsPerRev, const uint32_t periodUs) :
    mPul(pulPin), mDir(dirPin), mSlice(pwm_gpio_to_slice_num(pulPin)), mStepsPerRev(stepsPerRev), mPeriodUs(periodUs) {
    stpSlice[stpCount] = mSlice;
    stpPos[mSlice] = 0;
    stpPosSet[mSlice] = false;
    stpDir[mSlice] = true;
    stpCount++;

    gpio_init(dirPin);
    gpio_set_dir(dirPin, true);
    gpio_put(dirPin, true);

    initPwm();

    if (get_core_num() == 1 && alarmPoolForCore1 == nullptr) {
        alarmPoolForCore1 = alarm_pool_create_with_unused_hardware_alarm(8);
    }
}

void Stepper::setTimerPeriod(const uint32_t periodUs) {
    mPeriodUs = periodUs;
    stpTimer[mSlice].delay_us = -mPeriodUs;
}

void Stepper::setAccel(const int32_t accelSteps) {
    mAccelFp = static_cast<int64_t>(accelSteps) * 1000;
}

void Stepper::setAccel(const float accelRads) {
    setAccel(radsToSteps(accelRads));
}

void Stepper::setAccelFp(const int64_t accelFp) {
    mAccelFp = accelFp;
}

void Stepper::setDeaccelSteps(const int32_t deaccelSteps) {
    mDeaccelSteps = deaccelSteps;
}


void Stepper::setPos(const int32_t currentSteps) {
    stpPos[mSlice] = currentSteps;
}

void Stepper::setPos(const float currentRads) {
    setPos(radsToSteps(currentRads));
}

int32_t Stepper::getPos() {
    return stpPos[mSlice];
}

float Stepper::getPosRads() {
    return static_cast<float>(getPos()) * 2.0f * mPi / mStepsPerRev;
}

void Stepper::setTargetPos(const int32_t targetSteps) {
    stpTargetPos[mSlice] = targetSteps;
    stpPosSet[mSlice] = true;
}

void Stepper::setTargetPos(const float targetRads) {
    setTargetPos(radsToSteps(targetRads));
}

void Stepper::setSpeed(const int32_t step) {
    setSpeedFp(step * 1000);
}

void Stepper::setSpeed(const float rad) {
    setSpeed(radsToSteps(rad));
}

void Stepper::setSpeedFp(const int64_t stepFp) {
    uint32_t wrap = mClockHz / static_cast<uint32_t>((stepFp / 1000));
    if (wrap < (0x0001 << 10)) {
        mClockDiv /= 2.0f;
        mClockHz = static_cast<uint32_t>(mSysClockHz / mClockDiv);
        pwm_set_clkdiv(mSlice, mClockDiv);
        setSpeedFp(stepFp);
    } else if (wrap > UINT16_MAX) {
        if (mClockDiv >= 255.9f) {
            wrap = UINT16_MAX;
        } else {
            mClockDiv *= 1.5f;
            mClockDiv = mClockDiv >= 255.92f ? 255.92f : mClockDiv;
            mClockHz = static_cast<uint32_t>(mSysClockHz / mClockDiv);
            pwm_set_clkdiv(mSlice, mClockDiv);
            setSpeedFp(stepFp);
        }
    }
    mWrap = wrap;
    mSpeedFp = stepFp;

    pwm_set_wrap(mSlice, mWrap);
    pwm_set_gpio_level(mPul, mWrap / 2);
}

void Stepper::setTargetSpeed(const int32_t targetSpeed) {
    setTargetSpeedFp(targetSpeed * 1000);
}

void Stepper::setTargetSpeed(const float targetSpeed) {
    setTargetSpeed(radsToSteps(targetSpeed));
}

void Stepper::setTargetSpeedFp(const int64_t targetSpeedFp) {
    mTargetSpeedFp = targetSpeedFp;
}

int32_t Stepper::getActualSpeed() {
    return mClockHz / mWrap;
}

float Stepper::getActualSpeedRads() {
    return stepsToRads(getActualSpeed());
}

void Stepper::setDir(const bool dir) {
    stpDir[mSlice] = dir ? 1 : -1;
    gpio_put(mDir, dir);
}

int Stepper::getDir() {
    return stpDir[mSlice];
}

void Stepper::enable(const bool en) {
    if (stpPosSet[mSlice]) {
        if (stpTargetPos[mSlice] == stpPos[mSlice]) {
            pwm_set_enabled(mSlice, false);
            cancel_repeating_timer(&stpTimer[mSlice]);
            return;
        } else {
            if (stpTimer[mSlice].alarm_id == 0) {
                mSpeedFp = 0;
                if (get_core_num() == 1) {
                    alarm_pool_add_repeating_timer_us(alarmPoolForCore1, -mPeriodUs, getTimerCallback(), this, &stpTimer[mSlice]);
                } else {
                    add_repeating_timer_us(-mPeriodUs, getTimerCallback(), this, &stpTimer[mSlice]);
                }
            }
        }
    }
    pwm_set_enabled(mSlice, en);
}

void Stepper::startMotion(const int32_t targetPosSteps, const int32_t accelSteps, const uint32_t timeMs, const bool start) {
    setAccel(accelSteps);
    setTargetPos(targetPosSteps);

    const int32_t targetSpeed = abs(calculateTargetSpeed(targetPosSteps - stpPos[mSlice], mSpeedFp / 1000, accelSteps, timeMs));
    setTargetSpeed(targetSpeed);

    setDeaccelSteps(getStepIncrease(targetSpeed, -abs(accelSteps), timeMs));

    enable(start);
}

void Stepper::startMotion(const float targetPosRads, const float accelRads, const float sec, const bool start) {
    startMotion(radsToSteps(targetPosRads), radsToSteps(accelRads), static_cast<uint32_t>(sec * 1000), start);
}

bool Stepper::isMoving() {
    return stpTimer[mSlice].alarm_id != 0;
}

Stepper::~Stepper() {
    pwm_set_irq_enabled(mSlice, false);
    pwm_set_enabled(mSlice, false);
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
    const uint16_t wrap { UINT16_MAX };

    pwm_config_set_clkdiv(&config, mClockDiv);
    pwm_config_set_wrap(&config, wrap);

    mClockHz = static_cast<uint32_t>(mSysClockHz / mClockDiv);

    pwm_init(mSlice, &config, false);
    pwm_set_irq_enabled(mSlice, true);
    pwm_set_enabled(mSlice, false);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, stepper_pwm_callback);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    enable(false);

    // #ifdef DEBUG
    // printf("Pwm %u => SysClockSpeed: %lu, PwmClockSpeed: %lu, ClockDiv: %.2f\n", mPul, mSysClockHz, mClockHz, mClockDiv);
    // #endif
}

int32_t Stepper::radsToSteps(const float rads) {
    return static_cast<int32_t>((rads / mPi) * (mStepsPerRev / 2));
}

float Stepper::stepsToRads(const int32_t steps) {
    return static_cast<float>((steps * 2.0f * mPi) / mStepsPerRev);
}

int32_t Stepper::calculateTargetSpeed(const int32_t deltaSteps, const int32_t initialSpeed, const int32_t accel, const uint32_t timeMs) {
    const float deltaT = timeMs / 1000.0f;
    const float a = 1;
    const float b = (2.0f * initialSpeed + deltaT * 2.0f * accel) / (-2.0f);
    const float c = (-(initialSpeed * initialSpeed) - 2 * accel * deltaSteps) / (-2.0f);

    const float disc = b * b - 4 * a * c;
    if (disc > 0) {
        float vf1 = (-b - sqrtf(disc)) / (2.0f * a);
        if (vf1 / accel > deltaT / 2.0f) {
            vf1 = (-b + sqrtf(disc)) / (2.0f * a);
        }
        if (vf1 < initialSpeed) {
            return static_cast<int32_t>((deltaSteps - initialSpeed * initialSpeed / (2.0f * accel)) / (deltaT - initialSpeed / static_cast<float>(accel)));
        } else {
            return static_cast<int32_t>(vf1);
        }
    }
    return 0;
}

int32_t Stepper::getStepIncrease(const int32_t currentSpeed, const int32_t currentAccel, const uint32_t timeMs) {
    const int64_t accelPart { static_cast<int64_t>(currentAccel) * timeMs * timeMs };
    const int64_t speedPart { static_cast<int64_t>(currentSpeed) * timeMs };
    const int32_t pos = static_cast<int32_t>((accelPart / (1000 * 1000 * 2)) + (speedPart / 1000));
    return pos;
}

int32_t Stepper::getSpeedIncrease(const int32_t currentAccel, const uint32_t timeMs) {
    const int32_t accelPart { static_cast<int32_t>(((currentAccel * timeMs) / 1000)) };
    return accelPart;
}

repeating_timer_callback_t Stepper::getTimerCallback() {
    switch (mSlice) {
    case 0:
        return stepper_timer_callback<0>;
    case 1:
        return stepper_timer_callback<1>;
    case 2:
        return stepper_timer_callback<2>;
    case 3:
        return stepper_timer_callback<3>;
    case 4:
        return stepper_timer_callback<4>;
    case 5:
        return stepper_timer_callback<5>;
    case 6:
        return stepper_timer_callback<6>;
    case 7:
        return stepper_timer_callback<7>;
    default:
        return nullptr;
    }
}
