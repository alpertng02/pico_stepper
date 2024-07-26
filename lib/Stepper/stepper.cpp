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
#include <cmath>

static alarm_pool* alarmPoolForCore1 { nullptr };

static volatile int stpCount = 0;

static volatile uint stpSlice[8] {};

static volatile int32_t stpDir[8] {};
static volatile int32_t stpPos[8] {};
static volatile int32_t stpTargetPos[8] {};
static volatile bool stpPosSet[8] {};

static volatile int64_t stpSpeedFp[8] {};
static volatile int64_t stpTargetSpeedFp[8] {};
static volatile int64_t stpAccelFp[8] {};

static repeating_timer stpTimer[8] {};

template <uint pul>
static bool stepper_timer_callback(repeating_timer* rt) {
    const uint slice = pwm_gpio_to_slice_num(pul);
    Stepper* stepper = static_cast<Stepper*>(rt->user_data);

    const int64_t speedFp = stpSpeedFp[slice];
    const int32_t remainingSteps = stpTargetPos[slice] - stpPos[slice];
    int64_t targetSpeedFp = stpTargetSpeedFp[slice];
    int64_t accelAmountFp = (stpAccelFp[slice] * rt->delay_us / 1000000);

    if (remainingSteps * stpDir[slice] < 0) {
        if (Stepper::IsInBounds(speedFp, -accelAmountFp, accelAmountFp)) {
            stepper->setDir(stpDir[slice] > 0 ? false : true);
            return true;
        } else {
            targetSpeedFp = 0;
        }
    }

    accelAmountFp *= targetSpeedFp < speedFp ? -1 : 1;
    int64_t changedSpeedFp = speedFp + accelAmountFp;
    changedSpeedFp = Stepper::IsInBounds(changedSpeedFp, targetSpeedFp - llabs(accelAmountFp), targetSpeedFp + llabs(accelAmountFp)) ? targetSpeedFp : changedSpeedFp;

    stepper->setSpeedFp(changedSpeedFp);
    return true;
}

static void stepper_pwm_callback() {
    uint32_t irq { pwm_get_irq_status_mask() };
    for (int i = 0; i < stpCount; i++) {
        const uint slice = stpSlice[i];

        if (irq & (1 << slice)) {
            // Clear the interrupt flag so the interrupt does not trigger again. 
            pwm_clear_irq(slice);

            stpPos[slice] += stpDir[slice];

            if (stpPos[slice] == stpTargetPos[slice] && stpPosSet[slice]) {
                stpPosSet[slice] = false;
                pwm_set_enabled(slice, false);
                cancel_repeating_timer(&stpTimer[slice]);
                continue;;
            }
        }
    }
};


/**
 * @brief Constructor for the Stepper class.
 * 
 * This constructor initializes a Stepper object with the specified parameters.
 * 
 * @param pulPin The GPIO pin number for the PUL pin.
 * @param dirPin The GPIO pin number for the DIR pin.
 * @param stepsPerRev The number of steps per revolution for the stepper motor.
 * @param periodMs The period in milliseconds for the stepper motor movement.
 */
Stepper::Stepper(const uint pulPin, const uint dirPin, const uint32_t stepsPerRev, const uint32_t periodMs) :
    mPul(pulPin), mDir(dirPin), mSlice(pwm_gpio_to_slice_num(pulPin)), mStepsPerRev(stepsPerRev), mPeriodMs(periodMs) {
    stpSlice[stpCount] = mSlice;
    stpPos[mSlice] = 0;
    stpDir[mSlice] = true;
    stpCount++;

    gpio_init(dirPin);
    gpio_set_dir(dirPin, true);
    gpio_put(dirPin, true);

    initPwm();

    if (get_core_num() == 1) {
        if (alarmPoolForCore1 == nullptr) {
            alarmPoolForCore1 = alarm_pool_create_with_unused_hardware_alarm(8);
        }
        alarm_pool_add_repeating_timer_ms(alarmPoolForCore1, -periodMs, stepper_timer_callback<0>, this, &stpTimer[mSlice]);
    } else {
        add_repeating_timer_ms(-periodMs, stepper_timer_callback<0>, this, &stpTimer[mSlice]);
    }
}

/**
 * Starts the motion of the stepper motor.
 *
 * @param targetPosSteps The target position in steps.
 * @param accelSteps The acceleration in steps per second squared.
 * @param timeMs The time in milliseconds.
 */
void Stepper::startMotion(const int32_t targetPosSteps, const int32_t accelSteps, const uint32_t timeMs) {
    setTargetPos(targetPosSteps);

    stpAccelFp[mSlice] = static_cast<int64_t>(accelSteps) * 1000;

    if (stpTimer[mSlice].alarm_id == 0) {
        if (get_core_num() == 1) {
            alarm_pool_add_repeating_timer_ms(alarmPoolForCore1, -mPeriodMs, stepper_timer_callback<0>, this, &stpTimer[mSlice]);
        } else {
            add_repeating_timer_ms(-mPeriodMs, stepper_timer_callback<0>, this, &stpTimer[mSlice]);
        }
    }

}

/**
 * @brief Sets the current position of the stepper motor.
 *
 * This function sets the current position of the stepper motor to the specified number of steps.
 *
 * @param currentSteps The number of steps to set as the current position.
 */
void Stepper::setPos(const int32_t currentSteps) {
    stpPos[mSlice] = currentSteps;
}

/**
 * @brief Sets the position of the stepper motor in radians.
 * 
 * This function sets the position of the stepper motor using the specified angle in radians.
 * It internally converts the angle to steps and then calls the `setPos` function with the step value.
 * 
 * @param currentRads The desired position of the stepper motor in radians.
 */
void Stepper::setPos(const float currentRads) {
    setPos(radsToSteps(currentRads));
}

/**
 * Sets the target position of the stepper motor in steps.
 * 
 * @param targetSteps The target position in steps.
 */
void Stepper::setTargetPos(const int32_t targetSteps) {
    stpTargetPos[mSlice] = targetSteps;
    stpPosSet[mSlice] = true;
}


/**
 * Sets the target position of the stepper motor in radians.
 * 
 * @param targetRads The target position in radians.
 */
void Stepper::setTargetPos(const float targetRads) {
    setTargetPos(radsToSteps(targetRads));
}

/**
 * @brief Sets the speed of the stepper motor.
 *
 * This function sets the speed of the stepper motor by calculating the appropriate clock division
 * and wrap values based on the desired step value. It adjusts the clock division and wrap values
 * recursively until the desired speed is achieved.
 *
 * @param step The desired step value.
 */
void Stepper::setSpeed(const int32_t step) {
    uint32_t wrap = mClockHz / step;
    if (wrap < (0x0001 << 10)) {
        mClockDiv /= 2.0f;
        mClockHz = static_cast<uint32_t>(mSysClockHz / mClockDiv);
        pwm_set_clkdiv(mSlice, mClockDiv);
        setSpeed(step);
    } else if (wrap > UINT16_MAX) {
        if (mClockDiv >= 255.9f) {
            wrap = UINT16_MAX;
        } else {
            mClockDiv *= 1.5f;
            mClockDiv = mClockDiv >= 255.9f ? 255.9f : mClockDiv;
            mClockHz = static_cast<uint32_t>(mSysClockHz / mClockDiv);
            pwm_set_clkdiv(mSlice, mClockDiv);
            setSpeed(step);
        }
    }
    mWrap = wrap;
    mSpeed = mClockHz / mWrap;
    stpSpeedFp[mSlice] = mSpeed * 1000;

    pwm_set_wrap(mSlice, mWrap);
    pwm_set_gpio_level(mPul, mWrap / 2);
}

/**
 * Sets the speed of the stepper motor in radians per second.
 * 
 * @param rad The desired speed of the stepper motor in radians per second.
 */
void Stepper::setSpeed(const float rad) {
    setSpeed(radsToSteps(rad));
}

/**
 * Sets the speed of the stepper motor in fixed-point format.
 *
 * This function calculates the appropriate clock frequency and clock divider
 * based on the desired step frequency in fixed-point format. It adjusts the
 * clock divider and clock frequency to ensure that the step frequency falls
 * within the acceptable range.
 *
 * @param stepFp The desired step frequency in fixed-point format.
 */
void Stepper::setSpeedFp(const int64_t stepFp) {
    uint32_t wrap = static_cast<uint32_t>(mClockHz / (stepFp / 1000));
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
            mClockDiv = mClockDiv >= 255.9f ? 255.9f : mClockDiv;
            mClockHz = static_cast<uint32_t>(mSysClockHz / mClockDiv);
            pwm_set_clkdiv(mSlice, mClockDiv);
            setSpeedFp(stepFp);
        }
    }
    mWrap = wrap;
    mSpeed = mClockHz / mWrap;
    stpSpeedFp[mSlice] = stepFp;

    pwm_set_wrap(mSlice, mWrap);
    pwm_set_gpio_level(mPul, mWrap / 2);
}

/**
 * Changes the speed of the stepper motor by the specified number of steps.
 *
 * @param changeSteps The number of steps to change the speed by.
 */
void Stepper::changeSpeed(const int32_t changeSteps) {
    setSpeed(mSpeed + changeSteps);
}

/**
 * Changes the speed of the stepper motor.
 *
 * This function takes a change in radians per second and converts it to the corresponding change in steps per second.
 * It then calls the `changeSpeed` function with the calculated change in steps per second.
 *
 * @param changeRads The change in radians per second.
 */
void Stepper::changeSpeed(const float changeRads) {
    changeSpeed(radsToSteps(changeRads));
}

/**
 * @brief Sets the direction of the stepper motor.
 *
 * This function sets the direction of the stepper motor by updating the `stpDir` array and
 * setting the corresponding GPIO pin.
 *
 * @param dir The direction of the stepper motor. `true` for forward, `false` for backward.
 */
void Stepper::setDir(const bool dir) {
    stpDir[mSlice] = dir ? 1 : -1;
    gpio_put(mDir, dir);
}

/**
 * @brief Get the direction of the stepper motor.
 * 
 * @return int The direction of the stepper motor.
 */
int Stepper::getDir() {
    return stpDir[mSlice];
}

/**
 * Enables or disables the stepper motor.
 *
 * @param en A boolean value indicating whether to enable or disable the stepper motor.
 */
void Stepper::enable(const bool en) {
    if (stpPosSet[mSlice]) {
        if (stpTargetPos[mSlice] == stpPos[mSlice]) {
            pwm_set_enabled(mSlice, false);
            return;
        } else {
            if (stpTargetPos[mSlice] > stpPos[mSlice]) {
                setDir(true);
            } else {
                setDir(false);
            }
        }
    }
    pwm_set_enabled(mSlice, en);
}

/**
 * Destructor for the Stepper class.
 * Disables the PWM interrupt and deinitializes the pulse (PUL) and direction (DIR) GPIO pins.
 */
Stepper::~Stepper() {
    pwm_set_irq_enabled(mSlice, false);
    gpio_deinit(mPul);
    gpio_deinit(mDir);
}

/**
 * @brief Initializes the PWM configuration for the stepper motor.
 * 
 * This function sets up the PWM functionality for the stepper motor by configuring the GPIO pin,
 * setting the clock division and wrap values, and enabling the PWM interrupt. It also initializes
 * the PWM slice with the provided configuration.
 * 
 * @note This function assumes that the `mPul` member variable has been properly set to the GPIO pin
 *       used for the PWM functionality.
 * 
 * @note This function assumes that the `mSysClockHz` and `mClockDiv` member variables have been properly
 *       set to the system clock frequency and the desired clock division value, respectively.
 * 
 * @param None
 * @return None
 */
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

    irq_set_exclusive_handler(PWM_IRQ_WRAP, stepper_pwm_callback);
    pwm_set_irq_enabled(mSlice, true);

    pwm_init(mSlice, &config, false);
#ifdef DEBUG
    printf("Pwm %u => ClockSpeed: %u ClockDiv: %.2f\n", mPul, mSysClockHz, mClockDiv);
#endif
}

/**
 * Converts radians to steps.
 *
 * This function takes a value in radians and converts it to the corresponding number of steps
 * based on the stepper motor's configuration.
 *
 * @param rads The value in radians to be converted.
 * @return The number of steps corresponding to the given value in radians.
 */
int32_t Stepper::radsToSteps(const float rads) {
    return static_cast<int32_t>((rads / mPi) * (mStepsPerRev / 2));
}

/**
 * Converts the given number of steps to radians.
 *
 * @param steps The number of steps to convert.
 * @return The equivalent value in radians.
 */
float Stepper::stepsToRads(const int32_t steps) {
    return static_cast<float>((steps * 2.0f * mPi) / mStepsPerRev);
}

