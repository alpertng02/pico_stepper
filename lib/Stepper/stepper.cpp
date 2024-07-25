#include "stepper.hpp"

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

static volatile int stpCount = 0;

static volatile uint stpSlice[8] {};

static volatile int32_t stpDir[8] {};
static volatile int32_t stpPos[8] {};
static volatile int32_t stpTargetPos[8] {};
static volatile bool stpPosSet[8] {};

static void stepper_pwm_callback() {
    uint32_t irq { pwm_get_irq_status_mask() };

    for (int i = 0; i < stpCount; i++) {
        const uint slice = stpSlice[i];

        if (irq & (1 << slice)) {
            // Clear the interrupt flag so the interrupt does not trigger again. 
            pwm_clear_irq(slice);

            stpPos[slice] += stpDir[slice];

            if (stpPosSet[slice]) {
                if (stpPos[slice] == stpTargetPos[slice]) {
                    stpPosSet[slice] = false;
                    pwm_set_enabled(slice, false);
                    continue;;
                }
            }
        }
    }
};

Stepper::Stepper(const uint pulPin, const uint dirPin, const uint32_t stepsPerRev = 400) :
    mPul(pulPin), mDir(dirPin), mSlice(pwm_gpio_to_slice_num(pulPin)), mStepsPerRev(stepsPerRev) {
    stpSlice[stpCount] = mSlice;
    stpPos[stpCount] = 0;
    stpDir[stpCount] = true;
    stpCount++;

    gpio_init(dirPin);
    gpio_set_dir(dirPin, true);
    gpio_put(dirPin, true);

    initPwm();
}

void Stepper::setPos(const int32_t currentStep) {
    stpPos[mSlice] = currentStep;
}

void Stepper::setPos(const float currentRads) {
    setPos(radsToSteps(currentRads));
}

void Stepper::setTargetPos(const int32_t targetStep) {
    stpTargetPos[mSlice] = targetStep;
    stpPosSet[mSlice] = true;
}

void Stepper::setTargetPos(const float targetRads) {
    setTargetPos(radsToSteps(targetRads));
}

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

    pwm_set_wrap(mSlice, mWrap);
    pwm_set_gpio_level(mPul, mWrap / 2);
}


void Stepper::setSpeed(const float rad) {
    setSpeed(radsToSteps(rad));
}


void Stepper::changeSpeed(const int32_t changeSteps) {
    setSpeed(mSpeed + changeSteps);
}

void Stepper::changeSpeed(const float changeRads) {
    changeSpeed(radsToSteps(changeRads));
}

void Stepper::setDir(const bool dir) {
    stpDir[mSlice] = dir ? 1 : -1;
    gpio_put(mDir, dir);
}

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

Stepper::~Stepper() {
    pwm_set_irq_enabled(mSlice, false);
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

    irq_set_exclusive_handler(PWM_IRQ_WRAP, stepper_pwm_callback);
    pwm_set_irq_enabled(mSlice, true);

    pwm_init(mSlice, &config, false);
#ifdef DEBUG
    printf("Pwm %u => ClockSpeed: %u ClockDiv: %.2f\n", mPul, mSysClockHz, mClockDiv);
#endif
}

int32_t Stepper::radsToSteps(const float rads) {
    return static_cast<int32_t>((rads / mPi) * (mStepsPerRev / 2));
}

float Stepper::stepsToRads(const int32_t steps) {
    return static_cast<float>((steps * 2.0f * mPi) / mStepsPerRev);
}

