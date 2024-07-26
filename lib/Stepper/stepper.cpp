#include "stepper.hpp"

#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "pico/time.h"
#include "pico/platform.h"

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

    const int32_t remainingSteps = stpTargetPos[slice] - stpPos[slice];
    int64_t targetSpeedFp = stpTargetSpeedFp[slice];
    int64_t accelAmountFp = (stpAccelFp[slice] * rt->delay_us / 1000000);

    if (remainingSteps * stpDir[slice] < 0) {
        if (Stepper::IsInBounds(stpSpeedFp[slice], accelAmountFp, -accelAmountFp)) {
            stepper->setDir(stpDir[slice] > 0 ? false : true);
            return true;
        } else {
            targetSpeedFp = 0;
        }
    }

    accelAmountFp *= targetSpeedFp < stpSpeedFp[slice] ? -1 : 1;
    int64_t changedSpeedFp = stpSpeedFp[slice] + accelAmountFp;
    changedSpeedFp = Stepper::IsInBounds(changedSpeedFp, targetSpeedFp + accelFp, targetSpeedFp - accelFp) ? targetSpeedFp : changedSpeedFp;

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
    stpSpeedFp[mSlice] = mSpeed * 1000;

    pwm_set_wrap(mSlice, mWrap);
    pwm_set_gpio_level(mPul, mWrap / 2);
}

void Stepper::setSpeed(const float rad) {
    setSpeed(radsToSteps(rad));
}

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

int Stepper::getDir() {
    return stpDir[mSlice];
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

