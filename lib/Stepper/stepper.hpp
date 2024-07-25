#ifndef __STEPPER_HPP__
#define __STEPPER_HPP__

#include "hardware/gpio.h"

class Stepper {
public:

    Stepper(const uint pulPin, const uint dirPin, const uint32_t stepsPerRev);

    void setPos(const int32_t currentStep);
    void setPos(const float currentRads);

    void setTargetPos(const int32_t targetStep);
    void setTargetPos(const float targetRads);

    void setSpeed(const int32_t step);
    void setSpeed(const float rad);

    void changeSpeed(const int32_t changeSteps);
    void changeSpeed(const float changeRads);

    void setDir(const bool dir);

    void enable(const bool en);

    static constexpr int32_t radsToSteps(const float rads, const uint32_t stepsPerRev) {
        return static_cast<int32_t>((rads / mPi) * (stepsPerRev / 2));
    }

    static constexpr float stepsToRads(const int32_t steps, const uint32_t stepsPerRev) {
        return static_cast<float>((steps * 2.0f * mPi) / stepsPerRev);
    }
    ~Stepper();

protected:
    int32_t radsToSteps(const float rads);
    float stepsToRads(const int32_t steps);
    int32_t mSpeed = 0;

private:
    const uint mPul;
    const uint mDir;
    const uint mSlice;


    const uint mStepsPerRev = 400;

    uint32_t mClockHz = 125 * 1000 * 1000;
    uint32_t mSysClockHz;
    uint16_t mWrap = UINT16_MAX;
    float mClockDiv = 1.0f;

    static constexpr float mPi = 3.1415926f;

    void initPwm();

};
#endif // __STEPPER_HPP__