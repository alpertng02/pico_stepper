#ifndef __STEPPER_HPP__
#define __STEPPER_HPP__

#include "hardware/gpio.h"

class Stepper {
public:

    Stepper(const uint pulPin, const uint dirPin, const uint32_t stepsPerRev = 400, const uint32_t periodMs = 5);

    void setPos(const int32_t currentStep);
    void setPos(const float currentRads);

    void setTargetPos(const int32_t targetStep);
    void setTargetPos(const float targetRads);

    void setSpeed(const int32_t step);
    void setSpeed(const float rad);
    void setSpeedFp(const int64_t stepFp);

    void changeSpeed(const int32_t changeSteps);
    void changeSpeed(const float changeRads);

    void setDir(const bool dir);
    int getDir();

    void enable(const bool en);

    void startMotion(const int32_t targetPosSteps, const int32_t accelSteps, const uint32_t timeMs);

    static constexpr float radsPerSteps(const uint32_t stepsPerRev) {
        return (mPi * 2.0f / stepsPerRev);
    }

    static constexpr float stepsPerRads(const uint32_t stepsPerRev) {
        return stepsPerRev / (2.0f * mPi);
    }
    template <typename T>
    static bool IsInBounds(const T& value, const T& low, const T& high) {
        return !(value < low) && !(high < value);
    }

    ~Stepper();

private:
    const uint mPul;
    const uint mDir;
    const uint mSlice;

    int32_t radsToSteps(const float rads);
    float stepsToRads(const int32_t steps);
    int32_t mSpeed = 0;

    const uint mStepsPerRev = 400;
    const uint32_t mPeriodMs = 1;

    uint32_t mClockHz = 125 * 1000 * 1000;
    uint32_t mSysClockHz;
    uint16_t mWrap = UINT16_MAX;
    float mClockDiv = 1.0f;

    static constexpr float mPi = 3.1415926f;

    void initPwm();

};
#endif // __STEPPER_HPP__