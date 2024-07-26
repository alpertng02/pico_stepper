#ifndef __STEPPER_HPP__
#define __STEPPER_HPP__

#include "hardware/gpio.h"

// Stepper motor control class
class Stepper {
public:
    // Constructor to initialize the stepper motor with given pins and settings
    Stepper(const uint pulPin, const uint dirPin, const uint32_t stepsPerRev = 400, const uint32_t periodMs = 5);

    // Set the current position in steps
    void setPos(const int32_t currentStep);
    // Set the current position in radians
    void setPos(const float currentRads);

    // Set the target position in steps
    void setTargetPos(const int32_t targetStep);
    // Set the target position in radians
    void setTargetPos(const float targetRads);

    // Set the speed in steps per second
    void setSpeed(const int32_t step);
    // Set the speed in radians per second
    void setSpeed(const float rad);
    // Set the speed using fixed-point arithmetic
    void setSpeedFp(const int64_t stepFp);

    // Change the speed by a given number of steps per second
    void changeSpeed(const int32_t changeSteps);
    // Change the speed by a given number of radians per second
    void changeSpeed(const float changeRads);

    // Set the direction of the stepper motor
    void setDir(const bool dir);
    // Get the current direction of the stepper motor
    int getDir();

    // Enable or disable the stepper motor
    void enable(const bool en);

    // Start motion towards a target position with acceleration over a given time
    void startMotion(const int32_t targetPosSteps, const int32_t accelSteps, const uint32_t timeMs);

    // Convert steps to radians based on steps per revolution
    static constexpr float radsPerSteps(const uint32_t stepsPerRev) {
        return (mPi * 2.0f / stepsPerRev);
    }

    // Convert radians to steps based on steps per revolution
    static constexpr float stepsPerRads(const uint32_t stepsPerRev) {
        return stepsPerRev / (2.0f * mPi);
    }

    // Check if a value is within a given range
    template <typename T>
    static bool IsInBounds(const T& value, const T& low, const T& high) {
        return !(value < low) && !(high < value);
    }

    // Destructor
    ~Stepper();

private:
    // Pin for the pulse signal
    const uint mPul;
    // Pin for the direction signal
    const uint mDir;
    // PWM slice number
    const uint mSlice;

    // Convert radians to steps
    int32_t radsToSteps(const float rads);
    // Convert steps to radians
    float stepsToRads(const int32_t steps);
    // Current speed in steps per second
    int32_t mSpeed = 0;

    // Number of steps per revolution
    const uint mStepsPerRev = 400;
    // Control period in milliseconds
    const uint32_t mPeriodMs = 1;

    // Clock frequency in Hz
    uint32_t mClockHz = 125 * 1000 * 1000;
    // System clock frequency in Hz
    uint32_t mSysClockHz;
    // PWM wrap value
    uint16_t mWrap = UINT16_MAX;
    // Clock divider for PWM
    float mClockDiv = 1.0f;

    // Value of Pi
    static constexpr float mPi = 3.1415926f;

    // Initialize the PWM settings
    void initPwm();
};

#endif // __STEPPER_HPP__