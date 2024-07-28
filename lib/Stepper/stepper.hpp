#ifndef __STEPPER_HPP__
#define __STEPPER_HPP__

#include "hardware/gpio.h"

/**
 * @brief Stepper class for controlling a stepper motor.
 *
 * This class provides functions to control a stepper motor using the Raspberry Pi Pico.
 * It uses the PUL and DIR pins to control the stepper motor.
 *
 * The class provides functions to change the speed of the stepper motor, set the target position,
 * set the direction, enable or disable the stepper motor, and start the motion of the stepper motor.
 *
 * The class also provides functions to set the current position of the stepper motor in steps or radians.
 */
class Stepper {
public:
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
    Stepper(const uint pulPin, const uint dirPin, const uint32_t stepsPerRev = 400, const uint32_t periodMs = 5);
    
    /**
     * Sets the acceleration steps for the stepper motor.
     *
     *   This function sets the acceleration factor for the stepper motor. The acceleration factor
     * determines how quickly the stepper motor reaches its maximum speed. A higher acceleration
     * factor will result in a faster acceleration, while a lower acceleration factor will result
     * in a slower acceleration.
     *
     * @param accelSteps The number of acceleration steps to set.
     */
    void setAccel(const int32_t accelSteps);

    /**
     * Sets the acceleration of the stepper motor.
     *
     * @param accelRads The acceleration value in radians per second squared.
     */
    void setAccel(const float accelRads);

    /**
     * @brief Sets the acceleration factor for the stepper motor.
     *
     * @param accelFp The acceleration factor to set, specified as a fixed-point number.
     */
    void setAccelFp(const int64_t accelFp);

    /**
      * @brief Sets the speed of the stepper motor.
      *
      * This function sets the speed of the stepper motor by calculating the appropriate clock division
      * and wrap values based on the desired step value. It adjusts the clock division and wrap values
      * recursively until the desired speed is achieved.
      *
      * @param step The desired step value.
    */
    void setSpeed(const int32_t step);

    /**
     * Sets the speed of the stepper motor in radians per second.
     *
     * @param rad The desired speed of the stepper motor in radians per second.
     */
    void setSpeed(const float rad);

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
    void setSpeedFp(const int64_t stepFp);

    void setTargetSpeed(const int32_t targetSpeed);
    void setTargetSpeed(const float targetSpeed);
    void setTargetSpeedFp(const int64_t targetSpeedFp);

    /**
     * Sets the target position of the stepper motor in steps.
     *
     * @param targetSteps The target position in steps.
     */
    void setTargetPos(const int32_t targetSteps);

    
    /**
     * @brief Sets the target position for the stepper motor.
     *
     * This function sets the target position for the stepper motor in radians.
     * The motor will move towards this target position when the `step` function is called.
     *
     * @param targetRads The target position in radians.
     */
    void setTargetPos(const float targetRads);

    /**
     * @brief Sets the direction of the stepper motor.
     *
     * This function sets the direction of the stepper motor by updating the `stpDir` array and
     * setting the corresponding GPIO pin.
     *
     * @param dir The direction of the stepper motor. `true` for forward, `false` for backward.
     */
    void setDir(const bool dir);

    /**
     * @brief Get the direction of the stepper motor.
     *
     * @return int The direction of the stepper motor.
     */
    int getDir();

    /**
     * Enables or disables the stepper motor.
     *
     * @param en A boolean value indicating whether to enable or disable the stepper motor.
     */
    void enable(const bool en);

    /**
     * Starts the motion of the stepper motor.
     *
     * @param targetPosSteps The target position in steps.
     * @param accelSteps The acceleration in steps per second squared.
     * @param timeMs The time in milliseconds.
     */
    void startMotion(const int32_t targetPosSteps, const int32_t accelSteps, const uint32_t timeMs);

    /**
     * @brief Sets the current position of the stepper motor.
     *
     * This function sets the current position of the stepper motor to the specified number of steps.
     *
     * @param currentSteps The number of steps to set as the current position.
     */
    void setPos(const int32_t currentSteps);

    /**
     * @brief Sets the position of the stepper motor in radians.
     *
     * This function sets the position of the stepper motor using the specified angle in radians.
     * It internally converts the angle to steps and then calls the `setPos` function with the step value.
     *
     * @param currentRads The desired position of the stepper motor in radians.
     */
    void setPos(const float currentRads);


    /**
     * @brief Gets the current position of the stepper motor.
     *
     * @return The current position of the stepper motor as a 32-bit signed integer.
     */
    int32_t getPos();

    /**
     * @brief Get the current position of the stepper motor in radians.
     *
     * @return The current position of the stepper motor in radians.
     */
    float getPosRads();

    /**
     * Calculates the radians per step for a given number of steps per revolution.
     *
     * @param stepsPerRev The number of steps per revolution.
     * @return The radians per step.
     */
    static constexpr float radsPerSteps(const uint32_t stepsPerRev) {
        return (mPi * 2.0f / stepsPerRev);
    }

    /**
     * Converts the number of steps per revolution to steps per radian.
     *
     * @param stepsPerRev The number of steps per revolution.
     * @return The equivalent number of steps per radian.
     */
    static constexpr float stepsPerRads(const uint32_t stepsPerRev) {
        return stepsPerRev / (2.0f * mPi);
    }

    /**
     * Checks if a value is within a specified range.
     *
     * @tparam T The type of the value and range boundaries.
     * @param value The value to check.
     * @param low The lower bound of the range.
     * @param high The upper bound of the range.
     * @return True if the value is within the range, false otherwise.
     */
    template <typename T>
    static bool IsInBounds(const T& value, const T& low, const T& high) {
        return !(value < low) && !(high < value);
    }

    /**
     * Destructor for the Stepper class.
     * Disables the PWM interrupt and deinitializes the pulse (PUL) and direction (DIR) GPIO pins.
     */
    ~Stepper();

private:
    // Pin for the pulse signal
    const uint mPul;
    // Pin for the direction signal
    const uint mDir;
    // PWM slice number
    const uint mSlice;

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
    // Convert radians to steps
    int32_t radsToSteps(const float rads);
    // Convert steps to radians
    float stepsToRads(const int32_t steps);
};
#endif // __STEPPER_HPP__