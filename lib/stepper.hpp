/**
 *@file stepper.hpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief Linear stepper motor driver for Raspberry Pi Pico
 * @version 0.1
 * @date 2024-07-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __STEPPER_HPP__
#define __STEPPER_HPP__

#ifdef STEPPER_DEBUG_LOG_ENABLED
#define STEPPER_DEBUG_LOG
#endif


#include "pico/time.h"

/**
 * @brief Stepper class for controlling a stepper motor using a stepper driver.
 *
 *  This class allows non blocking control of a stepper motor using linear speed
 * profile. \n  The non-blocking control is achieved by using a repeating timer
 * callback function that calculates the speed of the stepper motor based on the
 * desired speed, acceleration, and deceleration and travel duration\n . \n  The
 * repeating timer interrupt runs on the same core that creates the class
 * instance so that multiple stepper instances can be controlled from seperate
 * cores\n. \n  The position of the stepper motor is kept track using
 * PMW_WRAP_IRQ functionality of RP2040. \n Due to the nature of PWM_WRAP_IRQ,
 * each stepper pulse pin should be connected to a seperate PWM slice which
 * means a maximum of 8 steppers can be controlled.
 *
 *  All function related to speed, position and acceleration control have
 * argument overloads for steps and radian units. \n  Functions ending in Fp are
 * integer step units scaled by 1000. This is to avoid floating point
 * calculations in interrupts.
 *
 *
 */
class Stepper {
public:
  /**
   * @brief Constructor for the Stepper class.
   *
   * This constructor initializes a Stepper object with the specified
   * parameters.
   *
   * @param pulPin The GPIO pin number for the PUL pin.
   * @param dirPin The GPIO pin number for the DIR pin.
   * @param stepsPerRev The number of steps per revolution for the stepper
   * motor.
   */
  Stepper(const uint pulPin, const uint dirPin,
          const uint32_t stepsPerRev = 400);

 
  /**
   * @brief Sets the speed of the stepper motor.
   *
   * This function sets the speed of the stepper motor by calculating the
   * appropriate clock division and wrap values based on the desired step value.
   * It adjusts the clock division and wrap values recursively until the desired
   * speed is achieved.
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
   * @brief Set the steps in a single revolution of stepper motor.
   * 
   * @param steps 
   */
  void setStepsPerRev(const uint32_t steps);
  
  /**
   * @brief Gets the actual speed of the stepper motor.
   *
   * This function returns the current speed of the stepper motor in steps per
   * second.
   *
   * @return The actual speed of the stepper motor in steps per second.
   */
  int32_t getActualSpeed();

  /**
   * @brief Gets the actual speed of the stepper motor in radians per second.
   *
   * @return The actual speed of the stepper motor in radians per second.
   */
  float getActualSpeedRads();

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
   * The motor will move towards this target position when the `step` function
   * is called.
   *
   * @param targetRads The target position in radians.
   */
  void setTargetPos(const float targetRads);

  /**
   * @brief Sets the direction of the stepper motor.
   *
   * This function sets the direction of the stepper motor by updating the
   * `stpDir` array and setting the corresponding GPIO pin.
   *
   * @param dir The direction of the stepper motor. `true` for forward, `false`
   * for backward.
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
   * @param en A boolean value indicating whether to enable or disable the
   * stepper motor.
   */
  void enable(const bool en);

  /**
   * @brief Sets the current position of the stepper motor.
   *
   * This function sets the current position of the stepper motor to the
   * specified number of steps.
   *
   * @param currentSteps The number of steps to set as the current position.
   */
  void setPos(const int32_t currentSteps);

  /**
   * @brief Sets the position of the stepper motor in radians.
   *
   * This function sets the position of the stepper motor using the specified
   * angle in radians. It internally converts the angle to steps and then calls
   * the `setPos` function with the step value.
   *
   * @param currentRads The desired position of the stepper motor in radians.
   */
  void setPos(const float currentRads);

  /**
   * @brief Gets the current position of the stepper motor.
   *
   * @return The current position of the stepper motor as a 32-bit signed
   * integer.
   */
  int32_t getPos();

  /**
   * @brief Get the current position of the stepper motor in radians.
   *
   * @return The current position of the stepper motor in radians.
   */
  float getPosRads();

  /**
   * @brief Checks if the stepper motor is currently moving.
   *
   * @return true if the stepper motor is moving, false otherwise.
   */
  bool isMoving();

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
   * Destructor for the Stepper class.
   * Disables the PWM interrupt and deinitializes the pulse (PUL) and direction
   * (DIR) GPIO pins.
   */
  ~Stepper();

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
  static bool IsInBounds(const T &value, const T &low, const T &high) {
    return !(value < low) && !(high < value);
  }

private:
  // Pin for the pulse signal
  const uint mPul;
  // Pin for the direction signal
  const uint mDir;
  // PWM slice number
  const uint mSlice;

  // Number of steps per revolution
  uint mStepsPerRev = 400;

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

  /**
   * Initializes the PWM (Pulse Width Modulation) for the stepper motor.
   * This function sets up the necessary configurations and registers to enable
   * PWM control.
   */
  void initPwm();

  /**
   * Converts radians to steps.
   *
   * This function takes a floating-point value representing an angle in radians
   * and converts it to the corresponding number of steps.
   *
   * @param rads The angle in radians to be converted.
   * @return The number of steps corresponding to the given angle.
   */
  int32_t radsToSteps(const float rads);

  /**
   * @brief Converts the given number of steps to radians.
   *
   * This function takes an integer value representing the number of steps and
   * converts it to radians. The result is returned as a floating-point value.
   *
   * @param steps The number of steps to convert to radians.
   * @return The equivalent value in radians.
   */
  float stepsToRads(const int32_t steps);

};
#endif // __STEPPER_HPP__