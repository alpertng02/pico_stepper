/**
 *@file arm_test.cpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-07-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <pico/stdlib.h>

#include "stepper.hpp"

class RoboticArm {
public:
  RoboticArm(uint pul1 = 2, uint dir1 = 3, uint pul2 = 6, uint dir2 = 7,
             uint pul3 = 8, uint dir3 = 9)
      : stepper{Stepper{pul1, dir1, stepsPerRev[0], timerPeriodsMs[0]},
                Stepper{pul2, dir2, stepsPerRev[1], timerPeriodsMs[1]},
                Stepper{pul3, dir3, stepsPerRev[2], timerPeriodsMs[2]}},
        armRads(getArmRads(initPx, initPy, initT1)) {
    for (int i = 0; i < 3; i++) {
      stepper[i].setPos(armRads[i]);
    }
  }

  void move(const float px, const float py, const float theta, const float sec,
            const float gain = 2.0f) {
    const auto rads = getArmRads(px, py, theta);
    for (int i = 0; i < 3; i++) {
      float accel = fabs(rads[i] - stepper[i].getPosRads()) * gain;
      stepper[i].startMotion(rads[i],
                             std::clamp(accel, 0.f, maxAccels[i]), sec);
    }
  }

  void setCurrentPosition(const float px, const float py, const float theta) {
    armRads = getArmRads(px, py, theta);
    for (int i = 0; i < 3; i++) {
      stepper[i].setPos(armRads[i]);
    }
  }

private:
  
  std::array<Stepper, 3> stepper{
      Stepper{2, 3, stepsPerRev[0], timerPeriodsMs[0]},
      Stepper{6, 7, stepsPerRev[1], timerPeriodsMs[1]},
      Stepper{8, 9, stepsPerRev[2], timerPeriodsMs[2]}};
  std::array<float,3> armRads{};

  static constexpr std::array gearRatio{50.0f, 68.181818f, 2.f};
  static constexpr std::array<uint32_t, 3> stepsPerRev{400, 400, 2000};
  static constexpr std::array<uint32_t, 3> timerPeriodsMs{3, 3, 3};
  static constexpr float maxAccels[3]{
      gearRatio[0] * stepsPerRev[0] * Stepper::radsPerSteps(stepsPerRev[0]),
      gearRatio[1] * stepsPerRev[1] * Stepper::radsPerSteps(stepsPerRev[1]),
      gearRatio[2] * stepsPerRev[2] * Stepper::radsPerSteps(stepsPerRev[2])};

  static constexpr float initPx{500.0f};
  static constexpr float initPy{-500.0f};
  static constexpr float initT1{0};

  std::array<float,3> getArmRads(const float px, const float py,
                              const float theta) {
    constexpr int a1 = 500;
    constexpr int a2 = 500;

    const float cost3 = (px * px + py * py - a1 * a1 - a2 * a2) / (2 * a1 * a2);

    const float sint3 = -1 * sqrtf(1 - cost3 * cost3);
    // t3 = stepper 1;
    const float t3 = atan2f(sint3, cost3);

    const float cosft3 = cosf(t3);
    const float sinft3 = sinf(t3);

    const float detpart1 = ((a1 + a2 * cosft3));
    const float detpart2 = ((a2 * sinft3));
    const float determinant = detpart1 * detpart1 + detpart2 * detpart2;

    const float cost2 = px * (a1 + a2 * cosft3) - (-py * a2 * sinft3);

    const float sint2 = (py * (a1 + a2 * cosft3)) - (px * a2 * sinft3);

    // t2 = stepper 2;
    const float t2 = atan2f(sint2 / determinant, cost2 / determinant);

    return {t2 * gearRatio[0], (-t3 - t2) * gearRatio[1], theta * gearRatio[2]};
  }
};

int main() {
  stdio_init_all();
  sleep_ms(1000);

  RoboticArm arm{};

  printf("Enter Initial Position =>  px py theta:\n");
  float px = 500.0f, py = -300.0f, theta = 0.0f;
  scanf("%f %f %f", &px, &py, &theta);
  arm.setCurrentPosition(px, py, theta);

  printf("Enter New Position => px py theta sec:\n");
  while (true) {
    float sec{};
    scanf("%f %f %f %f", &px, &py, &theta, &sec);

    arm.move(px, py, theta, sec);

    sleep_ms(10);
  }
}