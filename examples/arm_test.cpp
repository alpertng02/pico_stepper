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

#include "pico/stdlib.h"
#include <cmath>
#include <cstdio>

#include "stepper.hpp"

class RoboticArm {
public:
    RoboticArm(uint pul1 = 2, uint dir1 = 3, uint pul2 = 6, uint dir2 = 7, uint pul3 = 8, uint dir3 = 9) :
        stepper {
            { pul1, dir1, stepsPerRev[0], timerPeriodsMs[0] },
            { pul2, dir2, stepsPerRev[1], timerPeriodsMs[1] },
            { pul3, dir3, stepsPerRev[2], timerPeriodsMs[2] } }, armRads(getArmRads(initPx, initPy, initT1)) {
        for(int i = 0; i < 3; i++) {
            stepper[i].setPos(armRads.theta[i]);
        }
    }

    void move(const float px, const float py, const float theta, const float sec, const float gain = 1.0f) { 
        const auto rads = getArmRads(px, py, theta);
        for (int i = 0; i < 3; i++) {
            stepper[i].startMotion(rads.theta[i], (rads.theta[i] - stepper[i].getPosRads()) * gain / sec, sec);
        }
    }

    void setCurrentPosition(const float px, const float py, const float theta) {
        armRads = getArmRads(px, py, theta);
        for (int i = 0; i < 3; i++) {
            stepper[i].setPos(armRads.theta[i]);
        }
    }

private:
    struct Rads {
        float theta[3] {};
    };

    Stepper stepper[3] {
        { 2, 3, 400, timerPeriodsMs[0] },
        { 6, 7, 400, timerPeriodsMs[1] },
        { 8, 9, 2000, timerPeriodsMs[2] }
    };
    RoboticArm::Rads armRads {};

    static constexpr float gearRatio[3] { 50.0f, 68.181818f, 2.f };
    static constexpr uint32_t stepsPerRev[3] { 400, 400, 2000 };
    static constexpr uint32_t timerPeriodsMs[3] { 3, 3, 3 };
    static constexpr float initPx { 500.0f };
    static constexpr float initPy { -580.0f };
    static constexpr float initT1 { 0 };

    RoboticArm::Rads getArmRads(const float px, const float py, const float theta) {
        constexpr int a1 = 580;
        constexpr int a2 = 500;

        const float cost3 = (
            px * px
            + py * py
            - a1 * a1
            - a2 * a2
            ) / (2 * a1 * a2);

        const float sint3 = -1 * sqrtf(1 - cost3 * cost3);
        // t3 = stepper 1;
        const float t3 = atan2f(sint3, cost3);

        const float cosft3 = cosf(t3);
        const float sinft3 = sinf(t3);

        const float detpart1 = ((a1 + a2 * cosft3));
        const float detpart2 = ((a2 * sinft3));
        const float determinant = detpart1 * detpart1 + detpart2 * detpart2;

        const float cost2 = px * (a1 + a2 * cosft3)
            - (-py * a2 * sinft3);

        const float sint2 = (py * (a1 + a2 * cosft3))
            - (px * a2 * sinft3);

        // t2 = stepper 2;
        const float t2 = atan2f(sint2 / determinant, cost2 / determinant);

        return  { t2 * gearRatio[0], (-t3 - t2) * gearRatio[1], theta * gearRatio[2] };
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
        float sec {};
        scanf("%f %f %f %f", &px, &py, &theta, &sec);

        arm.move(px, py, theta, sec);

        sleep_ms(10);
    }
}