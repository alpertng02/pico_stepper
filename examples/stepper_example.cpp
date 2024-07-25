#include "pico/stdlib.h"
#include <cstdio>

#include "stepper.hpp"

int main() {

    Stepper stepper(2, 3, 400);
    
    bool stepperDir = true;
    stepper.enable(true);
    while (true) {

        for (int32_t i = 0; i < 5000; i++) {
            stepper.setSpeed(i);
            sleep_ms(1);
        }
        for (int32_t i = 5000; i > 0; i++) {
            stepper.setSpeed(i);
            sleep_ms(1);
        }
        stepperDir = !stepperDir;
        stepper.setDir(stepperDir);
    }
}