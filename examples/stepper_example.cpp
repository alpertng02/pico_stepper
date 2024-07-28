#include "pico/stdlib.h"
#include <cstdio>

#include "stepper.hpp"

int main() {

    stdio_init_all();
    sleep_ms(1000);
    Stepper stepper[2] { { 2, 3, 400, 10 }, { 6, 7, 400, 5 } };

    printf("Enter =>  pos, speed, accel:\n");
    while (true) {
        int i {};
        int32_t pos {}, speed {}, accel {};
        scanf("%d %ld %ld %ld", &i, &pos, &speed, &accel);

        stepper[i].setAccel(accel);
        stepper[i].setTargetSpeed(speed);
        stepper[i].setTargetPos(pos);

        stepper[i].enable(true);

        sleep_ms(10);
    }
}