#include "pico/stdlib.h"
#include <cstdio>

#include "stepper.hpp"

int main() {

    stdio_init_all();
    sleep_ms(1000);
    Stepper stepper(2, 3, 400, 10);
    
    printf("Enter =>  pos, speed, accel:\n");
    while (true) {
        int32_t pos {}, speed {}, accel {};
        scanf("%ld %ld %ld", &pos, &speed, &accel);

        stepper.setAccel(accel);
        stepper.setTargetSpeed(speed);
        stepper.setTargetPos(pos);

        stepper.enable(true);

        sleep_ms(10);
    }
}