#include "pico/stdlib.h"
#include <cstdio>

#include "stepper.hpp"

int main() {

    stdio_init_all();
    sleep_ms(1000);
    Stepper stepper[2] { { 2, 3, 400, 10000 }, { 6, 7, 400, 5000 } };

    printf("Enter =>  index pos accel ms:\n");
    while (true) {
        int i {};
        int32_t pos {}, accel {};
        uint32_t ms {};
        scanf("%d %ld %ld %lu", &i, &pos, &accel, &ms);

        stepper[i].startMotion(pos, accel, ms);
        sleep_ms(10);
    }
}