/**
 *@file stepper_example.cpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-07-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "pico/stdlib.h"
#include <cstdio>

#include "stepper.hpp"

int main() {

    stdio_init_all();
    sleep_ms(1000);
    Stepper stepper[2] { { 2, 3, 400, 10 }, { 6, 7, 400, 5 } };

    printf("Enter =>  index pos accel ms:\n");
    while (true) {
        int i {};
        int32_t pos {}, accel {}, speed {};
        uint32_t ms {};
        scanf("%d %ld %ld %lu", &i, &pos, &accel, &ms);

        if (!stepper[i].startMotion(pos, accel, ms)) {
            printf("Error: Given trajectory was not possible in the motion duration!\n");
        } else {
            printf("Stepper %d started to move to %ld position with %ld acceleration in %lu ms\n", i, pos, accel, ms);
        }
        sleep_ms(10);
    }
}