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

#include <pico/stdlib.h>
#include <cstdio>

#include "stepper.hpp"

int main() {

    stdio_init_all();
    sleep_ms(1000);
    Stepper stepper(8, 9, 1600);

    printf("Enter => pos speed:\n");
    while (true) {
        // int i {};
        int32_t pos {}, speed {};
        
        scanf("%ld %ld", &pos, &speed);
        stepper.setTargetPos(pos);
        stepper.setSpeed(speed);
        stepper.enable(true);

        sleep_ms(10);
    }
}