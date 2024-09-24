#include <pico/multicore.h>
#include <pico/stdlib.h>
#include <stdio.h>

#include "stepper.hpp"
#include <bitset>

void core1Main() {
  printf("Entered core1\n");
  Stepper stepperCore1(6, 7, 400, 5);

  int32_t pos = 0, accel = 0;
  uint32_t timeMs = 0;

  while (true) {
    pos = static_cast<int32_t>(multicore_fifo_pop_blocking());
    accel = static_cast<int32_t>(multicore_fifo_pop_blocking());
    timeMs = multicore_fifo_pop_blocking();
    printf("Received Core1 pos %ld, accel %ld, timeMs %lu\n", pos, accel,
           timeMs);

    if (!stepperCore1.startMotion(pos, accel, timeMs)) {
      printf("Error, core1 movement is not possible!\n");
    }
    int32_t currentPos = stepperCore1.getPos();
    printf("StepperCore1 currentPos %ld\n", currentPos);
  }
}

int main() {

  stdio_init_all();
  sleep_ms(1000);

  Stepper stepperCore0(2, 3, 400, 5);
  multicore_launch_core1(core1Main);

  int i = 0;
  int32_t pos = 0, accel = 0;
  uint32_t timeMs = 0;
  while (true) {
    scanf("%d %ld %ld %lu", &i, &pos, &accel, &timeMs);
    if (i == 1) {
      bool res = true;
      constexpr uint64_t timeoutUs = 1000 * 1000;
      res &= multicore_fifo_push_timeout_us(pos, timeoutUs);
      res &= multicore_fifo_push_timeout_us(accel, timeoutUs);
      res &= multicore_fifo_push_timeout_us(timeMs, timeoutUs);
      if (!res) {
        printf("Could not send data to core1 through fifo, timeout reached!\n");
      }
    } else if (i == 0) {
      if (!stepperCore0.startMotion(pos, accel, timeMs)) {
        printf("Error, core0 movement is not possible!\n");
      }
    } else {
      printf("Error, Core index does not exist!\n");
    }
    int32_t currentPos = stepperCore0.getPos();
    printf("stepperCore0 currentPos %ld\n", currentPos);
  }
}
