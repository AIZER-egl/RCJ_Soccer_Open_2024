/**
 * cd ~/Desktop/pico-lib/build
 * make
 *
 * cp ~/Desktop/pico-lib/build/main.uf2 /media/iker/RPI-RP2/main.uf2
*/

#include "pico/stdlib.h"
#include "lib/hardware/motor.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

Motor::Motor motor;
int main () {
    stdio_init_all();
}

#pragma clang diagnostic pop
