//
// Created by kali on 2/13/24.
//

#include "../time.h"

void delay(unsigned long ms) {
    sleep_ms(ms);
}

unsigned long millis() {
    return time_us_64() / 1000;
}

unsigned long micros() {
    return time_us_64();
}

void delayMicroseconds(unsigned int us) {
    sleep_us(us);
}
