//
// Created by iker on 14/01/24.
//

#ifndef PICO_LIB_TIME_H
#define PICO_LIB_TIME_H

#include "pico/time.h"

void delay(unsigned long ms);
unsigned long millis();
unsigned long micros();
void delayMicroseconds(unsigned int us);

#endif //PICO_LIB_TIME_H
