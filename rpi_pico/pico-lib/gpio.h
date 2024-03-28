// Library for Raspberry pico GPIO 1.0.0 by @IkerCs / @aizer-egl
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include <string>
#include "./usb.h"

#ifndef GPIO_H
#define GPIO_H

#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
#define OUTPUT_PWM 4
#define HIGH 1
#define LOW 0
#define BUILTIN_LED 25

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
void analogWrite(uint8_t pin, uint8_t value);
void toggleFor(unsigned long ms, uint8_t pin);
uint8_t digitalRead(uint8_t pin);
uint16_t analogRead(uint8_t pin);

#endif