/**
 * cd ~/Desktop/pico-lib/build
 * make
 *
 * cp ~/Desktop/pico-lib/build/main.uf2 /media/iker/RPI-RP2/main.uf2
*/

#include "pico/stdlib.h"
#include "pico-lib/i2c.h"
#include "pico-lib/usb.h"
#include "pico-lib/gpio.h"
#include "lib/hardware/motor.h"
#include "lib/hardware/compass_classes.h"
#include "lib/software/bitmask.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

#define SDA 4
#define SCL 5
#define ARDUINO_I2C 0x30

#define KICKER 23

int main () {
    stdio_init_all();
    sleep_ms(5000);
    USB::print << "Starting main thread\n";

    pinMode(KICKER, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, HIGH);

    Motor::begin();

    Adafruit_BNO055 compass;
    compass.begin(I2C_PORT_0, 1000 * 100, SDA, SCL);

    for (;;) {
    }
}

#pragma clang diagnostic pop