/**
 * cd ~/Desktop/pico-lib/build
 * make
 *
 * cp ~/Desktop/pico-lib/build/main.uf2 /media/iker/RPI-RP2/main.uf2
*/
#include <cstdint>
#include <string>

#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico-lib/gpio.h"
#include "lib/software/bitmask.h"
#include "lib/software/pid.h"
#include "lib/software/hex.h"
#include "lib/hardware/motor.h"
#include "lib/hardware/USB_Serial.h"
#include "lib/hardware/compass_classes.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

#define SDA 4
#define SCL 5
#define ARDUINO_I2C 8
#define START_BUTTON 21

#define LDR 28

std::string message = "";
int yaw_offset = 0;
bool disabled = false;

int main () {
    stdio_init_all();

    USB_Serial_Init();

    Motor::begin();

    adc_init();
    adc_gpio_init(LDR);
    adc_select_input(2);

    pinMode(BUILTIN_LED, OUTPUT);
    pinMode(START_BUTTON, INPUT);
    digitalWrite(BUILTIN_LED, HIGH);

    Adafruit_BNO055 bno;
    bno.begin(I2C_PORT_0, 100 * 1000, SDA, SCL);

    unsigned long ldr_time = millis();
    unsigned long last_time = millis();
    unsigned long tick_time = millis();

    bool state = false;
    std::string message;

    for (;;) {
        if (millis() - last_time >= 500) {
            digitalWrite(BUILTIN_LED, state);
            state = !state;
            last_time = millis();
        }

        if (digitalRead(START_BUTTON)) {
            yaw_offset = static_cast<int>(bno.getYaw());
        }

        if (USB_Serial_Available()) {
            char byte = USB_Serial_Get_Byte();
            if (byte == '\n') {
                uint64_t custom_bitmask = hex::decode(message);
                Bitmask::setBitmask(custom_bitmask);
                int yaw = static_cast<int>(bno.getYaw());
                yaw = yaw - yaw_offset;
                if (yaw < 0) {
                    yaw += 360;
                }
                if (yaw > 360) {
                    yaw -= 360;
                }

                Bitmask::setBitmaskValue(yaw, static_cast<int>(Bitmask::COMPASS_ADDRESS::YAW));
                std::string bitmask = hex::encode(Bitmask::generateBitmask());
                bitmask += "\n";
                USB_Serial_Write(reinterpret_cast<const uint8_t *>(bitmask.c_str()), bitmask.length());
                message = "";
            } else {
                message += byte;
            }
        }


        if (millis() - tick_time >= 10) {
            Motor::tick();

            int s_speed = Bitmask::bitmaskToValue(static_cast<int>(Bitmask::MOTOR_ADDRESS::S_SPEED));
            int ne_speed = Bitmask::bitmaskToValue(static_cast<int>(Bitmask::MOTOR_ADDRESS::NE_SPEED));
            int nw_speed = Bitmask::bitmaskToValue(static_cast<int>(Bitmask::MOTOR_ADDRESS::NW_SPEED));

            s_speed *= Bitmask::bitmaskToValue(static_cast<int>(Bitmask::MOTOR_ADDRESS::S_DIR)) ? 1 : -1;
            ne_speed *= Bitmask::bitmaskToValue(static_cast<int>(Bitmask::MOTOR_ADDRESS::NE_DIR)) ? 1 : -1;
            nw_speed *= Bitmask::bitmaskToValue(static_cast<int>(Bitmask::MOTOR_ADDRESS::NW_DIR)) ? 1 : -1;

            if (s_speed == 0 && ne_speed == 0 && nw_speed == 0) {
                Motor::stop();
                tick_time = millis();
            }

//            std::cout << "S: " << Bitmask::bitmaskToValue(static_cast<int>(Bitmask::MOTOR_ADDRESS::S_SPEED)) << std::endl;
//            std::cout << "NE: " << Bitmask::bitmaskToValue(static_cast<int>(Bitmask::MOTOR_ADDRESS::NE_SPEED)) << std::endl;
//            std::cout << "NW: " << Bitmask::bitmaskToValue(static_cast<int>(Bitmask::MOTOR_ADDRESS::NW_SPEED)) << std::endl;

            Motor::moveS(s_speed);
            Motor::moveNE(ne_speed);
            Motor::moveNW(nw_speed);
            tick_time = millis();
        }
    }
}

#pragma clang diagnostic pop
