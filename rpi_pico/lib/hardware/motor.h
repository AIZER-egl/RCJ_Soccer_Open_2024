//
// Created by kali on 2/7/24.
//

#ifndef PICO_LIB_MOTOR_H
#define PICO_LIB_MOTOR_H

#include <cmath>
#include <algorithm>
#include <cstdint>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "../software/pid.h"
#include "../../pico-lib/time.h"
#include "../../pico-lib/usb.h"

#define MOTOR_S_DIR 2
#define MOTOR_S_PWM 3
#define MOTOR_S_ENC_A 6
#define MOTOR_S_ENC_B 7

#define MOTOR_NW_DIR 8
#define MOTOR_NW_PWM 9
#define MOTOR_NW_ENC_A 10
#define MOTOR_NW_ENC_B 11

#define MOTOR_NE_DIR 12
#define MOTOR_NE_PWM 13
#define MOTOR_NE_ENC_A 14
#define MOTOR_NE_ENC_B 15

#define DRIBBLER_DIR 12
#define DRIBBLER_PWM 13

#define MOTOR_S 0b0000
#define MOTOR_NE 0b0010
#define MOTOR_NW 0b0001

#define PULSES_PER_REVOLUTION 48
#define GEAR_RATIO 9.68
#define INTERVAL 1000000

#define MAX_SPEED 255
#define MIN_SPEED 25

#define PI 3.141592

namespace Motor {
    struct individualMotor {
        uint8_t id;
        int16_t speed;
        float rpmA;
        float rpmB;

        unsigned long previousPulsesA;
        unsigned long previousPulsesB;

        unsigned long previousTimeA;
        unsigned long previousTimeB;

        void callbackA (unsigned int gpio, unsigned long events);
        void callbackB (unsigned int gpio, unsigned long events);

        void getRPM_A ();
        void getRPM_B ();
    };

    struct Motor {
        individualMotor motorS;
        individualMotor motorNE;
        individualMotor motorNW;
        PID pid;
        PID::PidParameters movePID;
        PID::PidParameters rotatePID;
    };

    extern Motor motor;

    void begin ();

    void stop ();

    void motorS (int16_t speed);
    void motorNE (int16_t speed);
    void motorNW (int16_t speed);

    void move (int16_t speed, int16_t direction, int16_t facing);

    void rotate (int16_t angle);

    void dribbler(int16_t speed);
}


#endif //PICO_LIB_MOTOR_H
