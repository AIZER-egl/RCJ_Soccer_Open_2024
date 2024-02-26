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

#define MOTOR_N_DIR 7
#define MOTOR_N_PWM 6
#define MOTOR_N_ENC_A 16
#define MOTOR_N_ENC_B 17


#define MOTOR_SW_DIR 4
#define MOTOR_SW_PWM 5
#define MOTOR_SW_ENC_A 6
#define MOTOR_SW_ENC_B 7

#define MOTOR_SE_DIR 8
#define MOTOR_SE_PWM 9
#define MOTOR_SE_ENC_A 10
#define MOTOR_SE_ENC_B 11

#define MOTOR_N 0b0000
#define MOTOR_SE 0b0010
#define MOTOR_SW 0b0001

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
        individualMotor motorN;
        individualMotor motorSE;
        individualMotor motorSW;
        // todo: add pid
        PID pid;
        PID::PidParameters movePID;
        PID::PidParameters rotatePID;
    };

    extern Motor motor;

    void begin ();

    void motorN (int16_t speed);
    void motorSE (int16_t speed);

    void motorSW (int16_t speed);
    void move (int16_t speed, int16_t direction, int16_t facing);

    void rotate (int16_t angle);
}


#endif //PICO_LIB_MOTOR_H
