//
// Created by kali on 2/7/24.
//

#include "motor.h"

volatile unsigned long pulses_A_S;
volatile unsigned long pulses_B_S;
volatile unsigned long pulses_A_NW;
volatile unsigned long pulses_B_NW;
volatile unsigned long pulses_A_NE;
volatile unsigned long pulses_B_NE;

// FIXME: Pulses A *DOES NOT* gives the rpm, but the direction

namespace Motor {
    Motor motor;

    void individualMotor::callbackA(unsigned int gpio, unsigned long events) {
        // For whatever reason, trying to use pulsesA variable inside the individualMotor structure, won't work.
        // Therefore, I will be using global variables to solve it
        switch (gpio) {
            case MOTOR_S_ENC_A:
                pulses_A_S++;
                break;
            case MOTOR_NW_ENC_A:
                pulses_A_NW++;
                break;
            case MOTOR_NE_ENC_A:
                pulses_A_NE++;
                break;
        }
    }

    void individualMotor::callbackB(unsigned int gpio, unsigned long events) {
        switch (gpio) {
            case MOTOR_S_ENC_B:
                pulses_B_S++;
                break;
            case MOTOR_NW_ENC_B:
                pulses_B_NW++;
                break;
            case MOTOR_NE_ENC_B:
                pulses_B_NE++;
                break;
        }
    }

    void individualMotor::getRPM_A() {
        unsigned long pulsesA;
        switch (id) {
            case MOTOR_S:
                pulsesA = pulses_A_S;
                break;
            case MOTOR_NW:
                pulsesA = pulses_A_NW;
                break;
            case MOTOR_NE:
                pulsesA = pulses_A_NE;
                break;
        }
        unsigned long currentTime = micros();
        int timeDifference = currentTime - previousTimeA;
        int pulseDifference = std::abs(static_cast<int>(pulsesA - previousPulsesA));

        float rpm = (pulseDifference * INTERVAL / timeDifference) / (PULSES_PER_REVOLUTION * GEAR_RATIO);

        previousPulsesA = pulsesA;
        previousTimeA = currentTime;

        rpmA = rpm;
    }

    void stop () {
        motorS(0);
        motorNE(0);
        motorNW(0);
    }

    void individualMotor::getRPM_B() {
        unsigned long pulsesB;
        switch (id) {
            case MOTOR_S:
                pulsesB = pulses_B_S;
                break;
            case MOTOR_NW:
                pulsesB = pulses_B_NW;
                break;
            case MOTOR_NE:
                pulsesB = pulses_B_NE;
                break;
        }

        unsigned long currentTime = micros();
        int timeDifference = currentTime - previousTimeB;
        int pulseDifference = std::abs(static_cast<int>(pulsesB - previousPulsesB));
        float rpm = ((pulseDifference * INTERVAL / timeDifference) / (PULSES_PER_REVOLUTION * GEAR_RATIO)) * 60;

        previousPulsesB = pulsesB;
        previousTimeB = currentTime;

        rpmB = rpm;
    }

    void begin() {
        motor.motorS.id = MOTOR_S;
        motor.motorNE.id = MOTOR_NE;
        motor.motorNW.id = MOTOR_NW;
        gpio_init(MOTOR_S_DIR);
        gpio_set_dir(MOTOR_S_DIR, GPIO_OUT);
        gpio_set_function(MOTOR_S_PWM, GPIO_FUNC_PWM);
        gpio_set_dir(MOTOR_S_ENC_A, GPIO_IN);
        gpio_set_dir(MOTOR_S_ENC_B, GPIO_IN);

        gpio_set_irq_enabled_with_callback (
                MOTOR_S_ENC_A,
                GPIO_IRQ_EDGE_RISE,
                true,
                [] (unsigned int gpio, unsigned long events) {
                    static_cast <individualMotor *> (&motor.motorS) -> callbackA(gpio, events);
                }
        );

        gpio_set_irq_enabled_with_callback(
                MOTOR_S_ENC_B,
                GPIO_IRQ_EDGE_RISE,
                true,
                [] (unsigned int gpio, unsigned long events) {
                    static_cast <individualMotor *> (&motor.motorS) -> callbackB(gpio, events);
                }
        );

        gpio_init(MOTOR_NW_DIR);
        gpio_set_dir(MOTOR_NW_DIR, GPIO_OUT);
        gpio_set_function(MOTOR_NW_PWM, GPIO_FUNC_PWM);
        gpio_set_dir(MOTOR_NW_ENC_A, GPIO_IN);
        gpio_set_dir(MOTOR_NW_ENC_B, GPIO_IN);

        gpio_set_irq_enabled_with_callback (
                MOTOR_NW_ENC_A,
                GPIO_IRQ_EDGE_RISE,
                true,
                [] (unsigned int gpio, unsigned long events) {
                    static_cast <individualMotor *> (&motor.motorNW) -> callbackA(gpio, events);
                }
        );

        gpio_set_irq_enabled_with_callback(
                MOTOR_NW_ENC_B,
                GPIO_IRQ_EDGE_RISE,
                true,
                [] (unsigned int gpio, unsigned long events) {
                    static_cast <individualMotor *> (&motor.motorNW) -> callbackB(gpio, events);
                }
        );

        gpio_init(MOTOR_NE_DIR);
        gpio_set_dir(MOTOR_NE_DIR, GPIO_OUT);
        gpio_set_function(MOTOR_NE_PWM, GPIO_FUNC_PWM);
        gpio_set_dir(MOTOR_NE_ENC_A, GPIO_IN);
        gpio_set_dir(MOTOR_NE_ENC_B, GPIO_IN);

        gpio_set_irq_enabled_with_callback (
                MOTOR_NE_ENC_A,
                GPIO_IRQ_EDGE_RISE,
                true,
                [] (unsigned int gpio, unsigned long events) {
                    static_cast <individualMotor *> (&motor.motorNE) -> callbackA(gpio, events);
                }
        );

        gpio_set_irq_enabled_with_callback(
                MOTOR_NE_ENC_B,
                GPIO_IRQ_EDGE_RISE,
                true,
                [] (unsigned int gpio, unsigned long events) {
                    static_cast <individualMotor *> (&motor.motorNE) -> callbackB(gpio, events);
                }
        );

        gpio_init(DRIBBLER_DIR);
        gpio_set_dir(DRIBBLER_DIR, GPIO_OUT);
        gpio_set_function(DRIBBLER_PWM, GPIO_FUNC_PWM);

        motor.movePID.maxOutput = MAX_SPEED;
        motor.movePID.minOutput = 0;
        motor.movePID.maxError = 1000.0;
        motor.movePID.errorThresholdPermission = 3.0;
        motor.movePID.kp = 0.0;
        motor.movePID.ki = 0.0;
        motor.movePID.kd = 0.0;

        motor.rotatePID.maxOutput = MAX_SPEED;
        motor.rotatePID.minOutput = MIN_SPEED;
        motor.rotatePID.maxError = 1000.0;
        motor.rotatePID.errorThresholdPermission = 7.5;
        motor.rotatePID.kp = 0.0;
        motor.rotatePID.ki = 0.0;
        motor.rotatePID.kd = 0.0;

    }

    void rotate(int16_t angle) {
        // FIXME: This is a placeholder for the rotate controller, should be replaced with real BNO055 data
        uint16_t currentAngle = 0;
        uint16_t target = angle;

        target = fmod(target + 360 - currentAngle, 360);
        currentAngle = 0;

        motor.rotatePID.error = target - currentAngle;
        motor.rotatePID.target = target;
        motor.pid.compute(motor.rotatePID);

        motorS(motor.rotatePID.output);
        motorNE(motor.rotatePID.output);
        motorNW(motor.rotatePID.output);
    }

    void move(int16_t speed, int16_t direction, int16_t facing) {
        int motorSSpeed = speed * sin ((0 + direction) * (PI / 180));
        int motorNESpeed = speed * sin ((120 + direction) * (PI / 180));
        int motorNWSpeed = speed * sin ((240 + direction) * (PI / 180));

        int maxSpeed = std::max({std::abs(motorSSpeed), std::abs(motorNESpeed), std::abs(motorNWSpeed)});

        motorSSpeed *= (speed / maxSpeed);
        motorNESpeed *= (speed / maxSpeed);
        motorNWSpeed *= (speed / maxSpeed);

        // FIXME: This is a placeholder for the PID controller, should be replaced with real BNO055 data
        motor.movePID.error = facing - 0;
        motor.movePID.target = facing;
        motor.pid.compute(motor.movePID);

        motorS(motorSSpeed + motor.movePID.output);
        motorNE(-(motorNESpeed + motor.movePID.output));
        motorNW(motorNWSpeed + motor.movePID.output);
    }

    void motorS(int16_t speed) {
        std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

        uint slice_num = pwm_gpio_to_slice_num(MOTOR_S_PWM);
        pwm_set_wrap(slice_num, 255);
        pwm_set_chan_level(slice_num, PWM_CHAN_A + (MOTOR_S_PWM % 2), std::abs(speed));
        pwm_set_enabled(slice_num, true);

        gpio_set_dir(MOTOR_S_DIR, GPIO_OUT);
        gpio_put(MOTOR_S_DIR, speed > 0);
        motor.motorS.speed = std::abs(speed);
    }

    void motorNW(int16_t speed) {
        std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

        uint slice_num = pwm_gpio_to_slice_num(MOTOR_NW_PWM);
        pwm_set_wrap(slice_num, 255);
        pwm_set_chan_level(slice_num, PWM_CHAN_A + (MOTOR_NW_PWM % 2), std::abs(speed));
        pwm_set_enabled(slice_num, true);

        gpio_set_dir(MOTOR_NW_DIR, GPIO_OUT);
        gpio_put(MOTOR_NW_DIR, speed > 0);
        motor.motorNW.speed = std::abs(speed);
    }

    void motorNE(int16_t speed) {
        std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

        uint slice_num = pwm_gpio_to_slice_num(MOTOR_NE_PWM);
        pwm_set_wrap(slice_num, 255);
        pwm_set_chan_level(slice_num, PWM_CHAN_A + (MOTOR_NE_PWM % 2), std::abs(speed));
        pwm_set_enabled(slice_num, true);

        gpio_set_dir(MOTOR_NE_DIR, GPIO_OUT);
        gpio_put(MOTOR_NE_DIR, speed < 0);
        motor.motorNE.speed = std::abs(speed);
    }

    void dribbler(int16_t speed) {
        std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));
        uint slice_num = pwm_gpio_to_slice_num(DRIBBLER_PWM);
        pwm_set_wrap(slice_num, 255);
        pwm_set_chan_level(slice_num, PWM_CHAN_A + (DRIBBLER_PWM % 2), std::abs(speed));
        pwm_set_enabled(slice_num, true);

        gpio_set_dir(DRIBBLER_DIR, GPIO_OUT);
        gpio_put(DRIBBLER_DIR, speed > 0);
    }
}