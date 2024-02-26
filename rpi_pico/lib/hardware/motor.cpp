//
// Created by kali on 2/7/24.
//

#include "motor.h"

volatile unsigned long pulses_A_N;
volatile unsigned long pulses_B_N;
volatile unsigned long pulses_A_SW;
volatile unsigned long pulses_B_SW;
volatile unsigned long pulses_A_SE;
volatile unsigned long pulses_B_SE;

namespace Motor {
    Motor motor;

    void individualMotor::callbackA(unsigned int gpio, unsigned long events) {
        // For whatever reason, trying to use pulsesA variable inside the individualMotor structure, won't work.
        // Therefore, I will be using global variables to solve it
        switch (gpio) {
            case MOTOR_N_ENC_A:
                pulses_A_N++;
                break;
            case MOTOR_SW_ENC_A:
                pulses_A_SW++;
                break;
            case MOTOR_SE_ENC_A:
                pulses_A_SE++;
                break;
        }
    }

    void individualMotor::callbackB(unsigned int gpio, unsigned long events) {
        switch (gpio) {
            case MOTOR_N_ENC_B:
                pulses_B_N++;
                break;
            case MOTOR_SW_ENC_B:
                pulses_B_SW++;
                break;
            case MOTOR_SE_ENC_B:
                pulses_B_SE++;
                break;
        }
    }

    void individualMotor::getRPM_A() {
        unsigned long pulsesA;
        switch (id) {
            case MOTOR_N:
                pulsesA = pulses_A_N;
                break;
            case MOTOR_SW:
                pulsesA = pulses_A_SW;
                break;
            case MOTOR_SE:
                pulsesA = pulses_A_SE;
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

    void individualMotor::getRPM_B() {
        unsigned long pulsesB;
        switch (id) {
            case MOTOR_N:
                pulsesB = pulses_B_N;
                break;
            case MOTOR_SW:
                pulsesB = pulses_B_SW;
                break;
            case MOTOR_SE:
                pulsesB = pulses_B_SE;
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
        motor.motorN.id = MOTOR_N;
        motor.motorSE.id = MOTOR_SE;
        motor.motorSW.id = MOTOR_SW;
        gpio_init(MOTOR_N_DIR);
        gpio_set_dir(MOTOR_N_DIR, GPIO_OUT);
        gpio_set_function(MOTOR_N_PWM, GPIO_FUNC_PWM);
        gpio_set_dir(MOTOR_N_ENC_A, GPIO_IN);
        gpio_set_dir(MOTOR_N_ENC_B, GPIO_IN);

        gpio_set_irq_enabled_with_callback (
                MOTOR_N_ENC_A,
                GPIO_IRQ_EDGE_RISE,
                true,
                [] (unsigned int gpio, unsigned long events) {
                    static_cast <individualMotor *> (&motor.motorN) -> callbackA(gpio, events);
                }
        );

        gpio_set_irq_enabled_with_callback(
                MOTOR_N_ENC_B,
                GPIO_IRQ_EDGE_RISE,
                true,
                [] (unsigned int gpio, unsigned long events) {
                    static_cast <individualMotor *> (&motor.motorN) -> callbackB(gpio, events);
                }
        );

        gpio_init(MOTOR_SW_DIR);
        gpio_set_dir(MOTOR_SW_DIR, GPIO_OUT);
        gpio_set_function(MOTOR_SW_PWM, GPIO_FUNC_PWM);
        gpio_set_dir(MOTOR_SW_ENC_A, GPIO_IN);
        gpio_set_dir(MOTOR_SW_ENC_B, GPIO_IN);

        gpio_set_irq_enabled_with_callback (
                MOTOR_SW_ENC_A,
                GPIO_IRQ_EDGE_RISE,
                true,
                [] (unsigned int gpio, unsigned long events) {
                    static_cast <individualMotor *> (&motor.motorSW) -> callbackA(gpio, events);
                }
        );

        gpio_set_irq_enabled_with_callback(
                MOTOR_SW_ENC_B,
                GPIO_IRQ_EDGE_RISE,
                true,
                [] (unsigned int gpio, unsigned long events) {
                    static_cast <individualMotor *> (&motor.motorSW) -> callbackB(gpio, events);
                }
        );

        gpio_init(MOTOR_SE_DIR);
        gpio_set_dir(MOTOR_SE_DIR, GPIO_OUT);
        gpio_set_function(MOTOR_SE_PWM, GPIO_FUNC_PWM);
        gpio_set_dir(MOTOR_SE_ENC_A, GPIO_IN);
        gpio_set_dir(MOTOR_SE_ENC_B, GPIO_IN);

        gpio_set_irq_enabled_with_callback (
                MOTOR_SE_ENC_A,
                GPIO_IRQ_EDGE_RISE,
                true,
                [] (unsigned int gpio, unsigned long events) {
                    static_cast <individualMotor *> (&motor.motorSE) -> callbackA(gpio, events);
                }
        );

        gpio_set_irq_enabled_with_callback(
                MOTOR_SE_ENC_B,
                GPIO_IRQ_EDGE_RISE,
                true,
                [] (unsigned int gpio, unsigned long events) {
                    static_cast <individualMotor *> (&motor.motorSE) -> callbackB(gpio, events);
                }
        );

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

        motorN(motor.rotatePID.output);
        motorSE(motor.rotatePID.output);
        motorSW(motor.rotatePID.output);
    }

    void move(int16_t speed, int16_t direction, int16_t facing) {
        int motorNSpeed = speed * sin ((0 + direction) * (PI / 180));
        int motorSESpeed = speed * sin ((120 + direction) * (PI / 180));
        int motorSWSpeed = speed * sin ((240 + direction) * (PI / 180));

        int maxSpeed = std::max({std::abs(motorNSpeed), std::abs(motorSESpeed), std::abs(motorSWSpeed)});

        motorNSpeed *= (speed / maxSpeed);
        motorSESpeed *= (speed / maxSpeed);
        motorSWSpeed *= (speed / maxSpeed);

        // FIXME: This is a placeholder for the PID controller, should be replaced with real BNO055 data
        motor.movePID.error = facing - 0;
        motor.movePID.target = facing;
        motor.pid.compute(motor.movePID);

        motorN(motorNSpeed + motor.movePID.output);
        motorSE(motorSESpeed + motor.movePID.output);
        motorSW(motorSWSpeed + motor.movePID.output);
    }

    void motorN(int16_t speed) {
        USB::print << "   speed: " << speed << "    ";
        std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

        uint slice_num = pwm_gpio_to_slice_num(MOTOR_N_PWM);
        pwm_set_wrap(slice_num, 255);
        pwm_set_chan_level(slice_num, PWM_CHAN_A + (MOTOR_N_PWM % 2), std::abs(speed));
        pwm_set_enabled(slice_num, true);

        gpio_set_dir(MOTOR_N_DIR, GPIO_OUT);
        gpio_put(MOTOR_N_DIR, speed > 0);
        motor.motorN.speed = std::abs(speed);
    }

    void motorSW(int16_t speed) {
        std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

        uint slice_num = pwm_gpio_to_slice_num(MOTOR_SW_PWM);
        pwm_set_wrap(slice_num, 255);
        pwm_set_chan_level(slice_num, PWM_CHAN_A + (MOTOR_SW_PWM % 2), std::abs(speed));
        pwm_set_enabled(slice_num, true);

        gpio_set_dir(MOTOR_SW_DIR, GPIO_OUT);
        gpio_put(MOTOR_SW_DIR, speed > 0);
        motor.motorSW.speed = std::abs(speed);
    }

    void motorSE(int16_t speed) {
        std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

        uint slice_num = pwm_gpio_to_slice_num(MOTOR_SE_PWM);
        pwm_set_wrap(slice_num, 255);
        pwm_set_chan_level(slice_num, PWM_CHAN_A + (MOTOR_SE_PWM % 2), std::abs(speed));
        pwm_set_enabled(slice_num, true);

        gpio_set_dir(MOTOR_SE_DIR, GPIO_OUT);
        gpio_put(MOTOR_SE_DIR, speed > 0);
        motor.motorSE.speed = std::abs(speed);
    }
}