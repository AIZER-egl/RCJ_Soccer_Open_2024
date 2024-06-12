//
// Created by kali on 2/7/24.
//

#include "motor.h"

volatile float pulses_A_S;
volatile float pulses_B_S;
volatile float pulses_A_NW;
volatile float pulses_B_NW;
volatile float pulses_A_NE;
volatile float pulses_B_NE;

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

        float rpm = pulsesB * 1000 / 20 * 60 / 464.64;

        // When the motor is stalled, the rpm should be 0 but there is a mechanical state where it is in front of the sensor generating a absurdly high rpm value
        if (rpm <= 350) {
            rpmB = rpm;
        }

        switch (id) {
            case MOTOR_S:
                pulses_B_S = 0;
                break;
            case MOTOR_NW:
                pulses_B_NW = 0;
                break;
            case MOTOR_NE:
                pulses_B_NE = 0;
                break;
        }
    }

    void stop () {
        if (motor.motorS.speed == 0 && motor.motorNE.speed == 0 && motor.motorNW.speed == 0) return;
        motorS(motor.motorS.speed > 0 ? -MAX_SPEED : MAX_SPEED);
        motorNE(motor.motorNE.speed > 0 ? -MAX_SPEED : MAX_SPEED);
        motorNW(motor.motorNW.speed > 0 ? -MAX_SPEED : MAX_SPEED);
        delay(25);
        motorS(0);
        motorNE(0);
        motorNW(0);
    }

    void tick() {
        motor.motorS.getRPM_A();
        motor.motorS.getRPM_B();
        motor.motorNE.getRPM_A();
        motor.motorNE.getRPM_B();
        motor.motorNW.getRPM_A();
        motor.motorNW.getRPM_B();
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

        motor.movePID.maxOutput = 150;
        motor.movePID.minOutput = 0;
        motor.movePID.maxError = 1000.0;
        motor.movePID.errorThresholdPermission = 3.0;
        motor.movePID.kp = 0.0;
        motor.movePID.ki = 0.0;
        motor.movePID.kd = 0.0;

        motor.rotatePID.maxOutput = 150;
        motor.rotatePID.minOutput = 0;
        motor.rotatePID.maxError = 1000.0;
        motor.rotatePID.errorThresholdPermission = 7.5;
        motor.rotatePID.kp = 0.0;
        motor.rotatePID.ki = 0.0;
        motor.rotatePID.kd = 0.0;

        motor.motorS.rpmPID.maxOutput = 255;
        motor.motorS.rpmPID.minOutput = 0;
        motor.motorS.rpmPID.maxError = 1000.0;
        motor.motorS.rpmPID.errorThresholdPermission = 3.0;
        motor.motorS.rpmPID.kp = 0.6;
        motor.motorS.rpmPID.ki = 0.4;
        motor.motorS.rpmPID.kd = 0;

        motor.motorNE.rpmPID.maxOutput = 255;
        motor.motorNE.rpmPID.minOutput = 0;
        motor.motorNE.rpmPID.maxError = 1000.0;
        motor.motorNE.rpmPID.errorThresholdPermission = 3.0;
        motor.motorNE.rpmPID.kp = 0.6;
        motor.motorNE.rpmPID.ki = 0.4;
        motor.motorNE.rpmPID.kd = 0;

        motor.motorNW.rpmPID.maxOutput = 255;
        motor.motorNW.rpmPID.minOutput = 0;
        motor.motorNW.rpmPID.maxError = 1000.0;
        motor.motorNW.rpmPID.errorThresholdPermission = 3.0;
        motor.motorNW.rpmPID.kp = 0.6;
        motor.motorNW.rpmPID.ki = 0.4;
        motor.motorNW.rpmPID.kd = 0;
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

    void move(int16_t rpm, int16_t direction, int16_t facing) {
        int motorSSpeed = rpm * sin ((180 + direction) * (PI / 180));
        int motorNESpeed = rpm * sin ((60 + direction) * (PI / 180));
        int motorNWSpeed = rpm * sin ((300 + direction) * (PI / 180));

        int maxSpeed = std::max({std::abs(motorSSpeed), std::abs(motorNESpeed), std::abs(motorNWSpeed)});

        motorSSpeed *= (rpm / maxSpeed);
        motorNESpeed *= (rpm / maxSpeed);
        motorNWSpeed *= (rpm / maxSpeed);

        // FIXME: This is a placeholder for the PID controller, should be replaced with real BNO055 data
        motor.movePID.error = facing - 0;
        motor.movePID.target = facing;
        motor.pid.compute(motor.movePID);

        moveS(-(motorSSpeed + motor.movePID.output));
        moveNE(-(motorNESpeed + motor.movePID.output));
        moveNW(motorNWSpeed + motor.movePID.output);
    }

    void motorS(int16_t speed) {
        std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

        uint slice_num = pwm_gpio_to_slice_num(MOTOR_S_PWM);
        pwm_set_wrap(slice_num, 255);
        pwm_set_chan_level(slice_num, PWM_CHAN_A + (MOTOR_S_PWM % 2), std::abs(speed));
        pwm_set_enabled(slice_num, true);

        gpio_set_dir(MOTOR_S_DIR, GPIO_OUT);
        gpio_put(MOTOR_S_DIR, speed > 0);
        motor.motorS.speed = speed;
    }

    void motorNW(int16_t speed) {
        std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

        uint slice_num = pwm_gpio_to_slice_num(MOTOR_NW_PWM);
        pwm_set_wrap(slice_num, 255);
        pwm_set_chan_level(slice_num, PWM_CHAN_A + (MOTOR_NW_PWM % 2), std::abs(speed));
        pwm_set_enabled(slice_num, true);

        gpio_set_dir(MOTOR_NW_DIR, GPIO_OUT);
        gpio_put(MOTOR_NW_DIR, speed > 0);
        motor.motorNW.speed = speed;
    }

    void motorNE(int16_t speed) {
        std::clamp(speed, static_cast<int16_t>(-MAX_SPEED), static_cast<int16_t>(MAX_SPEED));

        uint slice_num = pwm_gpio_to_slice_num(MOTOR_NE_PWM);
        pwm_set_wrap(slice_num, 255);
        pwm_set_chan_level(slice_num, PWM_CHAN_A + (MOTOR_NE_PWM % 2), std::abs(speed));
        pwm_set_enabled(slice_num, true);

        gpio_set_dir(MOTOR_NE_DIR, GPIO_OUT);
        gpio_put(MOTOR_NE_DIR, speed > 0);
        motor.motorNE.speed = speed;
    }

    void moveS(int16_t rpm) {
        if (std::abs(rpm) <= 15) {
            motorS(0);
            motor.motorS.previousOut = 0;
            motor.motorS.rpmPID.errorSum = 0;
            motor.motorS.rpmPID.previousError = 0;
            return;
        }

        if (millis() - motor.motorS.lastIterationTime < motor.motorS.rpmPID.delayMiliseconds) return;

        motor.motorS.rpmPID.error = std::abs(rpm) - std::abs(motor.motorS.rpmB);
        motor.motorS.rpmPID.target = std::abs(rpm);
        motor.pid.compute(motor.motorS.rpmPID);

        if (rpm < 0) {
            if (motor.motorS.previousOut > 0) motor.motorS.previousOut *= -1;
            motor.motorS.previousOut = motor.motorS.previousOut - motor.motorS.rpmPID.output;
            motor.motorS.previousOut = std::clamp(motor.motorS.previousOut, -255, -20);
        } else {
            if (motor.motorS.previousOut < 0) motor.motorS.previousOut *= -1;
            motor.motorS.previousOut = motor.motorS.previousOut + motor.motorS.rpmPID.output;
            motor.motorS.previousOut = std::clamp(motor.motorS.previousOut, 20, 255);
        }

//        std::cout << " rpm: " << motor.motorS.rpmB << " out: " << motor.motorS.rpmPID.output << " prev: " << motor.motorS.previousOut << std::endl;

        motorS(motor.motorS.previousOut);
    }

    void moveNW(int16_t rpm) {
        if (std::abs(rpm) <= 15) {
            motorNW(0);
            motor.motorNW.previousOut = 0;
            motor.motorNW.rpmPID.errorSum = 0;
            motor.motorNW.rpmPID.previousError = 0;
            return;
        }

        if (millis() - motor.motorNW.lastIterationTime < motor.motorNW.rpmPID.delayMiliseconds) return;

        motor.motorNW.rpmPID.error = std::abs(rpm) - std::abs(motor.motorNW.rpmB);
        motor.motorNW.rpmPID.target = std::abs(rpm);
        motor.pid.compute(motor.motorNW.rpmPID);

        if (rpm < 0) {
            if (motor.motorNW.previousOut > 0) motor.motorNW.previousOut *= -1;
            motor.motorNW.previousOut = motor.motorNW.previousOut - motor.motorNW.rpmPID.output;
            motor.motorNW.previousOut = std::clamp(motor.motorNW.previousOut, -255, -20);
        } else {
            if (motor.motorNW.previousOut < 0) motor.motorNW.previousOut *= -1;
            motor.motorNW.previousOut = motor.motorNW.previousOut + motor.motorNW.rpmPID.output;
            motor.motorNW.previousOut = std::clamp(motor.motorNW.previousOut, 20, 255);
        }

//        std::cout << " rpm: " << motor.motorNW.rpmB << " output: " << motor.motorNW.previousOut << std::endl;

        motorNW(motor.motorNW.previousOut);
    }

    void moveNE(int16_t rpm) {
        if (std::abs(rpm) <= 15) {
            motorNE(0);
            motor.motorNE.previousOut = 0;
            motor.motorNE.rpmPID.errorSum = 0;
            motor.motorNE.rpmPID.previousError = 0;
            return;
        }

        if (millis() - motor.motorNE.lastIterationTime < motor.motorNE.rpmPID.delayMiliseconds) return;

        motor.motorNE.rpmPID.error = std::abs(rpm) - std::abs(motor.motorNE.rpmB);
        motor.motorNE.rpmPID.target = std::abs(rpm);
        motor.pid.compute(motor.motorNE.rpmPID);

        if (rpm < 0) {
            if (motor.motorNE.previousOut > 0) motor.motorNE.previousOut *= -1;
            motor.motorNE.previousOut = motor.motorNE.previousOut - motor.motorNE.rpmPID.output;
            motor.motorNE.previousOut = std::clamp(motor.motorNE.previousOut, -255, -20);
        } else {
            if (motor.motorNE.previousOut < 0) motor.motorNE.previousOut *= -1;
            motor.motorNE.previousOut = motor.motorNE.previousOut + motor.motorNE.rpmPID.output;
            motor.motorNE.previousOut = std::clamp(motor.motorNE.previousOut, 20, 255);
        }

        motorNE(motor.motorNE.previousOut);
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