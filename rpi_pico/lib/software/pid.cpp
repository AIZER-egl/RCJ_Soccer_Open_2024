#include "pid.h"

void PID::reset(PidParameters& pid) {
    pid.firstRun = true;
    pid.errorSum = 0;
    pid.previousError = 0;
    pid.lastIterationTime = 0;
}

void PID::compute(PidParameters& pid) {
    if (pid.firstRun) {
        pid.previousError = pid.error;
        pid.firstRun = false;
        pid.lastIterationTime = millis();
    }

    if (millis() - pid.lastIterationTime >= pid.delayMiliseconds) {
        pid.lastIterationTime = millis();
        pid.errorSum += pid.error;

        if (std::abs(pid.errorSum) > pid.maxError) {
            int multiply = pid.errorSum / std::abs(pid.errorSum);
            pid.errorSum = pid.maxError * multiply;
        }

        float proportionalTerm = pid.kp * pid.error;
        float integralTerm = pid.ki * pid.errorSum;
        float derivativeTerm = pid.kd * (pid.error - pid.previousError);

        int output = proportionalTerm + integralTerm + derivativeTerm;

        if (std::abs(output) > pid.maxOutput) {
            int multiply = output / std::abs(output);
            output = pid.maxOutput * multiply;
        }

        if (std::abs(output) < pid.minOutput) {
            int multiply = output / std::abs(output);
            output = pid.minOutput * multiply;
        }

        if (std::abs(pid.error) < pid.errorThresholdPermission) {
            output = 0;
            reset(pid);
        }

        pid.previousError = pid.error;

        pid.output = output;
    }
}