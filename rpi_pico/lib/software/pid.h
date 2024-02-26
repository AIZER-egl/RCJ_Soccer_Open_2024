//
// Created by kali on 2/7/24.
//

#ifndef PICO_LIB_PID_H
#define PICO_LIB_PID_H

#include "../../pico-lib/time.h"

class PID {
public:
    struct PidParameters {
        bool firstRun = false;

        int maxOutput;
        int minOutput;
        float maxError;
        float errorThresholdPermission;

        float kp;
        float ki;
        float kd;

        int errorSum = 0;
        int previousError = 0;
        int delayMiliseconds = 100;
        unsigned long lastIterationTime = 0;

        int error;
        int output;
        int target;
    };
    void reset(PidParameters& pid);
    void compute(PidParameters& pid);
};


#endif //PICO_LIB_PID_H
