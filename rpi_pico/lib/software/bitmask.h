//
// Created by kali on 3/14/24.
//

#include <algorithm>
#include <vector>

#ifndef PICO_LIB_BITMASK_H
#define PICO_LIB_BITMASK_H

#define BITMASK_DEFAULT_VALUE 0

namespace Bitmask {
    enum class MOTOR_ADDRESS {
        N_SPEED = 0x00,
        SW_SPEED = 0x04,
        SE_SPEED = 0x08,
        N_RPM = 0x0C,
        SW_RPM = 0x10,
        SE_RPM = 0x14,
    };

    enum class MOTOR_SIZES {
        N_SPEED = 4,
        SW_SPEED = 4,
        SE_SPEED = 4,
        N_RPM = 4,
        SW_RPM = 4,
        SE_RPM = 4,
    };

    enum class COMPASS_ADDRESS {
        YAW = 0x18,
        PITCH = 0x1C,
        ROLL = 0x20,
    };

    enum class COMPASS_SIZES {
        YAW = 4,
        PITCH = 4,
        ROLL = 4,
    };

    enum class PERIPHERALS_ADDRESS {
        DRIBBLER = 0x24,
        KICKER = 0x28,
    };

    enum class PERIPHERALS_SIZES {
        DRIBBLER = 4,
        KICKER = 1,
    };

    enum class LDR_ADDRESS {
        LDR1 = 0x29,
        LDR2 = 0x2A,
        LDR3 = 0x2B,
        LDR4 = 0x2C,
        LDR5 = 0x2D,
        LDR6 = 0x2E,
        LDR7 = 0x2F,
        LDR8 = 0x30,
        LDR9 = 0x31,
    };

    enum class LDR_SIZES {
        LDR1 = 1,
        LDR2 = 1,
        LDR3 = 1,
        LDR4 = 1,
        LDR5 = 1,
        LDR6 = 1,
        LDR7 = 1,
        LDR8 = 1,
        LDR9 = 1,
    };

    struct BitmaskStructure {
        int value;
        int bitSize;
        int bitShift;
    };

    extern std::vector<BitmaskStructure> bitmask;

    unsigned long generateBitmask();
    int bitmaskToValue(int address);

    void setBitmask(unsigned long custom_bitmask);
    void setBitmaskValue(int value, int address);
}


#endif //PICO_LIB_BITMASK_H
