//
// Created by kali on 3/14/24.
//

#include <algorithm>
#include <vector>
#include <cstdint>
#include <bitset>

#ifndef PICO_LIB_BITMASK_H
#define PICO_LIB_BITMASK_H

#define BITMASK_DEFAULT_VALUE 0

namespace RPI_Bitmask {
    enum class MOTOR_ADDRESS {
        S_SPEED = 0x00,
        NW_SPEED = 0x08,
        NE_SPEED = 0x10,
        S_DIR = 0x18,
        NW_DIR = 0x19,
        NE_DIR = 0x1A,
    };

    enum class MOTOR_SIZES {
        S_SPEED = 8,
        NW_SPEED = 8,
        NE_SPEED = 8,
        S_DIR = 1,
        NW_DIR = 1,
        NE_DIR = 1,
    };

    enum class COMPASS_ADDRESS {
        YAW = 0x1B,
    };

    enum class COMPASS_SIZES {
        YAW = 9,
    };

    enum class PERIPHERALS_ADDRESS {
        DRIBBLER = 0x24,
        KICKER = 0x2C,
    };

    enum class PERIPHERALS_SIZES {
        DRIBBLER = 8,
        KICKER = 1,
    };

    enum class LDR_ADDRESS {
        LDR1 = 0x2D,
        LDR2 = 0x2E,
        LDR3 = 0x2F,
        LDR4 = 0x30,
        LDR5 = 0x31,
        LDR6 = 0x32,
        LDR7 = 0x33,
        LDR8 = 0x34,
        LDR9 = 0x35,
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

    uint64_t generateBitmask();
    int bitmaskToValue(int address);

    void setBitmask(uint64_t custom_bitmask);
    void setBitmaskValue(int value, int address);
}


#endif //PICO_LIB_BITMASK_H
