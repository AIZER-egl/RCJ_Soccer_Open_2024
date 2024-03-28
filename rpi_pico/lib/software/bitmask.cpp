//
// Created by kali on 3/14/24.
//

#include "bitmask.h"

namespace Bitmask {
    std::vector<BitmaskStructure> bitmask = {
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(MOTOR_SIZES::N_SPEED),
                static_cast<int>(MOTOR_ADDRESS::N_SPEED)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(MOTOR_SIZES::SW_SPEED),
                static_cast<int>(MOTOR_ADDRESS::SW_SPEED)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(MOTOR_SIZES::SE_SPEED),
                static_cast<int>(MOTOR_ADDRESS::SE_SPEED)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(MOTOR_SIZES::N_RPM),
                static_cast<int>(MOTOR_ADDRESS::N_RPM)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(MOTOR_SIZES::SW_RPM),
                static_cast<int>(MOTOR_ADDRESS::SW_RPM)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(MOTOR_SIZES::SE_RPM),
                static_cast<int>(MOTOR_ADDRESS::SE_RPM)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(COMPASS_SIZES::YAW),
                static_cast<int>(COMPASS_ADDRESS::YAW)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(COMPASS_SIZES::PITCH),
                static_cast<int>(COMPASS_ADDRESS::PITCH)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(COMPASS_SIZES::ROLL),
                static_cast<int>(COMPASS_ADDRESS::ROLL)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(PERIPHERALS_SIZES::DRIBBLER),
                static_cast<int>(PERIPHERALS_ADDRESS::DRIBBLER)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(PERIPHERALS_SIZES::KICKER),
                static_cast<int>(PERIPHERALS_ADDRESS::KICKER)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(LDR_SIZES::LDR1),
                static_cast<int>(LDR_ADDRESS::LDR1)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(LDR_SIZES::LDR2),
                static_cast<int>(LDR_ADDRESS::LDR2)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(LDR_SIZES::LDR3),
                static_cast<int>(LDR_ADDRESS::LDR3)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(LDR_SIZES::LDR4),
                static_cast<int>(LDR_ADDRESS::LDR4)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(LDR_SIZES::LDR5),
                static_cast<int>(LDR_ADDRESS::LDR5)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(LDR_SIZES::LDR6),
                static_cast<int>(LDR_ADDRESS::LDR6)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(LDR_SIZES::LDR7),
                static_cast<int>(LDR_ADDRESS::LDR7)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(LDR_SIZES::LDR8),
                static_cast<int>(LDR_ADDRESS::LDR8)
            },
            {
                BITMASK_DEFAULT_VALUE,
                static_cast<int>(LDR_SIZES::LDR9),
                static_cast<int>(LDR_ADDRESS::LDR9)
            }
    };

    unsigned long generateBitmask () {
        unsigned long generatedBitmask = 0;
        for (int i = 0; i < bitmask.size(); i++) {
            generatedBitmask |= (bitmask[i].value << bitmask[i].bitShift);
        }
        return generatedBitmask;
    }

    int bitmaskToValue(int address) {

        auto element = std::find_if(bitmask.begin(), bitmask.end(), [address](const BitmaskStructure &element) {
            return element.bitShift == address;
        });

        if (element -> bitShift == address) {
            return element -> value;
        }

        return -1;
    }

    void setBitmaskValue(int value, int address) {
        auto element = std::find_if(bitmask.begin(), bitmask.end(), [address](const BitmaskStructure &element) {
            return element.bitShift == address;
        });

        if (element -> bitShift == address) {
            element -> value = value;
        }
    }

    void setBitmask(unsigned long custom_bitmask) {
        for (int i = 0; i < bitmask.size(); i++) {
            bitmask[i].value = (custom_bitmask >> bitmask[i].bitShift) & ((1 << bitmask[i].bitSize) - 1);
        }
    }
}