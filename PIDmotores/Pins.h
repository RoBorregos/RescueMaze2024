#ifndef Pins_h
#define Pins_h

namespace Pins {
    constexpr uint8_t pwmPin[4] = {
        23, // BACK_LEFT
        26, // FRONT_LEFT
        16, // BACK_RIGHT
        27  // FRONT_RIGHT
    };

    constexpr uint8_t digitalOne[4] = {
        18, // BACK_LEFT
        25, // FRONT_LEFT
        5, // BACK_RIGHT
        32  // FRONT_RIGHT
    };  

    constexpr uint8_t digitalTwo[4] = {
        19, // BACK_LEFT
        33, // FRONT_LEFT
        17, // BACK_RIGHT
        14  // FRONT_RIGHT
    };

    constexpr uint8_t encoderA[4] = {
        34, // BACK_LEFT
        36, // FRONT_LEFT
        35, // BACK_RIGHT
        39  // FRONT_RIGHT
    };

    constexpr uint8_t vlxPins[1] = {
        6
    };
}
#endif