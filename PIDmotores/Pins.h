#ifndef Pins_h
#define Pins_h

namespace Pins {
    constexpr uint8_t pwmPin[4] = {
        16, // BACK_LEFT
        32, // FRONT_LEFT
        15, // BACK_RIGHT
        26  // FRONT_RIGHT
    };

    constexpr uint8_t digitalOne[4] = {
        17, // BACK_LEFT
        33, // FRONT_LEFT
        4, // BACK_RIGHT
        14  // FRONT_RIGHT
    };  

    constexpr uint8_t digitalTwo[4] = {
        5, // BACK_LEFT
        25, // FRONT_LEFT
        2, // BACK_RIGHT
        27  // FRONT_RIGHT
    };

    constexpr uint8_t encoderA[4] = {
        35, // BACK_LEFT
        39, // FRONT_LEFT
        34, // BACK_RIGHT
        36  // FRONT_RIGHT
    };

    constexpr uint8_t vlxPins[5] = {
        // PINES DEFINIDOS EL DIA 20 DE MARZO JALAN BIEN CAMBIARLO EN EL
        0,  // FRONT_LEFT
        2, // LEFT
        1, // BACK
        4,  // RIGHT
        3 // FRONT_RIGHT
    };

    constexpr uint8_t limitSwitchPins[2] = {
        13, // LEFT
        23  // RIGHT
    };

}
#endif