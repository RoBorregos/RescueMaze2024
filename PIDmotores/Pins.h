#ifndef Pins_h
#define Pins_h

namespace Pins {
    constexpr uint8_t pwmPin[4] = {
        26, // BACK_LEFT 16
        15, // FRONT_LEFT 32
        32, // BACK_RIGHT 15
        5  // FRONT_RIGHT 26
    };

    constexpr uint8_t digitalOne[4] = {
        27, // BACK_LEFT 17
        2, // FRONT_LEFT 33
        33, // BACK_RIGHT 4
        16  // FRONT_RIGHT 14
    };  

    constexpr uint8_t digitalTwo[4] = {
        14, // BACK_LEFT 5
        4, // FRONT_LEFT 25
        25, // BACK_RIGHT 2
        17  // FRONT_RIGHT 27
    };

    constexpr uint8_t encoderA[4] = {
        36, // BACK_LEFT 35
        34, // FRONT_LEFT 39
        39, // BACK_RIGHT 34
        35  // FRONT_RIGHT 36
    };

    constexpr uint8_t vlxPins[5] = {
        // Defined Pins on 20th of March
        2,  // FRONT_LEFT  fine 0
        3, // LEFT         fine 1        
        0, // BACK         fine 2
        5,  // RIGHT       fine 4
        4 // FRONT_RIGHT   fine 3
    };

    constexpr uint8_t limitSwitchPins[2] = {
        13, // LEFT
        23  // RIGHT
    };

    constexpr uint8_t tcsPins[1] = {
        1 // 5
    };

}
#endif