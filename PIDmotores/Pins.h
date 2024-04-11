#ifndef Pins_h
#define Pins_h

namespace Pins {
    constexpr uint8_t pwmPin[4] = {
        5, // BACK_LEFT 16
        32, // FRONT_LEFT 32
        15, // BACK_RIGHT 15
        26  // FRONT_RIGHT 26
    };

    constexpr uint8_t digitalOne[4] = {
        17, // BACK_LEFT 17
        33, // FRONT_LEFT 33
        2, // BACK_RIGHT 4
        14  // FRONT_RIGHT 14
    };  

    constexpr uint8_t digitalTwo[4] = {
        16, // BACK_LEFT 5
        25, // FRONT_LEFT 25
        4, // BACK_RIGHT 2
        27  // FRONT_RIGHT 27
    };

    constexpr uint8_t encoderA[4] = {
        35, // BACK_LEFT 35
        39, // FRONT_LEFT 39
        34, // BACK_RIGHT 34
        36  // FRONT_RIGHT 36
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

    constexpr uint8_t servoPin = 18;

    constexpr uint8_t buttonPin = 19;
}
#endif