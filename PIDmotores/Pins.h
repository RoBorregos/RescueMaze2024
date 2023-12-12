#ifndef Pins_h
#define Pins_h

// CON PINES YA DEFINIDOS CORRECTAMENTE
constexpr uint8_t pwmPin[4] = {
    27, // BACK_LEFT
    16, // FRONT_LEFT
    26, // BACK_RIGHT
    23  // FRONT_RIGHT
};

constexpr uint8_t digitalOne[4] = {
    32, // BACK_LEFT
    17, // FRONT_LEFT
    25, // BACK_RIGHT
    18  // FRONT_RIGHT
};  

constexpr uint8_t digitalTwo[4] = {
    14, // BACK_LEFT
    5, // FRONT_LEFT
    33, // BACK_RIGHT
    19  // FRONT_RIGHT
};

constexpr uint8_t encoderA[4] = {
    34, // BACK_LEFT
    36, // FRONT_LEFT
    35, // BACK_RIGHT
    39  // FRONT_RIGHT
};
#endif