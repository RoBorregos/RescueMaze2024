#ifndef Pines_h
#define Pines_h

// pines de motores y encoders
// front left
/* constexpr uint8_t pwmFL = 8;
constexpr uint8_t daFL = 26;
constexpr uint8_t dbFL = 27;
constexpr uint8_t eaFL = 3;
// front right
constexpr uint8_t pwmFR = 5;
constexpr uint8_t daFR = 29;
constexpr uint8_t dbFR = 28;
constexpr uint8_t eaFR = 2;
// back left
constexpr uint8_t pwmBL = 4;
constexpr uint8_t daBL = 24;
constexpr uint8_t dbBL = 25;
constexpr uint8_t eaBL = 19;
// back right
constexpr uint8_t pwmBR = 7;
constexpr uint8_t daBR = 23;
constexpr uint8_t dbBR = 22;
constexpr uint8_t eaBR = 18; */


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
// tics por revolucion
// 496 BACK_LEFT
// 496 FRONT_LEFT
// 496 BACK_RIGHT
// 496 FRONT_RIGHT


#endif