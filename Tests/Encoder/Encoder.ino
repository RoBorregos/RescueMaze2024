#include <Arduino.h>

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

const int timeEpochTics_ = 0;

void deltaTics(const int deltaTics) {
    timeEpochTics_ += deltaTics;
}

void setup {
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(encoderA[0]), deltaTics(1), RISING);
    attachInterrupt(digitalPinToInterrupt(encoderA[1]), deltaTics(1), RISING);
    attachInterrupt(digitalPinToInterrupt(encoderA[2]), deltaTics(1), RISING);
    attachInterrupt(digitalPinToInterrupt(encoderA[3]), deltaTics(1), RISING);
}

void loop {
  

}
