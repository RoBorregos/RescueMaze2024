#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"
#include "Motor.h"
#include "Movement.h"

namespace Encoder {
    void backLeftEncoder();
    void backRightEncoder();
    void frontLeftEncoder();
    void frontRightEncoder();
};

#endif