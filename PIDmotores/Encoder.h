#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"
#include "Motor.h"
#include "Movement.h"

// IDS
/* #define FRONT_LEFT 0
#define BACK_LEFT 1
#define FRONT_RIGHT 2
#define BACK_RIGHT 3 

#define RobotForward 1
#define RobotTurnRigth 2
#define RobotTurnLeft 3
#define RobotBackward 4  */

namespace Encoder {
    void initEncoder();
    void backLeftEncoder();
    void backRightEncoder();
    void frontLeftEncoder();
    void frontRightEncoder();
};

#endif