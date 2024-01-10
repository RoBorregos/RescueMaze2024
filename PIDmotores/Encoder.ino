#include "Encoder.h"
#include "Pins.h"

extern Movement robot;

void Encoder::backLeftEncoder() {
    robot.updateTics(MotorID::kBackLeft);
}

void Encoder::backRightEncoder() {
    robot.updateTics(MotorID::kBackRight);
}

void Encoder::frontLeftEncoder() {
    robot.updateTics(MotorID::kFrontLeft);
}

void Encoder::frontRightEncoder() {
    robot.updateTics(MotorID::kFrontRight);
}