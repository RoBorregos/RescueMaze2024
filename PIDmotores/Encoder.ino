#include "Encoder.h"
#include "Pins.h"

/* void Encoder::updateTics(Motor *motor){
    motor->deltaPidTics(1);

    if (motor->getCurrentState() == MotorState::kForward){
        motor->deltaEncoderTics(1);
    } else if (motor->getCurrentState() == MotorState::kBackward){
        motor->deltaEncoderTics(-1);
    }
    else {
        return;
    }
}  */

// descomentarizar

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

void Encoder::initEncoder() {
    pinMode(encoderA[0], INPUT);
    pinMode(encoderA[1], INPUT);
    pinMode(encoderA[2], INPUT);
    pinMode(encoderA[3], INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderA[0]), backLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderA[1]), frontLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderA[2]), backRightEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderA[3]), frontRightEncoder, RISING);
}