#include "Encoder.h"

void Encoder::updateTics(Motor *motor){
    motor->deltaPidTics(1);

    if (motor->getCurrentState() == MotorState::Forward){
        motor->deltaEncoderTics(1);
    } else if (motor->getCurrentState() == MotorState::Backward){
        motor->deltaEncoderTics(-1);
    }
    else {
        return;
    }
}

void Encoder::backLeftEncoder(){
    updateTics(&backLeft);
}

void Encoder::backRightEncoder(){
    updateTics(&backRight);
}

void Encoder::frontLeftEncoder(){
    updateTics(&frontLeft);
}

void Encoder::frontRightEncoder(){
    updateTics(&frontRight);
}

