
#include "Movement.h"
#include "Pins.h"
//#include "Encoder.h"

constexpr double kP = 5.0; 
constexpr double kI = 0.008;
constexpr double kD = 0.0;

PID pidStraight(kP, kI, kD);
BNO bno;

Movement::Movement() {
    this->motor[4];

}

void Movement::setup() {
    setupInternal(MotorID::kFrontLeft);
    setupInternal(MotorID::kFrontRight);
    setupInternal(MotorID::kBackLeft);
    setupInternal(MotorID::kBackRight);
    bno.setupBNO();
    //Encoder::initEncoder();
}

void Movement::setupInternal(MotorID motorId) {
    int index = static_cast<int>(motorId);
    motor[index].motorSetup(
        pwmPin[index],
        digitalOne[index],
        digitalTwo[index],
        encoderA[index],
        motorId);
}

void Movement::stopMotors() {
    for(int i = 0; i < 4; ++i){
        motor[i].motorStop();
    }
}

void Movement::forwardMotors(const uint8_t pwms[4]) {
    for(int i = 0; i < 4; ++i){
        motor[i].motorForward(pwms[i]);
    }
}

void Movement::backwardMotors(const uint8_t pwms[4]) {
    for(int i = 0; i < 4; ++i){
        motor[i].motorBackward(pwms[i]);
    }
}

void Movement::moveMotors(MotorState state) {
    uint8_t pwms[4];
    int pwm = 60;
    double targetOrientation = 0;
    double currentOrientation = bno.getOrientationX();
    double pwmLeft = 0;
    double pwmRight = 0;
    switch (state)
    {
        case (MotorState::kStop): {
            stopMotors();
            break;
        }
        
        case (MotorState::kForward): {
            pidStraight.computeStraight(targetOrientation,currentOrientation, pwmLeft, pwmRight);

            pwms[static_cast<int>(MotorID::kFrontLeft)]= pwmLeft;
            pwms[static_cast<int>(MotorID::kBackLeft)]= pwmLeft;
            pwms[static_cast<int>(MotorID::kFrontRight)]= pwmRight;
            pwms[static_cast<int>(MotorID::kBackRight)]= pwmRight;

            forwardMotors(pwms);
            break;
        }
        case (MotorState::kBackward): {
            pwms[0]= pwm;
            pwms[1]= pwm;
            pwms[2]= pwm;
            pwms[3]= pwm;
            backwardMotors(pwms);
            break;
        }
        default : {
            break;
        }
    }
}

// CALIZZ ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''FEDFE'FREFEJ,FBEJFFBHUWBFUHEWBFHJWERBFHJRBEJHFBRJFBKWRJNSBJFBKREJFBKEJB
void Movement::updateTics(MotorID motorId) {

    int index = static_cast<int>(motorId);
    if (index >= 0 && index < 4) {
        motor[index].deltaPidTics(1);
        if (motor[index].getCurrentState() == MotorState::kForward){
            motor[index].deltaEncoderTics(1);
        } else if (motor[index].getCurrentState() == MotorState::kBackward){
            motor[index].deltaEncoderTics(-1);
        }
        else {
            return;
        }
    }
    /* motor->deltaPidTics(1);

    if (motor->getCurrentState() == MotorState::kForward){
        motor->deltaEncoderTics(1);
    } else if (motor->getCurrentState() == MotorState::kBackward){
        motor->deltaEncoderTics(-1);
    }
    else {
        return;
    } */
} 

int Movement::getBackLeftEncoderTics() {
    return motor[static_cast<int>(MotorID::kBackLeft)].getEncoderTics();
}

int Movement::getFrontLeftEncoderTics() {
    return motor[static_cast<int>(MotorID::kFrontLeft)].getEncoderTics();
}

int Movement::getBackRightEncoderTics() {
    return motor[static_cast<int>(MotorID::kBackRight)].getEncoderTics();
}

int Movement::getFrontRightEncoderTics() {
    return motor[static_cast<int>(MotorID::kFrontRight)].getEncoderTics();
}
