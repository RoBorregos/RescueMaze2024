
#include "Movement.h"
#include "Pines.h"
//#include "Encoder.h"

// Ajustar
// JALAN 2/3
constexpr double kP = 5.0; 
constexpr double kI = 0.008;
constexpr double kD = 0.0;

PID pidStraight(kP, kI, kD);
BNO bno;

Movement::Movement() {
    this->motor[4];

    this-> next_time = millis();

    this-> errorPrevOrientation = 0;
    this-> errorAcumuladoOrientation = 0;
}

void Movement::setup() {
    setupInternal(MotorID::FRONT_LEFT);
    setupInternal(MotorID::FRONT_RIGHT);
    setupInternal(MotorID::BACK_LEFT);
    setupInternal(MotorID::BACK_RIGHT);
    bno.setupBNO();
    //Encoder::initEncoder();
}

void Movement::setupInternal(MotorID motorId) {
    int index = static_cast<int>(motorId);
    motor[index].motoresSetup(
        pwmPin[index],
        digitalOne[index],
        digitalTwo[index],
        encoderA[index],
        motorId);
}

void Movement::moveForward(int pwmA, int pwmB, int pwmC, int pwmD) {
    motorFL.updateRPM();
    motorFR.updateRPM();
    motorBL.updateRPM();
    motorBR.updateRPM();
    motorFL.setPWM(pwmA);
    motorFR.setPWM(pwmB);
    motorBL.setPWM(pwmC);
    motorBR.setPWM(pwmD);

}

float Movement::getRPMFL() {
    return motorFL.getRPM();
}

float Movement::getRPMFR() {
    return motorFR.getRPM();
}

float Movement::getRPMBL() {
    return motorBL.getRPM();
}

float Movement::getRPMBR() {
    return motorBR.getRPM();
}

void Movement::stopMotors() {
    for(int i=0; i<4; ++i){
        motor[i].motorStop();
    }
}

void Movement:: forwardMotors(const uint8_t pwms[4]) {
    for(int i=0; i<4; i++){
        motor[i].motorForward(pwms[i]);
    }
}

void Movement:: backwardMotors(const uint8_t pwms[4]) {
    for(int i=0; i<4; i++){
        motor[i].motorBackward(pwms[i]);
    }
}
void Movement:: turnRight(const uint8_t pwms[4]) {
    motor[0].motorBackward(pwms[0]);
    motor[1].motorBackward(pwms[1]);
    motor[2].motorBackward(pwms[2]);
    motor[3].motorBackward(pwms[3]);
}

void Movement:: turnLeft(const uint8_t pwms[4]) {
    motor[0].motorForward(pwms[0]);
    motor[1].motorForward(pwms[1]);
    motor[2].motorForward(pwms[2]);
    motor[3].motorForward(pwms[3]);
}


void Movement:: moveMotors(MotorState state) {
    uint8_t pwms[4];
    int pwm = 60;
    double targetOrientation = 0;
    double currentOrientation = bno.getOrientationX();
    double pwmLeft = 0;
    double pwmRight = 0;
    switch (state)
    {
        case (MotorState::Stop): {
            stopMotors();
            break;
        }
        
        case (MotorState::Forward): {
            pidStraight.computeStraight(targetOrientation,currentOrientation, pwmLeft, pwmRight);

            pwms[static_cast<int>(MotorID::FRONT_LEFT)]= pwmLeft;
            pwms[static_cast<int>(MotorID::BACK_LEFT)]= pwmLeft;
            pwms[static_cast<int>(MotorID::FRONT_RIGHT)]= pwmRight;
            pwms[static_cast<int>(MotorID::BACK_RIGHT)]= pwmRight;

            forwardMotors(pwms);
            break;
        }
        case (MotorState::Backward): {
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
        if (motor[index].getCurrentState() == MotorState::Forward){
            motor[index].deltaEncoderTics(1);
        } else if (motor[index].getCurrentState() == MotorState::Backward){
            motor[index].deltaEncoderTics(-1);
        }
        else {
            return;
        }
    }
    /* motor->deltaPidTics(1);

    if (motor->getCurrentState() == MotorState::Forward){
        motor->deltaEncoderTics(1);
    } else if (motor->getCurrentState() == MotorState::Backward){
        motor->deltaEncoderTics(-1);
    }
    else {
        return;
    } */
} 

int Movement::getBackLeftEncoderTics() {
    return motor[static_cast<int>(MotorID::BACK_LEFT)].getEncoderTics();
}

int Movement::getFrontLeftEncoderTics() {
    return motor[static_cast<int>(MotorID::FRONT_LEFT)].getEncoderTics();
}

int Movement::getBackRightEncoderTics() {
    return motor[static_cast<int>(MotorID::BACK_RIGHT)].getEncoderTics();
}

int Movement::getFrontRightEncoderTics() {
    return motor[static_cast<int>(MotorID::FRONT_RIGHT)].getEncoderTics();
}