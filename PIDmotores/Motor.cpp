
#include "Motor.h"
#include "Encoder.h"
#include "Pines.h"


Motor::Motor() {
    this->pwmPin = 0;
    this->digitalOne = 0;
    this->digitalTwo = 0;
    this->encoderA = 0;
    this->rpm = 0;
    this->next_time = millis();
    this->pwmInicial = 50;
    this->errorPrev = 0;
    this->errorAcumulado = 0;
    this->motorId = MotorID::NONE;
}

Motor::Motor(const uint8_t digitalOne, const uint8_t digitalTwo, const uint8_t pwmPin, const uint8_t encoderA, const MotorID motorid) {
    this->pwmPin = pwmPin;
    this->digitalOne = digitalOne;
    this->digitalTwo = digitalTwo;
    this->encoderA = encoderA;
    this->motorId = motorid;
    this->encoders[0] = 0;
    this->encoders[1] = 0;
    this->encoders[2] = 0;
    this->encoders[3] = 0;
}

uint8_t Motor::getEncoderA() {
    return encoderA;
}

MotorState Motor::getCurrentState() {
    return currentState;
}

int Motor::getEncoderTics() {
    return ticsCounter;
}

double Motor::getCurrentSpeed() {
    return currentSpeed;
}

double Motor::getTargetSpeed() {
    return RpmToRps(targetSpeed);
}

int Motor::getPidTics() {
    return pidTics;
}

void Motor::initMotor() {
    motorSetup();
    initEncoder();
    motorStop();
}


void Motor::motorSetup() {
    pinMode(pwmPin, OUTPUT);
    pinMode(digitalOne, OUTPUT);
    pinMode(digitalTwo, OUTPUT);
    pinMode(encoderA, INPUT);
}

// no comentar
/* void Motor::initEncoder() {
    pinMode(encoderA, INPUT_PULLUP);
    
    switch (motorId) {

        case (MotorID::FRONT_LEFT):{
            attachInterrupt(digitalPinToInterrupt(encoderA), Encoder::frontLeftEncoder, RISING);
            break;
        }
        case (MotorID::FRONT_RIGHT): {
            attachInterrupt(digitalPinToInterrupt(encoderA), Encoder::frontRightEncoder, RISING);
            break;
        }
        case (MotorID::BACK_LEFT): {
            attachInterrupt(digitalPinToInterrupt(encoderA), Encoder::backLeftEncoder, RISING);
            break;
        }
        case (MotorID::BACK_RIGHT): {
            attachInterrupt(digitalPinToInterrupt(encoderA), Encoder::backRightEncoder, RISING);
            break;
        }
        default: { 
            break;
        }
    }
} */

void Motor::motoresSetup(const uint8_t pwmPin, const uint8_t digitalOne, const uint8_t digitalTwo, const uint8_t encoderA, const MotorID motorid) {
    this->pwmPin = pwmPin;
    this->digitalOne = digitalOne;
    this->digitalTwo = digitalTwo;
    this->motorId = motorid;
    this->encoderA = encoderA;
    pinMode(pwmPin, OUTPUT);
    pinMode(digitalOne, OUTPUT);
    pinMode(digitalTwo, OUTPUT);

    	//no comentar
    //initEncoder();
} 

void Motor::deltaEncoderTics(int deltaTics) {
    ticsCounter += deltaTics;
}

void Motor::deltaPidTics(int deltaTics) {
    pidTics += deltaTics;
}

void Motor:: motorForward(const uint8_t pwm) {
    analogWrite(pwmPin, pwm);
    /* if (currentState == MotorState::Forward)
    {
        return;
    } */
    //Serial.println("Forward");
    
    digitalWrite(digitalOne, HIGH);
    digitalWrite(digitalTwo, LOW);

    currentState = MotorState::Forward;
}

void Motor:: motorBackward(uint8_t pwm) {
    analogWrite(pwmPin, pwm);
    /* if (currentState == MotorState::Backward)
    {
        return;
    } */
    //Serial.println("Backward");
    
    digitalWrite(digitalOne, LOW);
    digitalWrite(digitalTwo, HIGH);


    currentState = MotorState::Backward;
}

void Motor:: motorStop() {
    pwm=0;
    analogWrite(pwmPin, pwm);

    /* if (currentState == MotorState::Stop)
    {
        return;
    } */

    digitalWrite(digitalOne, LOW);
    digitalWrite(digitalTwo, LOW);

    currentState = MotorState::Stop;
}

double Motor::getTargetRps(double velocity) {
   return MsToRps(velocity);
}

double Motor::RpmToRps(double velocity) {
    return velocity / 60.00;
}

double Motor::MsToRps(double ms) {

    // Warning: the 1000.00 supposes the distance per rev but it isn't the case, is necesary to change it
    return ms / 1000.00;
}


int Motor::getPWM(){
    return pwm;
}

double Motor::getDistanceTraveled() {
    return (getEncoderTics() / kPulsesPerRev) * (DistancePerRev);
}

void Motor:: setEncoderTics(int tics) {
    ticsCounter = tics;
}



// CHECK IF THIS IS FUNCTIONAL OR NOT, AND IF YES, SEE HOW TO IMPLEMENT THE OTHER ONE

/* void Motor::setPID(double targetSpeed,double kP, double kI, double kD){
    updateRPM();
    double error = targetSpeed - rpm;
    errorAcumulado = error + errorAcumulado;
    // pwmInicial = pwmInicial + (kP * error + kI * (errorAcumulado) + kD * (error - errorPrev));
    pwmInicial = kP * error + kI * errorAcumulado + kD * (error - errorPrev);
    errorPrev = error;
    if (pwmInicial > 255)
        pwmInicial = 255;
    else if (pwmInicial < 50)
        pwmInicial = 50;
    setPWM(pwmInicial);
    
} */


double Motor::getRPM() {
    return rpm;
}

