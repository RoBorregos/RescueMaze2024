#include "Motor.h"
#include "Encoder.h"
#include "Pins.h"

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
    this->motorId = MotorID::kNone;
    this->pid.setTunnings(150, 100, 0.0, 0, 255, 4000, 100);
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
    return currentState_;
}

long long Motor::getTotalTics() {
    return totalTics;
}

long long Motor::getEpochTics() {
    return timeEpochTics;
}

void Motor::initMotor() {
    motorSetup(pwmPin, digitalOne, digitalTwo, encoderA, motorId);
    //initEncoder();
    motorStop();
}

void Motor::initEncoder() {
    pinMode(encoderA, INPUT_PULLUP);
    
    switch (motorId) {

        case (MotorID::kFrontLeft):{
            attachInterrupt(digitalPinToInterrupt(encoderA), Encoder::frontLeftEncoder, RISING);
            break;
        }
        case (MotorID::kFrontRight): {
            attachInterrupt(digitalPinToInterrupt(encoderA), Encoder::frontRightEncoder, RISING);
            break;
        }
        case (MotorID::kBackLeft): {
            attachInterrupt(digitalPinToInterrupt(encoderA), Encoder::backLeftEncoder, RISING);
            break;
        }
        case (MotorID::kBackRight): {
            attachInterrupt(digitalPinToInterrupt(encoderA), Encoder::backRightEncoder, RISING);
            break;
        }
        default: { 
            break;
        }
    }
} 

void Motor::motorSetup(const uint8_t pwmPin, const uint8_t digitalOne, const uint8_t digitalTwo, const uint8_t encoderA, const MotorID motorid) {
    this->pwmPin = pwmPin;
    this->digitalOne = digitalOne;
    this->digitalTwo = digitalTwo;
    this->motorId = motorid;
    this->encoderA = encoderA;
    pinMode(pwmPin, OUTPUT);
    pinMode(digitalOne, OUTPUT);
    pinMode(digitalTwo, OUTPUT);
    initEncoder();
} 

void Motor::deltaTotalTics(const int deltaTics) {
    totalTics += deltaTics;
}

void Motor::deltaTics(const int deltaTics) {
    timeEpochTics += deltaTics;
}

void Motor:: motorForward(const uint8_t pwm) {
    analogWrite(pwmPin, pwm);
    if (currentState_ == MotorState::kForward) {
        return;
    } 
    //Serial.println("Forward");
    
    digitalWrite(digitalOne, HIGH);
    digitalWrite(digitalTwo, LOW);

    currentState_ = MotorState::kForward;
}

void Motor:: motorBackward(uint8_t pwm) {
    analogWrite(pwmPin, pwm);
    if (currentState_ == MotorState::kBackward) {
        return;
    }
    //Serial.println("Backward");
    
    digitalWrite(digitalOne, LOW);
    digitalWrite(digitalTwo, HIGH);


    currentState_ = MotorState::kBackward;
}

void Motor:: motorStop() {
    pwm = 0;
    analogWrite(pwmPin, pwm);

    if (currentState_ == MotorState::kStop) {
        return;
    } 

    digitalWrite(digitalOne, LOW);
    digitalWrite(digitalTwo, LOW);

    currentState_ = MotorState::kStop;
}

double Motor::getRPM() {
    return rpm;
}

void Motor::setPwmAndDirection(const uint8_t pwm, const MotorState direction) {
    if (direction == MotorState::kForward) {
        motorForward(pwm);
    } else if (direction == MotorState::kBackward) {
        motorBackward(pwm);
    } else {
        motorStop();
    }
}

double Motor::getSpeed() {
    return currentSpeed_;
}

static double ticsToSpeed(const long long tics, const unsigned long time) {
    const double timeDouble = static_cast<double>(time);
    const double timeInSec = timeDouble / kOneSecInMs;
    const double deltaTics = tics;
    const double deltaRev = deltaTics / kPulsesPerRev;
    const double deltaMeters = deltaRev * kDistancePerRev;
    const double deltaMetersPerSecond = deltaMeters / timeInSec;
    return deltaMetersPerSecond;
}

double Motor::ticsToMs() {
    const unsigned long currentTime = millis();
    const unsigned long deltaTime = currentTime - previousTime;
    double speed = 0;
    
    if (deltaTime > kOneSecInMs) {
        previousTime = currentTime;
        speed = ticsToSpeed(timeEpochTics, deltaTime);
        timeEpochTics = 0; 
        currentSpeed_ = speed;
    }
    return currentSpeed_;
}

void Motor::constantSpeed(const double speed) {
    double tmpPwm = pwm;
    pid.compute(speed, currentSpeed_, tmpPwm, timeEpochTics, &ticsToSpeed);
    setPwmAndDirection(tmpPwm, MotorState::kForward);
}