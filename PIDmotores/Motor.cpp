#include "Motor.h"
#include "Encoder.h"
#include "Pins.h"

Motor::Motor() {
    this->pwmPin_ = 0;
    this->digitalOne_ = 0;
    this->digitalTwo_ = 0;
    this->encoderA_ = 0;
    this->rpm_ = 0;
    this->nextTime_ = millis();
    this->pwmInicial_ = 50;
    this->errorPrev_ = 0;
    this->errorAcumulado_ = 0;
    this->motorId_ = MotorID::kNone;
    this->pid_.setTunningsMotors(kP_, kI_, kD_, minOutput_, maxOutput_, maxErrorSum_, sampleTime_);
}

Motor::Motor(const uint8_t digitalOne, const uint8_t digitalTwo, const uint8_t pwmPin, const uint8_t encoderA, const MotorID motorid) {
    this->pwmPin_ = pwmPin;
    this->digitalOne_ = digitalOne;
    this->digitalTwo_ = digitalTwo;
    this->encoderA_ = encoderA;
    this->motorId_ = motorid;
    this->encoders_[0] = 0;
    this->encoders_[1] = 0;
    this->encoders_[2] = 0;
    this->encoders_[3] = 0;
    this->pid_.setTunningsMotors(kP_, kI_, kD_, minOutput_, maxOutput_, maxErrorSum_, sampleTime_);
}

uint8_t Motor::getEncoderA() {
    return encoderA_;
}

MotorState Motor::getCurrentState() {
    return currentState_;
}

long long Motor::getTotalTics() {
    return totalTics_;
}

long long Motor::getEpochTics() {
    return timeEpochTics_;
}

void Motor::initMotor() {
    motorSetup(pwmPin_, digitalOne_, digitalTwo_, encoderA_, motorId_);
    motorStop(0);
}

void Motor::initEncoder() {
    pinMode(encoderA_, INPUT_PULLUP);
    
    switch (motorId_) {

        case (MotorID::kFrontLeft):{
            attachInterrupt(digitalPinToInterrupt(encoderA_), Encoder::frontLeftEncoder, RISING);
            break;
        }
        case (MotorID::kFrontRight): {
            attachInterrupt(digitalPinToInterrupt(encoderA_), Encoder::frontRightEncoder, RISING);
            break;
        }
        case (MotorID::kBackLeft): {
            attachInterrupt(digitalPinToInterrupt(encoderA_), Encoder::backLeftEncoder, RISING);
            break;
        }
        case (MotorID::kBackRight): {
            attachInterrupt(digitalPinToInterrupt(encoderA_), Encoder::backRightEncoder, RISING);
            break;
        }
        default: { 
            break;
        }
    }
} 

void Motor::motorSetup(const uint8_t pwmPin, const uint8_t digitalOne, const uint8_t digitalTwo, const uint8_t encoderA, const MotorID motorid) {
    this->pwmPin_ = pwmPin;
    this->digitalOne_ = digitalOne;
    this->digitalTwo_ = digitalTwo;
    this->motorId_ = motorid;
    this->encoderA_ = encoderA;
    pinMode(pwmPin_, OUTPUT);
    pinMode(digitalOne_, OUTPUT);
    pinMode(digitalTwo_, OUTPUT);
    initEncoder();
} 

void Motor::deltaTotalTics(const int deltaTics) {
    totalTics_ += deltaTics;
}

void Motor::deltaTics(const int deltaTics) {
    timeEpochTics_ += deltaTics;
}

void Motor::motorForward(const uint8_t pwm) {
    analogWrite(pwmPin_, pwm);
    if (currentState_ == MotorState::kForward) {
        return;
    } 
    
    digitalWrite(digitalOne_, HIGH);
    digitalWrite(digitalTwo_, LOW);

    currentState_ = MotorState::kForward;
}

void Motor::motorBackward(uint8_t pwm) {
    analogWrite(pwmPin_, pwm);
    if (currentState_ == MotorState::kBackward) {
        return;
    }
    
    digitalWrite(digitalOne_, LOW);
    digitalWrite(digitalTwo_, HIGH);

    currentState_ = MotorState::kBackward;
}

void Motor::motorStop(uint8_t pwm) {
    pwm = 0;
    analogWrite(pwmPin_, pwm);

    if (currentState_ == MotorState::kStop) {
        return;
    } 

    digitalWrite(digitalOne_, LOW);
    digitalWrite(digitalTwo_, LOW);

    previousTics_ = totalTics_;
    currentSpeed_ = 0;
    currentState_ = MotorState::kStop;
}


void Motor::setPwmAndDirection(const uint8_t pwm, const MotorState direction) {
    if (direction == MotorState::kForward) {
        motorForward(pwm);
    } else if (direction == MotorState::kBackward) {
        motorBackward(pwm);
    } else {
        motorStop(0);
    }
}

void Motor::setSpeedAndDirection(const double speed, const MotorState direction) {
    if (direction == MotorState::kStop) {
        motorStop(0);
        return;
    } 
    const uint8_t pwm = speedToPwm(speed);
    if (direction == MotorState::kForward) {
        motorForward(pwm);
    } else if (direction == MotorState::kBackward) {
        motorBackward(pwm);
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

uint8_t Motor::speedToPwm(const double speed) {
    double tmpPwm = 0;
    pid_.compute(speed, currentSpeed_, tmpPwm, timeEpochTics_, &ticsToSpeed);

    return tmpPwm;
}

void Motor::constantSpeed(const double speed, const MotorState direction) {
    double tmpPwm = 0;
    pid_.compute(speed, currentSpeed_, tmpPwm, timeEpochTics_, &ticsToSpeed);
    setPwmAndDirection(tmpPwm, direction);
}

bool Motor::hasTraveledDistance(const double distance) {
    const double distanceInMeters = distance;
    const double distanceInPulses = distanceInMeters * kPulsesPerRev / kDistancePerRev;
    const long long distanceInTics = static_cast<long long>(distanceInPulses);
    const long long currentTics = totalTics_ - previousTics_;
    
    return currentTics >= distanceInTics;
}