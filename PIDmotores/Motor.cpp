#include "Arduino.h"
#include "Motor.h"

Motor::Motor(){
    this-> pwmPin = 0;
    this-> digitalOne = 0;
    this-> digitalTwo = 0;
    this-> encoderA = 0;
    this-> rpm = 0;
    this-> next_time = millis();
    this-> pwmInicial = 50;
    this-> errorPrev = 0;
    this-> errorAcumulado = 0;
    this-> motorId = MotorID::NONE;
}

Motor::Motor(uint8_t digitalOne, uint8_t digitalTwo, uint8_t pwmPin, uint8_t encoderA, MotorID motorid){
    this-> pwmPin = pwmPin;
    this-> digitalOne = digitalOne;
    this-> digitalTwo = digitalTwo;
    this-> encoderA = encoderA;
    this-> motorId = motorid;
}

uint8_t Motor::getEncoderA(){
    return encoderA;
}

int Motor::getEncoderTics(){
    switch (motorId)
    {
    case (MotorID::FRONT_LEFT):
        return encoderTicsFL;
        break;
    case (MotorID::FRONT_RIGHT):
        return encoderTicsFR;
        break;
    case (MotorID::BACK_LEFT):
        return encoderTicsBL;
        break;
    case (MotorID::BACK_RIGHT):
        return encoderTicsBR;
        break;
    default:
        break;
    }
    return encoderTics;
}

double Motor::getCurrentSpeed(){
    return currentSpeed;
}

double Motor::getTargetSpeed(){
    return RpmToRps(targetSpeed);
}

int Motor::getPidTics(){
    return pidTics;
}



void Motor::motoresSetup(uint8_t pwmPin, uint8_t digitalOne, uint8_t digitalTwo, uint8_t encoderA, MotorID motorid){
    this-> pwmPin = pwmPin;
    this-> digitalOne = digitalOne;
    this-> digitalTwo = digitalTwo;
    this-> motorId = motorid;
    this-> encoderA = encoderA;
    pinMode(pwmPin, OUTPUT);
    pinMode(digitalOne, OUTPUT);
    pinMode(digitalTwo, OUTPUT);

    initEncoder();
}
void Motor::setPWM(uint8_t pwm){
    this->pwm = pwm;
    switch (currentState)
    {
    case (MotorState::FORWARD):
        motorForward();
        break;
    case (MotorState::BACKWARD):
        motorBackward();
        break;
    case (MotorState::STOP):
        motorStop();
        break;
    default:
        break;
    }
}
void Motor::initEncoder(){
    pinMode(encoderA, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(encoderA),updateTics, RISING);
    switch (motorId)
    {
    case (MotorID::FRONT_LEFT):
        attachInterrupt(digitalPinToInterrupt(encoderA),updateTicsFL, RISING);
        break;
    case (MotorID::FRONT_RIGHT):
        attachInterrupt(digitalPinToInterrupt(encoderA),updateTicsFR, RISING);
        break;
    case (MotorID::BACK_LEFT):
        attachInterrupt(digitalPinToInterrupt(encoderA),updateTicsBL, RISING);
        break;
    case (MotorID::BACK_RIGHT):
        attachInterrupt(digitalPinToInterrupt(encoderA),updateTicsBR, RISING);
        break;
    default:
        break;
    }
}


void Motor:: motorForward(){
    analogWrite(pwmPin, pwm);
    if (currentState == MotorState::FORWARD)
    {
        return;
    }
    
    digitalWrite(digitalOne, HIGH);
    digitalWrite(digitalTwo, LOW);

    pidStraight.reset();
    pidRotate.reset();

    currentState = MotorState::FORWARD;
}

void Motor:: motorBackward(){
    analogWrite(pwmPin, pwm);
    if (currentState == MotorState::BACKWARD)
    {
        return;
    }
    
    digitalWrite(digitalOne, LOW);
    digitalWrite(digitalTwo, HIGH);

    pidStraight.reset();
    pidRotate.reset();

    currentState = MotorState::BACKWARD;
}

void Motor:: motorStop(){
    analogWrite(pwmPin, LOW);

    if (currentState == MotorState::STOP)
    {
        return;
    }

    digitalWrite(digitalOne, LOW);
    digitalWrite(digitalTwo, LOW);

    pidStraight.reset();
    pidRotate.reset();

    currentState = MotorState::STOP;
}

double Motor::getTargetRps(double velocity){
   return MsToRps(velocity);
}

double Motor::RpmToRps(double velocity){
    return velocity / 60.00;
}

double Motor::MsToRps(double ms){

    // Warning: the 1000.00 supposes the distance per rev but it isn't the case, is necesary to change it
    return ms / 1000.00;
}

void Motor::deltaPidTics(int deltaTics){
    pidTics += deltaTics;
}

void Motor:: motorSpeedPID(double targetSpeed_, bool debug){
    int speedSign_ = min(1, max(-1, targetSpeed_ * 1000));
    this->targetSpeed = fabs(targetSpeed_);
    double pwm_ = pwm;
    switch (currentState)
    {
    case (0):
        motorStop();
        break;
    case (1):
        motorForward();
        break;
    case (-1):
        motorBackward();
        break;
    }
    
    pidStraight.computeSpeed(RpmToRps(targetSpeed), currentSpeed, pwm_, pidTics, kPulsesPerRev, kPidCountTimeSampleInSec ,debug);
    setPWM(pwm_);
}

void Motor::motorRotateIzqPID(double targetAngle, double currentAngle){
    double pwm_ = pwm;
    pidRotate.computeRotateIzq(targetAngle, currentAngle, pwm_);
    pwm_ = fabs(pwm_);
    if (pwm_ < 70) {
        pwm_ = 70;
    }
    else if (pwm_ > 255) {
        pwm_ = 255;
    }
    setPWM(pwm_);

}

void Motor::motorRotateDerPID(double targetAngle, double currentAngle){
    double pwm_ = pwm;
    pidRotate.computeRotateDer(targetAngle, currentAngle, pwm_);
    pwm_ = fabs(pwm_);
    if (pwm_ < 70) {
        pwm_ = 70;
    }
    else if (pwm_ > 255) {
        pwm_ = 255;
    }
    setPWM(pwm_);
}

int Motor::getPWM(){
    return pwm;
}

void Motor::motorSpeedPWM(double targetSpeed_){
    int speedSign_ = min(1, max(-1, targetSpeed_ * 1000));
    this->targetSpeed = fabs(targetSpeed_);
    pwm = targetSpeed;
    switch (speedSign_)
    {
    case (0):
        motorStop();
        break;
    case (1):
        motorForward();
        break;
    case (-1):
        motorBackward();
        break;
    }
}

void Motor:: setEncoderTics(int tics){
    ticsCounter = tics;
}

void Motor::PIDStraightTunings(double kp, double ki, double kd){
    pidStraight.setTunings(kp, ki, kd);
}

void Motor::PIDRotateTunings(double kp, double ki, double kd){
    pidRotate.setTunings(kp, ki, kd);
}


// volatile int Motor::encoderTics = 0;
volatile int Motor::encoderTicsFL = 0;
volatile int Motor::encoderTicsFR = 0;
volatile int Motor::encoderTicsBL = 0;
volatile int Motor::encoderTicsBR = 0;
/*  static void Motor::updateTics(){
      encoderTics++;
      return;
}*/
static void Motor::updateTicsFL(){
    encoderTicsFL++;
    return;
}

static void Motor::updateTicsFR(){
    encoderTicsFR++;
    return;
}

static void Motor::updateTicsBL(){
    encoderTicsBL++;
    return;
}

static void Motor::updateTicsBR(){
    encoderTicsBR++;
    return;
}

/*
int Motor::getEncoderTicsFR(){
    return encoderTicsFR;
}
int Motor::getEncoderTicsBL(){
    return encoderTicsBL;
}
int Motor::getEncoderTicsBR(){
    return encoderTicsBR;
}
*/
void Motor::updateRPM(){
    if (millis()-next_time>=100){ // si divido a 1000, multiplico abajo
        int tics=-1;
        switch (motorId)
        {
        case (MotorID::FRONT_LEFT):
            tics=encoderTicsFL;
            encoderTicsFL = 0;
            break;
        case (MotorID::FRONT_RIGHT):
            tics=encoderTicsFR;
            encoderTicsFR = 0;
            break;
        case (MotorID::BACK_LEFT):
            tics=encoderTicsBL;
            encoderTicsBL = 0;
            break;
        case (MotorID::BACK_RIGHT):
            tics=encoderTicsBR;
            encoderTicsBR = 0;
            break;
        default:
            break;
        }
        rpm = 10*(tics * 60.00) / 500.00; // 50 por lo q dividi

        /*
        encoderTicsBL = 0;
        encoderTicsBR = 0;
        encoderTicsFL = 0;
        encoderTicsFR = 0;
        */
        next_time = millis();
    }
    return;
}

// CHECK IF THIS IS FUNCTIONAL OR NOT, AND IF YES, SEE HOW TO IMPLEMENT THE OTHER ONE

/* void Motor::setPID(double targetSpeed,double kp, double ki, double kd){
    updateRPM();
    double error = targetSpeed - rpm;
    errorAcumulado = error + errorAcumulado;
    // pwmInicial = pwmInicial + (kp * error + ki * (errorAcumulado) + kd * (error - errorPrev));
    pwmInicial = kp * error + ki * errorAcumulado + kd * (error - errorPrev);
    errorPrev = error;
    if (pwmInicial > 255)
        pwmInicial = 255;
    else if (pwmInicial < 50)
        pwmInicial = 50;
    setPWM(pwmInicial);
    
} */


double Motor::getRPM(){
    return rpm;
}

