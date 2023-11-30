#include "Arduino.h"
#include "Motor.h"

Motor::Motor(){
    this->pwmPin = 0;
    this->digitalOne = 0;
    this->digitalTwo = 0;
    this->encoderA = 0;
    this->rpm = 0;
    this->next_time = millis();
    this->pwmInicial = 50;
    this->errorPrev = 0;
    this->errorAcumulado = 0;
}
void Motor::motoresSetup(uint8_t pwmPin, uint8_t digitalOne, uint8_t digitalTwo, uint8_t encoderA, MotorID motorId){
    this->pwmPin = pwmPin;
    this->digitalOne = digitalOne;
    this->digitalTwo = digitalTwo;
    this->motorId = motorId;
    this->encoderA = encoderA;
    pinMode(pwmPin, OUTPUT);
    pinMode(digitalOne, OUTPUT);
    pinMode(digitalTwo, OUTPUT);

    initEncoder();
}
void Motor::setPWM(uint8_t pwm){
    this->pwm = pwm;
    analogWrite(pwmPin, pwm);
    digitalWrite(digitalOne, HIGH);
    digitalWrite(digitalTwo, LOW);
}
void Motor::initEncoder(){
    pinMode(encoderA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderA),updateTics, RISING);
    /*switch (motorId)
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
    }*/
    return;
}
volatile int Motor::encoderTics = 0;
/*volatile int Motor::encoderTicsFR = 0;
volatile int Motor::encoderTicsBL = 0;
volatile int Motor::encoderTicsBR = 0;*/
static void Motor::updateTics(){
    encoderTics++;
    return;
}
/*static void Motor::updateTicsFR(){
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
}*/
int Motor::getEncoderTics(){
    return encoderTics;
}
/*int Motor::getEncoderTicsFR(){
    return encoderTicsFR;
}
int Motor::getEncoderTicsBL(){
    return encoderTicsBL;
}
int Motor::getEncoderTicsBR(){
    return encoderTicsBR;
}*/
/*motor::setPWM(uint8_t pwm){
    this->pwm = pwm;
    analogWrite(pwmPin, pwm);
}*/
void Motor::updateRPM(){
    if(millis()-next_time>=100){ // si divido a 1000, multiplico abajo
        rpm = 10*(encoderTics * 60.00) / 500.00;
        encoderTics = 0;
        next_time = millis();
    }
    return;
}
void Motor::setPID(float targetSpeed,float kp, float ki, float kd){
    updateRPM();
    float error = targetSpeed - rpm;
    errorAcumulado = error + errorAcumulado;
    pwmInicial = (kp * error + ki * (errorAcumulado) + kd * (error - errorPrev));
    errorPrev = error;
    if(pwmInicial>255)
        pwmInicial = 255;
    else if(pwmInicial<0)
        pwmInicial = 0;
    setPWM(pwmInicial);
}

float Motor::getRPM(){
    return rpm;
}

// hacer un metodo que devuelva el pwm inicial

float Motor::getPWMInicial(){
    return pwmInicial;
}



