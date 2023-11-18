#include "Arduino.h"
#include "Motor.h"

Motor::Motor(){
    this->pwmPin = 0;
    this->digitalOne = 0;
    this->digitalTwo = 0;
    this->encoderA = 0;
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
/*static void motor::a(motor* motora){
    encoder::updateTics(motora);
    return;
}*/
void Motor::initEncoder(){
    pinMode(encoderA, INPUT_PULLUP);
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
    return;
}
volatile int Motor::encoderTicsFL = 0;
volatile int Motor::encoderTicsFR = 0;
volatile int Motor::encoderTicsBL = 0;
volatile int Motor::encoderTicsBR = 0;
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
int Motor::getEncoderTicsFL(){
    return encoderTicsFL;
}
int Motor::getEncoderTicsFR(){
    return encoderTicsFR;
}
int Motor::getEncoderTicsBL(){
    return encoderTicsBL;
}
int Motor::getEncoderTicsBR(){
    return encoderTicsBR;
}
/*motor::setPWM(uint8_t pwm){
    this->pwm = pwm;
    analogWrite(pwmPin, pwm);
}*/
