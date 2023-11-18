#include "motor.h"
#include "encoder.h"

motor::motor(){
    this->pwmPin = 0;
    this->digitalOne = 0;
    this->digitalTwo = 0;
    this->encoderA = 0;
}
motor::motoresSetup(uint8_t pwmPin, uint8_t digitalOne, uint8_t digitalTwo, uint8_t encoderA){
    this->pwmPin = pwmPin;
    this->digitalOne = digitalOne;
    this->digitalTwo = digitalTwo;
    //this->motorID = motorID;
    pinMode(pwmPin, OUTPUT);
    pinMode(digitalOne, OUTPUT);
    pinMode(digitalTwo, OUTPUT);
}
void motor::initEncoder(uint8_t encoderA){
    this->encoderA = encoderA;
    attachInterrupt(digitalPinToInterrupt(encoderA), encoder::updateTics, RISING);
}
void motor::updateTics(){
    this->encoderTics += 1;
    return;
}
/*motor::setPWM(uint8_t pwm){
    this->pwm = pwm;
    analogWrite(pwmPin, pwm);
}*/
