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
    this->motorId = MotorID::NONE;
}
void Motor::motoresSetup(uint8_t pwmPin, uint8_t digitalOne, uint8_t digitalTwo, uint8_t encoderA, MotorID motorid){
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
void Motor::setPWM(uint8_t pwmX){
    pwm = pwmX;
    analogWrite(pwmPin, pwm);
    digitalWrite(digitalOne, HIGH);
    digitalWrite(digitalTwo, LOW);
}
void Motor::initEncoder(){
    pinMode(encoderA, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(encoderA),updateTics, RISING);
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
//volatile int Motor::encoderTics = 0;
volatile int Motor::encoderTicsFL = 0;
volatile int Motor::encoderTicsFR = 0;
volatile int Motor::encoderTicsBL = 0;
volatile int Motor::encoderTicsBR = 0;
/*static void Motor::updateTics(){
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
/*int Motor::getEncoderTicsFR(){
    return encoderTicsFR;
}
int Motor::getEncoderTicsBL(){
    return encoderTicsBL;
}
int Motor::getEncoderTicsBR(){
    return encoderTicsBR;
}*/
void Motor::updateRPM(){
    if(millis()-next_time>=100){ // si divido a 1000, multiplico abajo
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
        rpm = 10*(tics * 60.00) / 500.00; //50 por lo q dividi
        /*encoderTicsBL = 0;
        encoderTicsBR = 0;
        encoderTicsFL = 0;
        encoderTicsFR = 0;*/
        next_time = millis();
    }
    return;
}
void Motor::setPID(float targetSpeed,float kp, float ki, float kd){
    updateRPM();
    float error = targetSpeed - rpm;
    errorAcumulado = error + errorAcumulado;
    //pwmInicial = pwmInicial + (kp * error + ki * (errorAcumulado) + kd * (error - errorPrev));
    pwmInicial = kp * error + ki * errorAcumulado + kd * (error - errorPrev);
    errorPrev = error;
    if(pwmInicial>255)
        pwmInicial = 255;
    else if(pwmInicial<50)
        pwmInicial = 50;
    setPWM(pwmInicial);
    
    /*if(motorId==MotorID::FRONT_LEFT){
        Serial.print(pwmInicial);
        Serial.print(" ");
        Serial.print(rpm);
        Serial.print(" ");
        Serial.println(error);
    }*/
}

float Motor::getPWM(){
    return pwm;
}
float Motor::getRPM(){
    return rpm;
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
    //return encoderTics;
}