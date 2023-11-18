#include "Arduino.h"
#include "Movement.h"
#include "Motor.h"
#include "Pines.h"

Movement::Movement(){
    this->motorFL = Motor();
    this->motorFR = Motor();
    this->motorBL = Motor();
    this->motorBR = Motor();
    this->RMPFL = 0;
    this->RMPFR = 0;
    this->RMPBL = 0;
    this->RMPBR = 0;
    this->FLticsViejos = 0;
    this->FRticsViejos = 0;
    this->BLticsViejos = 0;
    this->BRticsViejos = 0;
    this->FLtics = 0;
    this->FRtics = 0;
    this->BLtics = 0;
    this->BRtics = 0;
    this->next_time = millis();
    this->kp = 0.01;
    this->ki = 0.001;
    this->kd = 0.001;
    this->pwmInicial = 50;
    this->errorPrev = 0;
}
void Movement::setup(){
    motorFL.motoresSetup(pwmFL, daFL, dbFL, eaFL, MotorID::FRONT_LEFT); 
    motorFR.motoresSetup(pwmFR, daFR, dbFR, eaFR, MotorID::FRONT_RIGHT); 
    motorBL.motoresSetup(pwmBL, daBL, dbBL, eaBL, MotorID::BACK_LEFT); 
    motorBR.motoresSetup(pwmBR, daBR, dbBR, eaBR, MotorID::BACK_RIGHT);
}
void Movement::moveForward(int pwmA, int pwmB, int pwmC, int pwmD){
    motorFL.setPWM(pwmA);
    motorFR.setPWM(pwmB);
    motorBL.setPWM(pwmC);
    motorBR.setPWM(pwmD);
}
//tics actuales * 60 / tics por vuelta
void Movement::updateRPM(){
    if(millis()-next_time>=1000){
        FLtics=motorFL.getEncoderTicsFL()-FLticsViejos;
        FRtics=motorFR.getEncoderTicsFR()-FRticsViejos;
        BLtics=motorBL.getEncoderTicsBL()-BLticsViejos;
        BRtics=motorBR.getEncoderTicsBR()-BRticsViejos;
        RMPFL = (FLtics * 60.00) / 500.00;
        RMPFR = (FRtics * 60.00) / 500.00;
        RMPBL = (BLtics * 60.00) / 500.00;
        RMPBR = (BRtics * 60.00) / 500.00;
        FLticsViejos = motorFL.getEncoderTicsFL();
        FRticsViejos = motorFR.getEncoderTicsFR();
        BLticsViejos = motorBL.getEncoderTicsBL();
        BRticsViejos = motorBR.getEncoderTicsBR();
        next_time = millis();
    }
}
void Movement::setSpeed(float targetSpeed){
    updateRPM();
    float error = targetSpeed - RMPFL;
    pwmInicial = pwmInicial + (kp * error + ki * (error + errorPrev) + kd * (error - errorPrev));
    errorPrev = error;
    if(pwmInicial>255)
        pwmInicial = 255;
    else if(pwmInicial<0)
        pwmInicial = 0;
    motorFL.setPWM(pwmInicial);

    /*error = targetSpeed - RMPFR;
    pwm = kp * error;
    if(pwm>255)
        pwm = 255;
    else if(pwm<0)
        pwm = 0;
    motorFR.setPWM(pwm);

    error = targetSpeed - RMPBL;
    pwm = kp * error;
    if(pwm>255)
        pwm = 255;
    else if(pwm<0)
        pwm = 0;
    motorBL.setPWM(pwm);

    error = targetSpeed - RMPBR;
    pwm = kp * error;
    if(pwm>255)
        pwm = 255;
    else if(pwm<0)
        pwm = 0;
    motorBR.setPWM(pwm);*/

    Serial.print("FL: ");
    Serial.print(pwmInicial);
    Serial.print("\t rpm: ");
    Serial.print(RMPFL);
    Serial.print("\t targetSpeed: ");
    Serial.print(targetSpeed);
    Serial.print("\t error: ");
    Serial.print(error);
    Serial.println();
}
Motor Movement::getMotorFL(){
    return motorFL;
}
Motor Movement::getMotorFR(){
    return motorFR;
}
Motor Movement::getMotorBL(){
    return motorBL;
}
Motor Movement::getMotorBR(){
    return motorBR;
}
float Movement::getRMPFL(){
    return RMPFL;
}
float Movement::getRMPFR(){
    return RMPFR;
}
float Movement::getRMPBL(){
    return RMPBL;
}
float Movement::getRMPBR(){
    return RMPBR;
}