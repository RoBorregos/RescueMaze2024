#include "Arduino.h"
#include "Movement.h"
#include "Motor.h"
#include "Pines.h"
#include "BNO.h"

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
    this->pwmInicialFL = 50;
    this->errorPrevFL = 0;
    this->pwmInicialFR = 50;
    this->errorPrevFR = 0;
    this->pwmInicialBL = 50;
    this->errorPrevBL = 0;
    this->pwmInicialBR = 50;
    this->errorPrevBR = 0;
    this->errorPrevOrientation = 0;
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
void Movement::setSpeed(float targetSpeed,float orientation,BNO bno){
    //PID orientation
    float errorOrientation = orientation - bno.getOrientationX();
    float Kp = 0.2; //AJUSTAR
    float Ki = 0.05;
    float Kd = 0.01;
    if(errorOrientation>300){
        errorOrientation=orientation-(360+bno.getOrientationX());
    }
    if(errorOrientation<-300){
        errorOrientation=(bno.getOrientationX()-(360+orientation))*-1;
    }
    float targetSpeedRight = targetSpeed - (Kp * errorOrientation + Ki * (errorOrientation + errorPrevOrientation) + Kd * (errorOrientation - errorPrevOrientation));
    float targetSpeedLeft = targetSpeed + (Kp * errorOrientation + Ki * (errorOrientation + errorPrevOrientation) + Kd * (errorOrientation - errorPrevOrientation));
    errorPrevOrientation = errorOrientation;
    if(targetSpeedLeft>255)
        targetSpeedLeft = 255;
    else if(targetSpeedLeft<0)
        targetSpeedLeft = 0;
    if(targetSpeedRight>255)
        targetSpeedRight = 255;
    else if(targetSpeedRight<0)
        targetSpeedRight = 0;
    Serial.print("targetSpeedLeft: ");
    Serial.print(targetSpeedLeft);
    Serial.print("\t targetSpeedRight: ");
    Serial.print(targetSpeedRight);
    Serial.print("\t angulo: ");
    Serial.print(bno.getOrientationX());
    Serial.println();
    //PID velocidades
    updateRPM();
    float error = targetSpeedLeft - RMPFL;
    pwmInicialFL = pwmInicialFL + (kp * error + ki * (error + errorPrevFL) + kd * (error - errorPrevFL));
    errorPrevFL = error;
    if(pwmInicialFL>255)
        pwmInicialFL = 255;
    else if(pwmInicialFL<0)
        pwmInicialFL = 0;
    motorFL.setPWM(pwmInicialFL);

    error = targetSpeedRight - RMPFR;
    pwmInicialFR = pwmInicialFR + (kp * error + ki * (error + errorPrevFR) + kd * (error - errorPrevFR));
    errorPrevFR = error;
    if(pwmInicialFR>255)
        pwmInicialFR = 255;
    else if(pwmInicialFR<0)
        pwmInicialFR = 0;
    motorFR.setPWM(pwmInicialFR);

    error = targetSpeedLeft - RMPBL;
    pwmInicialBL = pwmInicialBL + (kp * error + ki * (error + errorPrevBL) + kd * (error - errorPrevBL));
    errorPrevBL = error;
    if(pwmInicialBL>255)
        pwmInicialBL = 255;
    else if(pwmInicialBL<0)
        pwmInicialBL = 0;
    motorBL.setPWM(pwmInicialBL);

    error = targetSpeedRight - RMPBR;
    pwmInicialBR = pwmInicialBR + (kp * error + ki * (error + errorPrevBR) + kd * (error - errorPrevBR));
    errorPrevBR = error;
    if(pwmInicialBR>255)
        pwmInicialBR = 255;
    else if(pwmInicialBR<0)
        pwmInicialBR = 0;
    motorBR.setPWM(pwmInicialBR);

    /*Serial.print("FL: ");
    Serial.print(pwmInicialFL);
    Serial.print("\t rpm: ");
    Serial.print(RMPFL);
    Serial.print("\t targetSpeed: ");
    Serial.print(targetSpeed);
    Serial.print("\t error: ");
    Serial.print(error);
    Serial.println();*/
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