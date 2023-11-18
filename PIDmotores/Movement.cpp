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
}
void Movement::setup(){
    motorFL.motoresSetup(pwmFL, daFL, dbFL, eaFL, MotorID::FRONT_LEFT); 
    motorFR.motoresSetup(pwmFR, daFR, dbFR, eaFR, MotorID::FRONT_RIGHT); 
    motorBL.motoresSetup(pwmBL, daBL, dbBL, eaBL, MotorID::BACK_LEFT); 
    motorBR.motoresSetup(pwmBR, daBR, dbBR, eaBR, MotorID::BACK_RIGHT);
}
void Movement::moveForward(int pwm){
    motorFL.setPWM(pwm);
    motorFR.setPWM(pwm);
    motorBL.setPWM(pwm);
    motorBR.setPWM(pwm);
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
//tics actuales * 60 / tics por vuelta
void Movement::updateRPM(){
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