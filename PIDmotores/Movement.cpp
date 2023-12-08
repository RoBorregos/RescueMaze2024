#include "Arduino.h"
#include "Movement.h"
#include "Motor.h"
#include "Pines.h"
#include "BNO.h"

// Ajustar
constexpr double Kp = 5.0; 
constexpr double Ki = 0.05;
constexpr double Kd = 0.05;

PID pidStraight(Kp, Ki, Kd);\
BNO bno;



Movement::Movement(){
    this-> motorFL = Motor();
    this-> motorFR = Motor();
    this-> motorBL = Motor();
    this-> motorBR = Motor();
    this->motor[4];
    
    //this->pwm;
    
    /*
    this-> RMPFL = 0;
    this-> RMPFR = 0;
    this-> RMPBL = 0;
    this-> RMPBR = 0;
    this-> FLticsViejos = 0;
    this-> FRticsViejos = 0;
    this-> BLticsViejos = 0;
    this-> BRticsViejos = 0;
    this-> FLtics = 0;
    this-> FRtics = 0;
    this-> BLtics = 0;
    this-> BRtics = 0;
    */

    this-> next_time = millis();
    /* this-> kp = 0.01;
    this-> ki = 0.001;
    this-> kd = 0.001; */

    /*
    this-> pwmInicialFL = 50;
    this-> errorPrevFL = 0;
    this-> pwmInicialFR = 50;
    this-> errorPrevFR = 0;
    this-> pwmInicialBL = 50;
    this-> errorPrevBL = 0;
    this-> pwmInicialBR = 50;
    this-> errorPrevBR = 0;
    */
    this-> errorPrevOrientation = 0;
    this-> errorAcumuladoOrientation = 0;
}

void Movement::setup(){
    setupInternal(MotorID::FRONT_LEFT);
    setupInternal(MotorID::FRONT_RIGHT);
    setupInternal(MotorID::BACK_LEFT);
    setupInternal(MotorID::BACK_RIGHT);
    bno.setupBNO();
}

void Movement::setupInternal(MotorID motorId){
    int index = static_cast<int>(motorId);
    motor[index].motoresSetup(
        pwmPin[index],
        digitalOne[index],
        digitalTwo[index],
        encoderA[index],
        motorId);
}

void Movement::moveForward(int pwmA, int pwmB, int pwmC, int pwmD){
    motorFL.updateRPM();
    motorFR.updateRPM();
    motorBL.updateRPM();
    motorBR.updateRPM();
    motorFL.setPWM(pwmA);
    motorFR.setPWM(pwmB);
    motorBL.setPWM(pwmC);
    motorBR.setPWM(pwmD);
    /*Serial.print(motorFL.getRPM());
    Serial.print(" ");
    Serial.print(motorFR.getRPM());
    Serial.print(" ");
    Serial.print(motorBL.getRPM());
    Serial.print(" ");
    Serial.println(motorBR.getRPM());*/
}

//tics actuales * 60 / tics por vuelta
/*void Movement::updateRPM(){
    if(millis()-next_time>=1000){ //100?
        FLtics=motorFL.getEncoderTics()-FLticsViejos;
        FRtics=motorFR.getEncoderTics()-FRticsViejos;
        BLtics=motorBL.getEncoderTics()-BLticsViejos;
        BRtics=motorBR.getEncoderTics()-BRticsViejos;
        RMPFL = (FLtics * 60.00) / 500.00; // 50?
        RMPFR = (FRtics * 60.00) / 500.00;
        RMPBL = (BLtics * 60.00) / 500.00;
        RMPBR = (BRtics * 60.00) / 500.00;
        FLticsViejos = motorFL.getEncoderTics();
        FRticsViejos = motorFR.getEncoderTics();
        BLticsViejos = motorBL.getEncoderTics();
        BRticsViejos = motorBR.getEncoderTics();
        next_time = millis();
    }
    motorFL.updateRPM();
    motorFR.updateRPM();
    motorBL.updateRPM();
    motorBR.updateRPM();
}*/

void Movement::setSpeed(float targetSpeed,float orientation,BNO bno){
    //PID orientation
    float errorOrientation = orientation - bno.getOrientationX();


    // pwm = PID::getForwardPWM(double targetSpeed, const double kp, const double ki, const double kd) 
    // Motor::setPWM(double pwm)


/* float Kp = 0.2; //AJUSTAR
    float Ki = 0.05;
    float Kd = 0.01; */

    if (errorOrientation > 300) {
        errorOrientation = orientation - (360 + bno.getOrientationX());
    }

    if (errorOrientation < -300) {
        errorOrientation = (bno.getOrientationX() - (360 + orientation)) * -1;
    }
    errorAcumuladoOrientation = errorAcumuladoOrientation + errorOrientation;
    //float targetSpeedRight = targetSpeed - (Kp * errorOrientation + Ki * (errorAcumuladoOrientation) + Kd * (errorOrientation - errorPrevOrientation));
    float targetSpeedRight = - (Kp * errorOrientation + Ki * (errorAcumuladoOrientation) + Kd * (errorOrientation - errorPrevOrientation));
    //float targetSpeedLeft = targetSpeed + (Kp * errorOrientation + Ki * (errorAcumuladoOrientation) + Kd * (errorOrientation - errorPrevOrientation));
    float targetSpeedLeft = (Kp * errorOrientation + Ki * (errorAcumuladoOrientation) + Kd * (errorOrientation - errorPrevOrientation));
    errorPrevOrientation = errorOrientation;

    // TODO: checar espacios de if mas que todo formato
    if (targetSpeedLeft > 255)
        targetSpeedLeft = 255;
    else if (targetSpeedLeft < 0)
        targetSpeedLeft = 0;
    if (targetSpeedRight > 255)
        targetSpeedRight = 255;
    else if (targetSpeedRight < 0)
        targetSpeedRight = 0;
    Serial.print(" ");
    Serial.print(targetSpeedLeft);
    Serial.print(" ");
    Serial.print(targetSpeedRight);
    Serial.print(" ");

    //Serial.print("\t angulo: ");
    //Serial.print(bno.getOrientationX());
    //Serial.println();
    //Serial.print("targetSpeedLeft: ");
    //Serial.print(targetSpeedLeft);
    //Serial.print("\t targetSpeedRight: ");
    //Serial.print(targetSpeedRight);
    //Serial.print("\t angulo: ");
    //Serial.print(bno.getOrientationX());
    //Serial.println();
    //PID velocidades
    //updateRPM();

    //0.01 0.001 0.001
    /*motorFL.setPID(targetSpeedLeft, 0.01, 0.001, 0.001);
    motorFR.setPID(targetSpeedRight, 0.01, 0.001, 0.001);
    motorBL.setPID(targetSpeedLeft, 0.01, 0.001, 0.001);
    motorBR.setPID(targetSpeedRight, 0.01, 0.001, 0.001);*/
    motorFL.setPID(targetSpeed+targetSpeedLeft, 1, 0.005, 0.006); //SEGUIR AJUSTANDO
    motorFR.setPID(targetSpeed+targetSpeedRight, 1, 0.005, 0.006);
    motorBL.setPID(targetSpeed+targetSpeedLeft, 1, 0.005, 0.006);
    motorBR.setPID(targetSpeed+targetSpeedRight, 1, 0.005, 0.006);

    /*float error = targetSpeedLeft - RMPFL;
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
    motorBR.setPWM(pwmInicialBR);*/

    /*Serial.print("FL: ");
    Serial.print(pwmInicialFL);
    Serial.print("\t rpm: ");
    Serial.print(motorFL.getRPM());
    Serial.print("\t targetSpeed: ");
    Serial.print(targetSpeedLeft);
    Serial.print("\t error: ");
    Serial.print(error);
    Serial.println();*/

    Serial.print(motorFL.getRPM());
    Serial.print(" ");
    Serial.print(motorFR.getRPM());
    Serial.print(" ");
    Serial.print(motorBL.getRPM());
    Serial.print(" ");
    Serial.println(motorBR.getRPM());
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

/*float Movement::getRMPFL(){
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
}*/


float Movement::getRPMFL(){
    return motorFL.getRPM();
}

float Movement::getRPMFR(){
    return motorFR.getRPM();
}

float Movement::getRPMBL(){
    return motorBL.getRPM();
}

float Movement::getRPMBR(){
    return motorBR.getRPM();
}

/* float Movement::getPWMInicialFL(){
    return motorFL.getPWMInicial();
}

float Movement::getPWMInicialFR(){
    return motorFR.getPWMInicial();
}

float Movement::getPWMInicialBL(){
    return motorBL.getPWMInicial();
}

float Movement::getPWMInicialBR(){
    return motorBR.getPWMInicial();
} */

void Movement::stopMotors(){
    
    for(int i=0; i<4; i++){
        motor[i].motorStop();
    }
}

void Movement:: forwardMotors(uint8_t pwms[4]){
    for(int i=0; i<4; i++){
        motor[i].motorForward(pwms[i]);
    }
}

void Movement:: backwardMotors(uint8_t pwms[4]){
    for(int i=0; i<4; i++){
        motor[i].motorBackward(pwms[i]);
    }
}

void Movement:: moveMotors(MotorState state){
    uint8_t pwms[4];
    int pwm = 60;
    double targetOrientation = 0;
    double currentOrientation = bno.getOrientationX();
    double pwmLeft = 0;
    double pwmRight = 0;
    switch (state)
    {
    case (MotorState::Stop):
        stopMotors();
        break;
    
    case (MotorState::Forward):
        // 1- SET angulo deseado
        // 2- Llamar a la funcion computeStraight
        // 3- meter al arreglo
        // 4- llamar a forward motors
        pidStraight.computeStraight(targetOrientation,currentOrientation, pwmLeft, pwmRight);

        pwms[static_cast<int>(MotorID::FRONT_LEFT)]= pwmLeft;
        pwms[static_cast<int>(MotorID::BACK_LEFT)]= pwmLeft;
        pwms[static_cast<int>(MotorID::FRONT_RIGHT)]= pwmRight;
        pwms[static_cast<int>(MotorID::BACK_RIGHT)]= pwmRight;

        forwardMotors(pwms);
        break;

    case (MotorState::Backward):
        pwms[0]= pwm;
        pwms[1]= pwm;
        pwms[2]= pwm;
        pwms[3]= pwm;
        backwardMotors(pwms);
        break;
    default :
        break;
    }
    
}

// CALIZZ ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''FEDFE'FREFEJ,FBEJFFBHUWBFUHEWBFHJWERBFHJRBEJHFBRJFBKWRJNSBJFBKREJFBKEJB
void Movement::updateTics(MotorID motorId){

    int index = static_cast<int>(motorId);
    if (index >= 0 && index < 4){
        motor[index].deltaPidTics(1);
        if (motor[index].getCurrentState() == MotorState::Forward){
            motor[index].deltaEncoderTics(1);
        } else if (motor[index].getCurrentState() == MotorState::Backward){
            motor[index].deltaEncoderTics(-1);
        }
        else {
            return;
        }
    }
    /* motor->deltaPidTics(1);

    if (motor->getCurrentState() == MotorState::Forward){
        motor->deltaEncoderTics(1);
    } else if (motor->getCurrentState() == MotorState::Backward){
        motor->deltaEncoderTics(-1);
    }
    else {
        return;
    } */
} 


int Movement::getBackLeftEncoderTics(){
    return motor[static_cast<int>(MotorID::BACK_LEFT)].getEncoderTics();
}

int Movement::getFrontLeftEncoderTics(){
    return motor[static_cast<int>(MotorID::FRONT_LEFT)].getEncoderTics();
}

int Movement::getBackRightEncoderTics(){
    return motor[static_cast<int>(MotorID::BACK_RIGHT)].getEncoderTics();
}

int Movement::getFrontRightEncoderTics(){
    return motor[static_cast<int>(MotorID::FRONT_RIGHT)].getEncoderTics();
}

// no comentar
/* void Movement:: setSpeed(double targetSpeed){
    // El targetSpeed se multiplica por 1000 porque es una conversion de unidades de m/s a mm/s
    if (targetSpeed == 0) {
        stopMotors();
        return;
    }
    // TODO: checar como calcular el pwm 
    // compute(targetSpeed, currentSpeed, pwm, pidTics, 500, 1, false);



    switch (speedSign)
    {
    case (0):
        stopMotors();
        break;
    case (1):
        forwardMotors();
        break;
    case (-1):
    
        backwardMotors();
        break;
    }
} */


//no comentar
/* void Movement:: motorSpeedPID(double targetSpeed, bool debug){
    int speedSign = min(1, max(-1, targetSpeed * 1000));
    this->targetSpeed = fabs(targetSpeed);
    double tmpPwm = Motor::pwm;
    switch (speedSign)
    {
    case (0):
        stopMotors();
        break;
    case (1):
        forwardMotors();
        break;
    case (-1):

        backwardMotors();
        break;
    }    
} */

// TODO: comprobar que estos dos se implementen en el PID
/* 
void Movement::setSpeed(MotorID motorId, double targetSpeed){
    int index = static_cast<int>(motorId);
    if (index >= 0 && index < 4){
        motor[index].setSpeed(targetSpeed);
    }
}
*/

void Movement::setMotorSpeed(int leftSpeed, int rightSpeed) {
    motor[static_cast<int>(MotorID::FRONT_LEFT)].setPWM(leftSpeed);
    motor[static_cast<int>(MotorID::FRONT_RIGHT)].setPWM(rightSpeed);
    motor[static_cast<int>(MotorID::BACK_LEFT)].setPWM(leftSpeed);
    motor[static_cast<int>(MotorID::BACK_RIGHT)].setPWM(rightSpeed);

} 