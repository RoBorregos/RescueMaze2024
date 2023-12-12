
#include "Movement.h"
#include "Pines.h"
//#include "Encoder.h"


// Ajustar
// JALAN 2/3
constexpr double kP = 5.0; 
constexpr double kI = 0.008;
constexpr double kD = 0.0;

/* constexpr double kP = 5.0; 
constexpr double kI = 0.09;
constexpr double kD = 0.07; */

/* constexpr double kP = 120.0; 
constexpr double kI = 80;
constexpr double kD = 10.0; */

PID pidStraight(kP, kI, kD);
BNO bno;




Movement::Movement() {
    this-> motorFL = Motor();
    this-> motorFR = Motor();
    this-> motorBL = Motor();
    this-> motorBR = Motor();
    this->motor[4];

    this-> next_time = millis();

    this-> errorPrevOrientation = 0;
    this-> errorAcumuladoOrientation = 0;
}

void Movement::setup() {
    setupInternal(MotorID::FRONT_LEFT);
    setupInternal(MotorID::FRONT_RIGHT);
    setupInternal(MotorID::BACK_LEFT);
    setupInternal(MotorID::BACK_RIGHT);
    bno.setupBNO();
    //Encoder::initEncoder();
}

void Movement::setupInternal(MotorID motorId) {
    int index = static_cast<int>(motorId);
    motor[index].motoresSetup(
        pwmPin[index],
        digitalOne[index],
        digitalTwo[index],
        encoderA[index],
        motorId);
}

void Movement::moveForward(int pwmA, int pwmB, int pwmC, int pwmD) {
    motorFL.updateRPM();
    motorFR.updateRPM();
    motorBL.updateRPM();
    motorBR.updateRPM();
    motorFL.setPWM(pwmA);
    motorFR.setPWM(pwmB);
    motorBL.setPWM(pwmC);
    motorBR.setPWM(pwmD);

}

void Movement::setSpeed(float targetSpeed,float orientation,BNO bno) {
    //PID orientation
    float errorOrientation = orientation - bno.getOrientationX();

    if (errorOrientation > 300) {
        errorOrientation = orientation - (360 + bno.getOrientationX());
    }

    if (errorOrientation < -300) {
        errorOrientation = (bno.getOrientationX() - (360 + orientation)) * -1;
    }
    errorAcumuladoOrientation = errorAcumuladoOrientation + errorOrientation;
    //float targetSpeedRight = targetSpeed - (kP * errorOrientation + kI * (errorAcumuladoOrientation) + kD * (errorOrientation - errorPrevOrientation));
    float targetSpeedRight = - (kP * errorOrientation + kI * (errorAcumuladoOrientation) + kD * (errorOrientation - errorPrevOrientation));
    //float targetSpeedLeft = targetSpeed + (kP * errorOrientation + kI * (errorAcumuladoOrientation) + kD * (errorOrientation - errorPrevOrientation));
    float targetSpeedLeft = (kP * errorOrientation + kI * (errorAcumuladoOrientation) + kD * (errorOrientation - errorPrevOrientation));
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

    motorFL.setPID(targetSpeed+targetSpeedLeft, 1, 0.005, 0.006); //SEGUIR AJUSTANDO
    motorFR.setPID(targetSpeed+targetSpeedRight, 1, 0.005, 0.006);
    motorBL.setPID(targetSpeed+targetSpeedLeft, 1, 0.005, 0.006);
    motorBR.setPID(targetSpeed+targetSpeedRight, 1, 0.005, 0.006);

    Serial.print(motorFL.getRPM());
    Serial.print(" ");
    Serial.print(motorFR.getRPM());
    Serial.print(" ");
    Serial.print(motorBL.getRPM());
    Serial.print(" ");
    Serial.println(motorBR.getRPM());
}

Motor Movement::getMotorFL() {
    return motorFL;
}

Motor Movement::getMotorFR() {
    return motorFR;
}

Motor Movement::getMotorBL() {
    return motorBL;
}

Motor Movement::getMotorBR() {
    return motorBR;
}

float Movement::getRPMFL() {
    return motorFL.getRPM();
}

float Movement::getRPMFR() {
    return motorFR.getRPM();
}

float Movement::getRPMBL() {
    return motorBL.getRPM();
}

float Movement::getRPMBR() {
    return motorBR.getRPM();
}

void Movement::stopMotors() {
    
    for(int i=0; i<4; i++){
        motor[i].motorStop();
    }
}

void Movement:: forwardMotors(uint8_t pwms[4]) {
    for(int i=0; i<4; i++){
        motor[i].motorForward(pwms[i]);
    }
}

void Movement:: backwardMotors(uint8_t pwms[4]) {
    for(int i=0; i<4; i++){
        motor[i].motorBackward(pwms[i]);
    }
}
void Movement:: turnRight(uint8_t pwms[4]) {
    motor[0].motorBackward(pwms[0]);
    motor[1].motorBackward(pwms[1]);
    motor[2].motorBackward(pwms[2]);
    motor[3].motorBackward(pwms[3]);
}

void Movement:: turnLeft(uint8_t pwms[4]) {
    motor[0].motorForward(pwms[0]);
    motor[1].motorForward(pwms[1]);
    motor[2].motorForward(pwms[2]);
    motor[3].motorForward(pwms[3]);
}


void Movement:: moveMotors(MotorState state) {
    uint8_t pwms[4];
    int pwm = 60;
    double targetOrientation = 0;
    double currentOrientation = bno.getOrientationX();
    double pwmLeft = 0;
    double pwmRight = 0;
    switch (state)
    {
        case (MotorState::Stop): {
            stopMotors();
            break;
        }
        
        case (MotorState::Forward): {
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
        }
        case (MotorState::Backward): {
            pwms[0]= pwm;
            pwms[1]= pwm;
            pwms[2]= pwm;
            pwms[3]= pwm;
            backwardMotors(pwms);
            break;
        }
        default : {
            break;
        }
    }
}

// CALIZZ ''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''FEDFE'FREFEJ,FBEJFFBHUWBFUHEWBFHJWERBFHJRBEJHFBRJFBKWRJNSBJFBKREJFBKEJB
void Movement::updateTics(MotorID motorId) {

    int index = static_cast<int>(motorId);
    if (index >= 0 && index < 4) {
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


int Movement::getBackLeftEncoderTics() {
    return motor[static_cast<int>(MotorID::BACK_LEFT)].getEncoderTics();
}

int Movement::getFrontLeftEncoderTics() {
    return motor[static_cast<int>(MotorID::FRONT_LEFT)].getEncoderTics();
}

int Movement::getBackRightEncoderTics() {
    return motor[static_cast<int>(MotorID::BACK_RIGHT)].getEncoderTics();
}

int Movement::getFrontRightEncoderTics() {
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
        case (0):{
            stopMotors();
            break;
        }
        case (1):{
            forwardMotors();
            break;
        }
        case (-1):{
            backwardMotors();
            break;
        }
    }
} */


//no comentar
/* void Movement:: motorSpeedPID(double targetSpeed, bool debug){
    int speedSign = min(1, max(-1, targetSpeed * 1000));
    this->targetSpeed = fabs(targetSpeed);
    double tmpPwm = Motor::pwm;
    switch (speedSign)
    {
        case (0):{
            stopMotors();
            break;
        }
        case (1):{
            forwardMotors();
            break;
        }
        case (-1):{
            backwardMotors();
            break;
        }
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