#include "Encoder.h"
#include "Pines.h"

/* void Encoder::updateTics(Motor *motor){
    motor->deltaPidTics(1);

    if (motor->getCurrentState() == MotorState::Forward){
        motor->deltaEncoderTics(1);
    } else if (motor->getCurrentState() == MotorState::Backward){
        motor->deltaEncoderTics(-1);
    }
    else {
        return;
    }
}  */

// descomentarizar

extern Movement robot;

void Encoder::backLeftEncoder() {
    robot.updateTics(MotorID::BACK_LEFT);
    //updateTics( &motor[MotorID::BACK_LEFT]);
}

void Encoder::backRightEncoder() {
    robot.updateTics(MotorID::BACK_RIGHT);
    //updateTics(Motor backRight);
}

void Encoder::frontLeftEncoder() {
    robot.updateTics(MotorID::FRONT_LEFT);
    //updateTics(&frontLeft);
}

void Encoder::frontRightEncoder() {
    robot.updateTics(MotorID::FRONT_RIGHT);
    //updateTics(&frontRight);
} 


void Encoder::initEncoder() {
    pinMode(encoderA[0], INPUT);
    pinMode(encoderA[1], INPUT);
    pinMode(encoderA[2], INPUT);
    pinMode(encoderA[3], INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderA[0]), backLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderA[1]), frontLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderA[2]), backRightEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderA[3]), frontRightEncoder, RISING);
}
/* void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10); // wait for serial port to open!
    Encoder::initEncoder();
    
}

void loop()
{
    Serial.print("BACK_LEFT: ");
    Serial.println(movement.getBackLeftEncoderTics());
    Serial.print("FRONT_LEFT: ");
    Serial.println(movement.getFrontLeftEncoderTics());
    Serial.print("BACK_RIGHT: ");
    Serial.println(movement.getBackRightEncoderTics());
    Serial.print("FRONT_RIGHT: ");
    Serial.println(movement.getFrontRightEncoderTics());
   
    
} */