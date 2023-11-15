#include "Encoder.h"

void Encoder::updateTics(Motor *motor){
  motor->deltaPidTics(1);
  if(motor->getCurrentState() == MotorState::Forward){
    motor->deltaEncoderTics(1);
  }
  else if (motor->getCurrentState() == MotorState::Backward){
    motor->deltaEncoderTics(-1);
  }
  else {
    return;
  }
}
/*
void Encoder::backLeftEncoder() {
  updateTics(&robot->motor_[BACK_LEFT]);
}
  
void Encoder::frontLeftEncoder() {
  updateTics(&robot->motor_[FRONT_LEFT]);
}
  
void Encoder::backRightEncoder() {
  updateTics(&robot->motor_[BACK_RIGHT]);
}
  
void Encoder::frontRightEncoder() {
  updateTics(&robot->motor_[FRONT_RIGHT]);
}
*/
void captureCallBack()
{
    Serial.println(Encoder::backLeftEncoder()); 
}

voidp()
p{
    Serial.begin(9600);
    pinMode(19, INPUT);
    attachInterrupt(digitalPinToInterrupt(19), captureCallBack, RISING);

}
void loop()
{

}