#include "encoder.h"
#include "motor.h"

void encoder::updateTics(){ //Motor *motor
  motor->encoderTics();
  return;
}