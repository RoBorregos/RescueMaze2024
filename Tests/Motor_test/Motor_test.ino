#include "Motorclass.h"

motor BRmotor(22,23,7);

void setup (){
    Serial.begin(115200);


}


void loop (){
  BRmotor.set_pwm(90);
  BRmotor.forward();
  delay(2000);
  BRmotor.stop();
  delay(2000);
  BRmotor.set_pwm(90);
  BRmotor.backward();
  delay(2000);
  BRmotor.stop();
  delay(2000);


}