#include "Motorclass.h"

motor BRmotor(22,23,2);

void setup (){
    Serial.begin(115200);


}


void loop (){
  BRmotor.set_pwm(30);
  BRmotor.forward();
  delay(2000);
  BRmotor.stop();
  delay(2000);


}