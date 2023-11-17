#include "Motorclass.h"

motor BRmotor(5,4,3);

void setup (){
    Serial.begin(115200);


}


void loop (){
  BRmotor.set_pwm(80);
  BRmotor.forward();
  delay(2000);
  BRmotor.stop();
  delay(2000);


}