#include "Motorclass.h"

motor FRmotor(16,17,5);
motor BRmotor(27,14,26);
motor FLmotor(33,25,32);
motor BLmotor(2,4,15);

int velocity = 40;

void setup (){
    Serial.begin(115200);
    delay(2000);

}


void loop (){
  BRmotor.set_pwm(velocity);
  BRmotor.forward();
  FRmotor.set_pwm(velocity);
  FRmotor.forward();
  BLmotor.set_pwm(velocity);
  BLmotor.forward();
  FLmotor.set_pwm(velocity);
  FLmotor.forward();
  delay(2000);
  BRmotor.stop();
  FRmotor.stop();
  BLmotor.stop();
  FLmotor.stop();
  delay(2000);
  BRmotor.set_pwm(velocity);
  BRmotor.backward();
  FRmotor.set_pwm(velocity);
  FRmotor.backward();
  BLmotor.set_pwm(velocity);
  BLmotor.backward();
  FLmotor.set_pwm(velocity);
  FLmotor.backward();
  delay(2000);
  BRmotor.stop();
  FRmotor.stop();
  BLmotor.stop();
  FLmotor.stop();
  delay(2000);


}

