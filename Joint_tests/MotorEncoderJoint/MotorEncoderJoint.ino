#include "Motorclass.h"

motor FRmotor(23,22,5);
motor FLmotor(25,24,4);
motor BRmotor(26,27,7);
motor BLmotor(29,28,6);

void captureCallback(){
    Serial.println("Change");
}
void captureCallback_uno(){
    Serial.println("Change");
}
void captureCallback_dos(){
    Serial.println("Change");
}
void captureCallback_tres(){
    Serial.println("Change");
}
void setup (){
    Serial.begin(115200);
    pinMode(2,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2),captureCallback,RISING);
    pinMode(3,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(3),captureCallback_uno,RISING);
    pinMode(18,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(18),captureCallback_dos,RISING);
    pinMode(19,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(19),captureCallback_tres,RISING);
}
void loop (){
    BLmotor.set_pwm(50);
    BLmotor.forward();
    BRmotor.set_pwm(50);
    BRmotor.forward();
    FRmotor.set_pwm(50);
    FRmotor.forward();
    FLmotor.set_pwm(50);
    FLmotor.forward();
    delay(2000);
    BLmotor.stop();
    BRmotor.stop();
    FRmotor.stop();
    FLmotor.stop();
    delay(2000);
}
