#include "Motorclass.h"

motor FLmotor(26,27,8);
motor FRmotor(29,28,5);
motor BRmotor(23,22,7);
motor BLmotor(24,25,4);

int tick1 =0;
int tick2 =0;
int tick3 =0;
int tick4 =0;

void captureCallback(){
    tick1++;
}
void captureCallback_uno(){
    tick2++;
}
void captureCallback_dos(){
    tick3++;
}
void captureCallback_tres(){
    tick4++;
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
    BLmotor.set_pwm(150);
    BLmotor.forward();
    BRmotor.set_pwm(150);
    BRmotor.forward();
    FRmotor.set_pwm(150);
    FRmotor.forward();
    FLmotor.set_pwm(150);
    FLmotor.forward();
    delay(2000);
    Serial.print("tick1: ");
    Serial.print(tick1);
    Serial.print(" tick2: ");
    Serial.print(tick2);
    Serial.print(" tick3: ");
    Serial.print(tick3);
    Serial.print(" tick4: ");
    Serial.println(tick4);
    BLmotor.stop();
    BRmotor.stop();
    FRmotor.stop();
    FLmotor.stop();
    delay(2000);
}
