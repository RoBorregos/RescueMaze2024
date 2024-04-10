#include "Motorclass.h"

// motor FLmotor(4,2,15);
// motor FRmotor(17,16,5);
// motor BRmotor(25,33,32);
// motor BLmotor(14,27,26);

motor FRmotor(17,16,5);//FR
motor FLmotor(14,27,26); //FL
motor BLmotor(33,25,32); //BL
motor BRmotor(2,4,15);//BR

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
    pinMode(39,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(39),captureCallback,RISING);
    pinMode(36,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(36),captureCallback_uno,RISING);
    pinMode(35,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(35),captureCallback_dos,RISING);
    pinMode(34,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(34),captureCallback_tres,RISING);
}
void loop (){
    for(int i = 0; i < 200; i++){
        Serial.println("forward");
        BLmotor.set_pwm(i);
        BLmotor.forward();
        BRmotor.set_pwm(i);
        BRmotor.forward();
        FRmotor.set_pwm(i);
        FRmotor.forward();
        FLmotor.set_pwm(i);
        FLmotor.forward();
        delay(100);
        Serial.print("tick1: ");
    Serial.print(tick1);
    Serial.print(" tick2: ");
    Serial.print(tick2);
    Serial.print(" tick3: ");
    Serial.print(tick3);
    Serial.print(" tick4: ");
    Serial.println(tick4);
    }
    // Serial.print("tick1: ");
    // Serial.print(tick1);
    // Serial.print(" tick2: ");
    // Serial.print(tick2);
    // Serial.print(" tick3: ");
    // Serial.print(tick3);
    // Serial.print(" tick4: ");
    // Serial.println(tick4);
    BLmotor.stop();
    BRmotor.stop();
    FRmotor.stop();
    FLmotor.stop();
    delay(2000);
     for(int i = 0; i < 200; i++){
        Serial.println("backward");
        // BLmotor.set_pwm(i);
        // BLmotor.backward();
        BRmotor.set_pwm(i);
        BRmotor.backward();
        FRmotor.set_pwm(i);
        FRmotor.backward();
        FLmotor.set_pwm(i);
        FLmotor.backward();
        delay(100);
        Serial.print("tick1: ");
    Serial.print(tick1);
    Serial.print(" tick2: ");
    Serial.print(tick2);
    Serial.print(" tick3: ");
    Serial.print(tick3);
    Serial.print(" tick4: ");
    Serial.println(tick4);
    }
    BLmotor.stop();
    BRmotor.stop();
    FRmotor.stop();
    FLmotor.stop();
    delay(2000);
}
