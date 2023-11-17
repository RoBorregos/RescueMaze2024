#include "Arduino.h"
#include "Encoderclass.h"

encoder::encoder(byte pinA, byte pinB){
    this->pinA = pinA;
    this->pinB = pinB;
    pinMode(pinA,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA),callback_A,RISING);
    pinMode(pinB,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinB),callback_B,RISING);


}
volatile int encoder:: tick_A = 0;
volatile int encoder:: tick_B = 0;
float encoder::get_rpm(){
    float rpm = (this->tick_A*60.00)/496.00;
    this->tick_A = 0;
    return rpm;
}
void encoder::callback_A(){
    tick_A++;
}
void encoder::callback_B(){
    tick_B++;
}