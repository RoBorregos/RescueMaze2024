#include "Arduino.h"
#include "Motorclass.h"

motor::motor(byte pin1, byte pin2, byte pwm_pin){
    this->pin1 = pin1;
    this->pin2 = pin2;
    this->pwm_pin = pwm_pin;
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pwm_pin, OUTPUT);
}
void motor::forward(){
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
}
void motor::backward(){
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
}
void motor::stop(){
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
}
void motor::set_pwm(byte pwm){
    analogWrite(pwm_pin, pwm);
}