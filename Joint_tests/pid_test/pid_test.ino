
#include "Motorclass.h"
unsigned long next_time; 
motor FRmotor(5,4,3);
float rpm = 0;

int tick1 =0;
//496 ticks
void captureCallback(){
    //Serial.println(tick1);
    tick1++;
}
void get_vel(){
    Serial.print(tick1);
    Serial.print(" checking = ");
    rpm = (tick1*60.00)/496.00;
    Serial.println(rpm);
    tick1 = 0;
    
}
void setup(){
    Serial.begin(115200);
    pinMode(2,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2),captureCallback,RISING);
    next_time = millis();

}
void loop(){
    
    if(millis()-next_time>=1000){
        //Serial.print("1 second = ");
        //Serial.println(millis()-next_time);
        get_vel();
        next_time = millis();
    }

    FRmotor.forward();
    FRmotor.set_pwm(80);


}